//
// HoneyComm - Alternative RFBee firmware to communicate with
//             HR80 radiator controllers.
//
// Copyright (C) 2011 Wladimir Komarow
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "globals.h"
#include "CCx.h"
#include "CCxCfg.h"
#include "rfBeeSerial.h"
#include "SerialBitstream.h"
#include "ManchesterByteStream.h"

#define DEBUG 0
#define HEX_PRINT 0

#define PRS_INITIAL 0
#define PRS_PREPARE_RECEIVE 1
#define PRS_RECEIVING_PACKET 2

#define RSSI_OFFSET 74

#define PD2 2 // Wired with GDO0 on CC1101
#define SERIAL_DATA 0x04
#define PD3 3 // Wired with GDO2 on CC1101
#define SERIAL_CLK PD3

/* * * * Static functions * * * */
volatile word sync_buffer = 0;
volatile boolean sync_word_found = false;

volatile byte bit_counter = 0;
volatile byte byte_buffer = 0;
volatile byte byte_read   = 0;
byte byte_decoded = 0;
volatile boolean byte_ready = false;
volatile boolean searching_for_sync_word = true;

byte byte_counter = 0;
byte packet_length = 0;
boolean header = true;
boolean byte_one = true;
word packet_word;
int checksum;

#if DEBUG
byte debug_buffer[0x32];
#endif

enum processing_states
{
  SEARCHING_SYNC_WORD,
  FOUND_SYNC_WORD,
  READING_PREAMBLE_BYTE_1,
  READING_PREAMBLE_BYTE_2,
  READING_PREAMBLE_BYTE_3,
  READING_HEADER_1,
  READING_HEADER_2,
  READING_BODY_1,
  READING_BODY_2,
  READING_CRC_1,
  READING_CRC_2,
  READING_END_BLOCK
};

volatile int processing_state = SEARCHING_SYNC_WORD;


void find_sync_word(void) {
  if (processing_state == SEARCHING_SYNC_WORD)
  {
    sync_buffer = sync_buffer << 1;
    if (PIND & SERIAL_DATA)
    {
      sync_buffer++;
    }
    if (sync_buffer == 0xFC01)
    {
      processing_state++;

      byte_buffer = 0;
      bit_counter = 0;
      byte_ready = false;
    }
  }
  else
  {
    // 0123456789
    // S76543210S
    if (bit_counter > 0)
    {
      if (bit_counter < 9)
      {
        // read bit
        byte_buffer = byte_buffer >> 1;
        if (PIND & SERIAL_DATA)
        {
          byte_buffer = byte_buffer | 0x80;
        }
      }
      else
      {
         byte_read = byte_buffer;
         byte_ready = true;
         byte_buffer = 0;
         bit_counter = 0;
         return;
      }
      
    }
    bit_counter++;
  }
}

/* Source:
   https://github.com/jrosser/honeymon/blob/master/decoder.cpp
*/
char decodeManchester(byte manchester_byte)
{
    byte result = -1;

    switch(raw_byte)
    {
      case 0xAA: val=0x00; break;
      case 0xA9: val=0x01; break;
      case 0xA6: val=0x02; break;
      case 0xA5: val=0x03; break;
      case 0x9A: val=0x04; break;
      case 0x99: val=0x05; break;
      case 0x96: val=0x06; break;
      case 0x95: val=0x07; break;
      case 0x6A: val=0x08; break;
      case 0x69: val=0x09; break;
      case 0x66: val=0x0A; break;
      case 0x65: val=0x0B; break;
      case 0x5A: val=0x0C; break;
      case 0x59: val=0x0D; break;
      case 0x56: val=0x0E; break;
      case 0x55: val=0x0F; break;
    }

    return result;
}

void process_byte(byte a_byte) {
  byte error = 0;
  
  switch(processing_state)
  {
    case READING_PREAMBLE_BYTE_1:
      if (a_byte == 0x33)
      {
         processing_state++;
      }
      else
      {
        error = 0x01;
      }
      break;
    
    case READING_PREAMBLE_BYTE_2:
      if (a_byte == 0x55)
      {
         processing_state++;
      }
      else
      {
        error = 0x02;
      }
      break;
   
   case READING_PREAMBLE_BYTE_3:
      if (a_byte == 0x53)
      {
         processing_state++;
      }
      else
      {
        error = 0x03;
      }
      checksum = 0;
      break;
   
   case READING_HEADER_1:
     packet_word = a_byte;
     processing_state++;
     return;
     break;
   
   case READING_HEADER_2:
       packet_word |= a_byte << 8;
       if ( !manchesterByteStream.decodeByte((byte*)&packet_word, &a_byte) )
       {
         checksum += a_byte;
         if (byte_counter == 0)
         {
           // Header byte
           switch(a_byte)
           {
             case 0x18:
               // <header: 1 byte><id1:3 bytes><id2:3 bytes><command:2 bytes><length:1 byte> == 10 bytes
               packet_length = 9;
                break;
             case 0x1C:
               // <header: 1 byte><id1:3 bytes><id2:3 bytes><command:2 bytes><length:1 byte>
               packet_length = 9;
               break;
             default:
               Serial.print(a_byte, HEX);
               error = 0x04;
           }
           processing_state--;
         }
         else
         {
           if (byte_counter == packet_length)
           {
             packet_length += a_byte;
             processing_state++;
           }
           else
           {
             processing_state--;
           }
         }
#if HEX_PRINT
         if (a_byte < 0x10) Serial.print(F("0"));
         Serial.print(a_byte, HEX);
#else
         Serial.write(a_byte);
#endif
         byte_counter++;
       }
       else
       {
         // Decoding error, terminate packet processing
         //Serial.print(a_byte, HEX);
         error = 0x05;
       }
     break;
   
   case READING_BODY_1:
     packet_word = a_byte;
     processing_state++;
     return;
     break;
   
   case READING_BODY_2:
     packet_word |= a_byte << 8;
     if ( !manchesterByteStream.decodeByte((byte*)&packet_word, &a_byte) )
     {
       checksum += a_byte;
       if (byte_counter == packet_length)
       {
         processing_state++;
       }
       else
       {
         processing_state--;
       }
#if HEX_PRINT
       if (a_byte < 0x10) Serial.print(F("0"));
       Serial.print(a_byte, HEX);
#else
       Serial.write(a_byte);
#endif
       byte_counter++;
     }
     else
     {
       // Decoding error, terminate packet processing
       //Serial.print(a_byte, HEX);
       error = 0x06;
     }
     break;

   case READING_CRC_1:
     packet_word = a_byte;
     processing_state++;
     return;
     break;
   
   case READING_CRC_2:
     packet_word |= a_byte << 8;
     if ( !manchesterByteStream.decodeByte((byte*)&packet_word, &a_byte) )
     {
       checksum += a_byte;
       if ((checksum & 0xFF) != 0)
       {
         error = 0x07; // Checksum error
       }
       processing_state++;
     }
     else
     {
       // Decoding error, terminate packet processing
       //Serial.print(a_byte, HEX);
       error = 0x08;
     }
#if HEX_PRINT
     if (a_byte < 0x10) Serial.print(F("0"));
     Serial.print(a_byte, HEX);
#else
     Serial.write(a_byte);
#endif
     byte_counter++;
     break;
     
   case READING_END_BLOCK:
     if (a_byte == 0x35)
     {
       // Succesfully read a complete packages
       error = 0xff;
     }
     else
     {
       error = 0x09;
     }
     break;
  }

#if DEBUG
  debug_buffer[byte_counter-1] = a_byte;
#endif
  
  //Serial.write(a_byte);
  
  if (error != 0x00)
  {
#if DEBUG
  for (int i = 0 ; i != (packet_length+2) ; i++)
  {
    if (debug_buffer[i] < 0x10) Serial.print(F("0"));
    Serial.print(debug_buffer[i], HEX);
    Serial.print(F(" "));
  }
#endif
    if (error == 0xFF)
    {
      processing_state = SEARCHING_SYNC_WORD;
      Serial.print("+++;");
      byte_counter = 0;
      packet_length = 0;
    }
    else
    {
      processing_state = SEARCHING_SYNC_WORD;
      Serial.print("+++E");
      Serial.print(error, HEX);
      Serial.print(";");
      byte_counter = 0;
      packet_length = 0;
    }
  }
}

void setup() {

  delay(500);

  CCx.PowerOnStartUp();
  setCCxConfig();

  Serial.begin(115200);

  //Serial.println(F("-- HoneyCommQS - v1.0"));
  delay(500);

  sync_buffer = 0;
  sync_word_found = false;
  //Serial.println(F("Successfully started"));
  attachInterrupt(1, find_sync_word, FALLING);
}

//
// Main loop
//
void loop() {
  if (processing_state == FOUND_SYNC_WORD)
  {
    //Serial.print(F("F"));
    sync_buffer = 0;
    processing_state++;
  }
  if (byte_ready)
  {
    byte_ready = false;
    process_byte(byte_read);
  }
}













