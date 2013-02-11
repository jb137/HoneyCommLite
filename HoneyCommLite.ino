//
// HoneyCommLite - Alternative RFBee firmware to communicate with
//                 Evohome / Hometronix / CM67z and other Honeywell 868MHz based RF devices.
//
// Copyright (C) 2012 JB137
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

// Compile for RFbee using board: Arduino Pro or Pro Mini (3.3V, 8MHz) w/ATmega 168

#include "globals.h"
#include "CCx.h"
#include "CCxCfg.h"
#include "rfBeeSerial.h"

#define DEBUG 0      // Prints complete packets if turn on in HEX format
#define HEX_PRINT 0  // Prints all data in HEX, too slow for real processing

#define SERIAL_DATA   4 // INT(0) PD2, wired to GDO0 on CC1101
#define SERIAL_CLK    1 // INT(1) PD3, wired to GDO2 on CC1101

// We are looking for FF00, the data stream however
// includes start/stop bits:
// [0]10101010[1][0]10101010[1][0]11111111[1][0]00000000[1]
// 
// The last 32 bits are:
// 0[1][0]10101010[1][0]11111111[1][0]00000000[1]
//   
// Equals: 0xFC01
//#define SYNC_WORD     0x5557FC01
#define SYNC_WORD     0xFC01

// Processing states
// For HEADER/BODY/CRC bytes are manchester encoded
// Part _2 of these states decodes and processes the data byte
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

// Variables used in interrupt functions
volatile word     sync_buffer       = 0; // LIFO buffer to search for sync word
volatile byte     bit_counter       = 0; // Counts the bits coming in to build a byte
volatile byte     byte_buffer       = 0; // Buffer used to gather the data bits coming in
volatile byte     byte_read         = 0; // Filled with a data byte once complete
volatile boolean  byte_ready        = false; // Set when a complete byte has been reveiced 
volatile int      processing_state = SEARCHING_SYNC_WORD; // Holds the current state of the processing


// Variables used to hold the processing state
byte    byte_counter   = 0; // Used to beep track of data bytes inside packet
byte    packet_length  = 0; // Holds the packetlength
byte    first_nible    = 0; // Buffer to hold manchester data
int     checksum       = 0; // Holds to checksum while processing packet bytes

#if DEBUG
byte debug_buffer[0x32]; // Buffer complete packages in DEBUG mode
#endif

// Interrupt function find_sync_word
// Depending on the processing_state this funcion either
// Searches for the sync word using sync_buffer as LIFO or
// Reads data bytes from the bit stream

void find_sync_word(void) {
  if (processing_state == SEARCHING_SYNC_WORD)
  {
    sync_buffer = sync_buffer << 1;
    if (PIND & SERIAL_DATA)
    {
      sync_buffer++;
    }
    if (sync_buffer == SYNC_WORD)
    {
      // The sync word has been detected, proceed to next state
      processing_state++;
      
      // Reset variables used in byte receiving process
      byte_buffer = 0;
      bit_counter = 0;
      byte_ready = false;
    }
  }
  else
  {
    // Incoming bits: 0123456789
    //                S76543210S
    if (bit_counter > 0) // Skip start bit
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
      {  // Last bit has been received
         
         // Move byte into byte_read
         byte_read = byte_buffer;
         
         // Reset variables used in byte receiving process
         byte_buffer = 0;
         bit_counter = 0;
         
         // Set byte ready indicator
         byte_ready = true;

         return;
      }
      
    }
    bit_counter++;
  }
}

/* Source:
   https://github.com/jrosser/honeymon/blob/master/decoder.cpp
*/
byte decodeManchester(byte manchester_byte)
{
    int result = 0xFF;

    switch(manchester_byte)
    {
      case 0xAA: result = 0x00; break;
      case 0xA9: result = 0x01; break;
      case 0xA6: result = 0x02; break;
      case 0xA5: result = 0x03; break;
      case 0x9A: result = 0x04; break;
      case 0x99: result = 0x05; break;
      case 0x96: result = 0x06; break;
      case 0x95: result = 0x07; break;
      case 0x6A: result = 0x08; break;
      case 0x69: result = 0x09; break;
      case 0x66: result = 0x0A; break;
      case 0x65: result = 0x0B; break;
      case 0x5A: result = 0x0C; break;
      case 0x59: result = 0x0D; break;
      case 0x56: result = 0x0E; break;
      case 0x55: result = 0x0F; break;
    }

    return result;
}

// Process a received byte
void process_byte(byte a_byte) {
  byte error = 0; // If the error is set to a non-zero value the processing of a packet will be terminated
  
  switch(processing_state)
  {
    case READING_PREAMBLE_BYTE_1:
      if (a_byte == 0x33)
      {  // Correct preamble byte 1
         processing_state++;
      }
      else
      {
        error = 0x01;
      }
      break;
    
    case READING_PREAMBLE_BYTE_2:
      if (a_byte == 0x55)
      {  // Correct preamble byte 2
         processing_state++;
      }
      else
      {
        error = 0x02;
      }
      break;
   
   case READING_PREAMBLE_BYTE_3:
      if (a_byte == 0x53)
      {  // Correct preamble byte 3
         processing_state++;
      }
      else
      {
        error = 0x03;
      }
      checksum = 0;
      break;
   
   case READING_HEADER_1:
     // Stored the first part of the manchester encoded byte
     first_nible = decodeManchester(a_byte);
     if (first_nible > 0x0F)
     {
       error = 0x04;
     }
     processing_state++;
     return; // Skip further processing
     break;
   
   case READING_HEADER_2:
     // Store second part of the manchester encoded byte
     a_byte = decodeManchester(a_byte);
     if (a_byte <= 0x0F)
     {
       a_byte = (first_nible << 4) | a_byte;
       checksum += a_byte; // update checksum
       if (byte_counter == 0)
       {
         // The header byte (first byte) is used to determine the length
         // of the packet up to the length byte, or in other words the position
         // of the length byte
         switch(a_byte)
         {
           case 0x18:
             // <header: 1 byte><id1:3 bytes><id2:3 bytes><command:2 bytes><length:1 byte> == 10 bytes
             packet_length = 9; // counting starts at zero
              break;
           case 0x1C:
             // <header: 1 byte><id1:3 bytes><id2:3 bytes><command:2 bytes><length:1 byte> == 10 bytes
             packet_length = 9;
             break;
           default:
             // Set length to longest header
             // <header: 1 byte><id0:3 bytes><id1:3 bytes><id2:3 bytes><p0:1 bytes><p1:1 bytes><command:2 bytes><length:1 byte> == 15 bytes
             packet_length = 15;
             //Serial.print(a_byte);
             //error = 0x05;
         }
         processing_state--; // Return to processing the first part of the next manchester encoded byte
       }
       else
       {
         if (byte_counter == packet_length)
         { // This is the packet length byte
           // Add the value to packet_length
           packet_length += a_byte;
           processing_state++; // Go to the next state, processing the body bytes
         }
         else
         {
           processing_state--; // Return to processing the first part of the next manchester encoded byte
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
       error = 0x06;
     }
     break;
   
   case READING_BODY_1:
     first_nible = decodeManchester(a_byte);
     if (first_nible > 0x0F)
     {
       error = 0x07;
     }
     processing_state++;
     return;
     break;
   
   case READING_BODY_2:
     a_byte = decodeManchester(a_byte);
     if (a_byte <= 0x0F)
     {
       a_byte = (first_nible << 4) | a_byte;
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
       error = 0x08;
     }
     break;

   case READING_CRC_1:
     first_nible = decodeManchester(a_byte);
     if (first_nible > 0x0F)
     {
       error = 0x09;
     }
     processing_state++;
     return;
     break;
   
   case READING_CRC_2:
     a_byte = decodeManchester(a_byte);
     if (a_byte <= 0x0F)
     {
       a_byte = (first_nible << 4) | a_byte;
       checksum += a_byte;
       if ((checksum & 0xFF) != 0)
       {
         error = 0x0A; // Checksum error
       }
       processing_state++;
     }
     else
     {
       // Decoding error, terminate packet processing
       //Serial.print(a_byte, HEX);
       error = 0x0B;
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
       error = 0xFF;
     }
     else
     {
       error = 0x0C;
     }
     break;
  }

#if DEBUG
  debug_buffer[byte_counter-1] = a_byte;
#endif
    
  if (error != 0x00)
  {
#if DEBUG
    // Print the complete packet in human readable form
    for (int i = 0 ; i != (packet_length+2) ; i++)
    {
      if (debug_buffer[i] < 0x10) Serial.print(F("0"));
      Serial.print(debug_buffer[i], HEX);
      Serial.print(F(" "));
    }
#endif
    if (error == 0xFF)
    {
      // Start looking for the sync word again
      processing_state = SEARCHING_SYNC_WORD;
      // Print end block +++;
      // Split using /(.*?(?:\+{3}(?:E.)?;))/m
      Serial.print("+++;");
      
      // Reset to get ready for next packet
      byte_counter = 0;
      packet_length = 0;
    }
    else
    {
      // Start looking for the sync word again
      processing_state = SEARCHING_SYNC_WORD;
      
      // Print end block with error code +++E.;
      // Split using /(.*?(?:\+{3}(?:E.)?;))/m
      // Error can be extracted using /\+{3}(E.)?;/m
      Serial.print("+++E");
      Serial.print(error, HEX);
      Serial.print(";");
      
      // Reset to get ready for next packet
      byte_counter = 0;
      packet_length = 0;
    }
  }
}

// Setup
void setup() {
  delay(500); // Is this needed?

  // Power up and configure the CC1101
  CCx.PowerOnStartUp();
  setCCxConfig();
  
  // Data is received at 38k4 (packet bytes only at 19k2 due to manchster encoding)
  // 115k2 provides enough speed to perform processing and write the received
  // bytes to serial
  Serial.begin(115200);
 
  delay(500); // Is this needed?

  // Get ready to find the sync word
  sync_buffer      = 0;
  processing_state = SEARCHING_SYNC_WORD;

  // Attach the find_sync_word interrupt function to the
  // falling edge of the serial clock connected to INT(1)
  attachInterrupt(SERIAL_CLK, find_sync_word, FALLING);
}

// Main loop
void loop() {
  if (processing_state == FOUND_SYNC_WORD)
  { // Found sync word, reset sync buffer and on to the next state
    sync_buffer = 0;
    processing_state++;
  }
  if (byte_ready)
  { // Complete byte has been read from the incoming stream
    byte_ready = false;
    process_byte(byte_read);
  }
}













