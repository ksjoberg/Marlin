/*
  modbus.cpp - MODBUS interface
  
  Copyright (c) 2016 Kristoffer Sjoberg

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Marlin.h"
#include "modbus.h"
#if ENABLED(MODBUS_CONTROL)
#undef HardwareSerial_h
#include "HardwareSerial.h"

//extern HardwareSerial SerialModbus;
#define SERIAL Serial2

#define MODBUS_TXEN 32
#define modbus_transmitenable() digitalWrite(MODBUS_TXEN, HIGH)
#define modbus_transmitdisable() digitalWrite(MODBUS_TXEN, LOW)

/*
ISR(USART2_TXC0_vect)
{
  
  cbi(UCSR2B, TXCIE0);
}
*/

#define VFD_RESPONSE_TIMEOUT  10000
//#define MODBUS_LOCALECHO

// Huanyang VFD
// 0x01 0x03 0x01 0x08 0xF1 0x8E
// Slave address, function code, request length, stop, CRC16

// Function codes:
// 
// FunctionRead = 0x01,
// FunctionWrite = 0x02,
// WriteControlData = 0x03,
// ReadControlData = 0x04,
// WriteInverterFrequencyData = 0x05,




//===========================================================================
//============================= public variables ============================
//===========================================================================


//===========================================================================
//============================= private variables ===========================
//===========================================================================
//static makes it impossible to be called from outside of this file by extern.!

// Huanyang VFD Registers:
// PD005 - Frequency Upper Limit
// PD011 - Frequency Lower Limit
// PD144 - RPM @ 50Hz (typically 3000)
static uint16_t pd005 = 0;
static uint16_t pd011 = 0;
static uint16_t pd144 = 0;

static millis_t last_command = 0;

//===========================================================================
//================================ functions ================================
//===========================================================================

FORCE_INLINE static uint16_t modbus_calculatecrc_step(uint16_t state, uint8_t b)
{
  state ^= b;
  for(uint8_t j = 0; j<8; j++)
  {
    if (state & 1)
      state = (state>>1) ^ 0xA001;
    else
      state >>= 1;
  }
  return state;
}

static uint16_t modbus_calculatecrc(const char* data, uint8_t len)
{
  uint16_t r = 0xffff;
  while(len--)
  {
    r = modbus_calculatecrc_step(r, *(data++));
  }
  return r; // take lower byte first, then higher
}

static void modbus_clear()
{
  uint8_t written = 0;
  while (SERIAL.available() > 0)
  {
    if (!written)
    {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("MODBUS Extra: ");
      written = 1;
    }
    uint8_t c = SERIAL.read();
    MYSERIAL.print(c, HEX);
    MYSERIAL.print(" ");
  }
  if (written)
  {
    SERIAL_EOL;
  }
}
void modbus_init() {
  pinMode(MODBUS_TXEN, OUTPUT);
  SERIAL.begin(9600, SERIAL_8N1);
  pinMode(17, INPUT_PULLUP); // Serial2 RX pull-up on.
  modbus_clear();
  
  pd005 = spindlevfd_readregister(VFD_ADDRESS, 5); // Frequency Upper Limit
  // 0x01 0x01 0x03 0x05 0x9C 0x40 0x44 0xBF

  pd011 = spindlevfd_readregister(VFD_ADDRESS, 11); // Frequency Lower Limit
  // 0x01 0x01 0x03 0x0B 0x00 0x00 0x4D 0x8C

  pd144 = spindlevfd_readregister(VFD_ADDRESS, 144); // RPM @ 50Hz
  // 0x01 0x01 0x03 0x90 0x0B 0xB8 0x3B 0x21


  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("MODBUS PD005: ");
  MYSERIAL.print(pd011, DEC);
  SERIAL_EOL;

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("MODBUS PD011: ");
  MYSERIAL.print(pd005, DEC);
  SERIAL_EOL;

  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("MODBUS PD144: ");
  MYSERIAL.print(pd144, DEC);
  SERIAL_EOL;

  uint8_t c = spindlevfd_readcontrol(VFD_ADDRESS);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM("MODBUS CNTRL: ");
  MYSERIAL.print(c, HEX);
  SERIAL_EOL;

  

}


void modbus_sendwait()
{
    //sbi(UCSR2B, TXCIE0);
    SERIAL.flush();
    modbus_transmitdisable();
}


/**
 * Block until all buffered steps are executed
 */
int modbus_send(uint8_t address, uint8_t func, const uint8_t* msg, uint8_t len, uint8_t* rcvmsg, uint8_t rcvlen) {
  uint16_t crc = 0xFFFF;
  uint8_t rcvstate = 0;
  uint8_t state;
  #if ENABLED(MODBUS_LOCALECHO)
    state = 0;
  #else
    state = 1;
  #endif
  uint8_t allok = 0;

  millis_t timeSinceLastCommand = millis() - last_command;
  if (timeSinceLastCommand < 50)
  {
    delay(50 - timeSinceLastCommand);
  }
  last_command = millis();
  
  modbus_clear();
  modbus_transmitenable();
  crc = modbus_calculatecrc_step(crc, address);
  SERIAL.write(address);

  crc = modbus_calculatecrc_step(crc, func);
  SERIAL.write(func);

  crc = modbus_calculatecrc_step(crc, len);
  SERIAL.write(len);

  for(uint8_t i=0; i<len; i++)
  {
    crc = modbus_calculatecrc_step(crc, msg[i]);
    SERIAL.write(msg[i]);
  }

  SERIAL.write(crc&0xff);
  SERIAL.write(crc>>8);
  modbus_sendwait();
  // Transmit Disable is done via interrupt

  int incoming;
  char c;
  millis_t start = millis();
  while (!allok) {
    while((SERIAL.available() == 0) && (millis()-start < VFD_RESPONSE_TIMEOUT));
    
    if (SERIAL.available() == 0)
      break;
      
    incoming = SERIAL.read();
    #if ENABLED(MODBUS_LOCALECHO)
    if (state == 0)
    {
      // Validate proper echo
      if ((rcvstate == 0) && (incoming == address))
      {
        rcvstate++;
      } else if ((rcvstate == 1) && (incoming == func))
      {
        rcvstate++;
      } else if ((rcvstate == 2) && (incoming == len))
      {
        rcvstate++;
      } else if ((rcvstate > 2) && (rcvstate < (3+len)) && (incoming == msg[rcvstate-3]))
      {
        rcvstate++;
      } else if ((rcvstate == 3+len) && (incoming == (crc&0xff)))
      {
        rcvstate++;
      } else if ((rcvstate == 4+len) && (incoming == (crc>>8)))
      {
        state = 1;
        if (rcvmsg == NULL)
        {
          allok = 1;
        } else {
          rcvstate = 0;
          crc = 0xffff;
        }
      } else {
        break;
      }
    } else
      // Echo was valid, now extra data!
    #endif
    if (state == 1) {
      if ((rcvstate == 0) && (incoming == address))
      {
        crc = 0xffff;
        crc = modbus_calculatecrc_step(crc, incoming);
        rcvstate++;
      } else if ((rcvstate == 1) && (incoming == func))
      {
        crc = modbus_calculatecrc_step(crc, incoming);
        rcvstate++;
      } else if (rcvstate == 2)
      {
        len = incoming;
        crc = modbus_calculatecrc_step(crc, incoming);
        rcvstate++;
      } else if ((rcvstate > 2) && (rcvstate < (3+len)))
      {
        crc = modbus_calculatecrc_step(crc, incoming);
        if (rcvmsg && (rcvstate-3 < rcvlen))
        {
          rcvmsg[rcvstate-3] = incoming;
        }
        rcvstate++;
      } else if ((rcvstate == 3+len) && (incoming == (crc&0xff)))
      {
        rcvstate++;
      } else if ((rcvstate == 4+len) && (incoming == (crc>>8)))
      {
        state = 2;
        allok = 1;
      } else {
        // incoming byte was expected at this rcvstate
        break;
      }
    }
  }

  return allok ? len : -1;
}


uint16_t spindlevfd_readregister(uint8_t address, uint8_t reg)
{
  uint8_t data[3];
  data[0] = reg;
  data[1] = data[2] = 0x00;
  if (modbus_send(address, 0x01, data, 3, data, 3) > 0) {
    return (((uint16_t)data[1])<<8) | data[2];
  }
  return 0;
}

void spindlevfd_writeregister(uint8_t address, uint8_t reg, uint16_t value)
{
  uint8_t data[3];
  data[0] = reg;
  data[1] = value >> 8; 
  data[2] = value & 0x0ff;
  modbus_send(address, 0x02, data, 3, NULL, 0);
  return;
}

uint8_t spindlevfd_readcontrol(uint8_t address)
{
  uint8_t data[3];
  data[0] = 0;
  if (modbus_send(address, 0x04, data, 1, data, 3) > 0) {
    return data[0];
  }
  return 0;
}

void spindlevfd_writecontrol(uint8_t address, uint8_t control)
{
  modbus_send(address, 0x03, &control, 1, NULL, 0);
  return;
}

void spindlevfd_setfrequency(uint8_t address, uint16_t freq)
{
  uint8_t data[2];

  // bounds check
  if (freq < pd011)
    freq = pd011;
  else if (freq > pd005)
    freq = pd005;
  
  data[0] = freq >> 8;
  data[1] = freq & 0xff;
  
  modbus_send(address, 0x05, data, 2, NULL, 0);
  return;
}

void spindlevfd_setrpm(uint8_t address, uint16_t rpm)
{
  uint32_t temp = rpm;
  temp *= 50 * 100;
  temp /= pd144;
  spindlevfd_setfrequency(address, temp);
  return;
}



#endif
