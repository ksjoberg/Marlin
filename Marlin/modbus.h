/*
  modbus.h - MODBUS interface
  
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

#ifndef modbus_h
#define modbus_h
#include "Marlin.h"

#if ENABLED(MODBUS_CONTROL)

// Initialize and start the stepper motor subsystem
void modbus_init();

// Block until all buffered steps are executed
int modbus_send(uint8_t address, uint8_t func, const uint8_t* msg, uint8_t len, uint8_t* rcvmsg, uint8_t rcvlen);


#define SPINDLEVFD_CFG_SETF  0
#define SPINDLEVFD_CFG_OUTF  1
#define SPINDLEVFD_CFG_OUTA  2
#define SPINDLEVFD_CFG_ROTT  3
#define SPINDLEVFD_CFG_DCV   4
#define SPINDLEVFD_CFG_ACV   5
#define SPINDLEVFD_CFG_CONT  6
#define SPINDLEVFD_CFG_TMP   7

uint16_t spindlevfd_readregister(uint8_t address, uint8_t reg);
void spindlevfd_writeregister(uint8_t address, uint8_t reg, uint16_t value);
uint16_t spindlevfd_readcontrol(uint8_t address, uint8_t cfg);
void spindlevfd_writecontrol(uint8_t address, uint8_t control);
void spindlevfd_setfrequency(uint8_t address, uint16_t freq);
void spindlevfd_setrpm(uint8_t address, uint16_t rpm);
void spindlevfd_kill(uint8_t address);

#endif

#endif
