#ifndef MODBUSSETTINGS_H_
#define MODBUSSETTINGS_H_

#include "DataTypes.h"
#include "Serial.h"


// Device settings (DSP f28335)

// Change this according to your DSP
#define CPU_FREQ			150
//#define CPU_FREQ            100
//#define LOW_SPEED_CLOCK 	25000000
//#define LOW_SPEED_CLOCK 	15000000
#define LOW_SPEED_CLOCK     37500000
// Serial settings ========================================
#define SERIAL_BAUDRATE 	115200
#define SERIAL_PARITY 		SERIAL_PARITY_NONE
#define SERIAL_BITS_NUMBER 	8

#define SERIAL_START_STOP_NUMBER_BITS	1
#if SERIAL_PARITY == SERIAL_PARITY_NONE
#define SERIAL_PARITY_NUMBER_BITS		0
#else
#define SERIAL_PARITY_NUMBER_BITS		1
#endif


#endif /* MODBUSSETTINGS_H_ */
