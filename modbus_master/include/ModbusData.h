#ifndef MODBUS_DATA_RESPONSE_H_
#define MODBUS_DATA_RESPONSE_H_

#include "ModbusSettings.h"

typedef struct ModbusData ModbusData;                      // define ModbusData as a structure datatype

struct ModbusData {
	Uint16 slaveAddress;
	Uint16 functionCode;
	Uint16 contentIdx;
	Uint16 content[MB_BUFFER_SIZE];
	Uint16 size;
	Uint16 crc;

	void (*clear)(ModbusData *self);                            //what is this?
	Uint16 * (*getTransmitString)(ModbusData *self);            //function inside structure?
	Uint16 * (*getTransmitStringWithoutCRC)(ModbusData *self);
};

inline void data_clear(ModbusData *self);
inline Uint16 * data_getTransmitString(ModbusData *self);
inline Uint16 * data_getTransmitStringWithoutCRC(ModbusData *self);
ModbusData construct_ModbusData();

#endif
