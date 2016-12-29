#ifndef MODBUS_MASTER_H_
#define MODBUS_MASTER_H_

#include "ModbusDefinitions.h"
#include "ModbusData.h"
#include "Serial.h"
#include "Timer.h"
#include "Crc.h"
#include MB_DATA_MAP

//typedef struct ModbusMaster ModbusMaster;

#include "ModbusRequester.h"

struct ModbusMaster {
	ModbusMasterState state;

	ModbusData dataRequest;
	ModbusData dataResponse;

	ModbusRequester requester;

	Serial serial;
	Timer timer;

#if MB_COILS_ENABLED
	ModbusCoilsMap coils;
#endif
#if MB_INPUTS_ENABLED
	ModbusInputsMap inputs;
#endif
#if MB_HOLDING_REGISTERS_ENABLED
	ModbusHoldingRegistersMap holdingRegisters;
#endif
#if MB_INPUT_REGISTERS_ENABLED
	ModbusInputRegistersMap inputRegisters;
#endif

	Uint16 timeoutCounter;
	Uint16 successfulRequests;
	bool timeout;
	bool requestReady;
	bool requestProcessed;

	void (*loopStates)(ModbusMaster *self);
	void (*create)(ModbusMaster *self);
	void (*start)(ModbusMaster *self);
	void (*wait)(ModbusMaster *self);
	void (*request)(ModbusMaster *self);
	void (*receive)(ModbusMaster *self);
	void (*process)(ModbusMaster *self);
	void (*destroy)(ModbusMaster *self);
};

void master_loopStates(ModbusMaster *self);
inline void master_create(ModbusMaster *self);
inline void master_start(ModbusMaster *self);
inline void master_wait(ModbusMaster *self);
inline void master_receive(ModbusMaster *self);
inline void master_process(ModbusMaster *self);
inline void master_destroy(ModbusMaster *self);
inline void master_request(ModbusMaster *self);
ModbusMaster construct_ModbusMaster();

extern ModbusMaster mb;

#endif
