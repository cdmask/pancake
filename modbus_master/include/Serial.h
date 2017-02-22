#ifndef MODBUS_SERIAL_H_
#define MODBUS_SERIAL_H_

#include "DataTypes.h"
#include "DSP2833x_Sci.h"

typedef struct Serial Serial;

// Parity constants
typedef enum {
	SERIAL_PARITY_NONE,
	SERIAL_PARITY_EVEN,
	SERIAL_PARITY_ODD
} SerialParity;

typedef enum {
    SCI_A,
    SCI_B,
    SCI_C
} SCIPort;


struct Serial {

    SCIPort Port;
    volatile struct  SCI_REGS *SciRegs;
    Uint16  bitsNumber;
	Uint16  parityType;
	Uint32  baudrate;
	Uint16  fifoWaitBuffer;
// These are function pointers, which
	void (*clear)(Serial *self);
	Uint16 (*rxBufferStatus)(Serial *self);
	void (*setSerialRxEnabled)(Serial *self,bool status);
	void (*setSerialTxEnabled)(Serial *self,bool status);
	void (*init)(Serial *self);
	void (*transmitData)(Serial *self,Uint16 * data, Uint16 size);
	Uint16 (*getRxBufferedWord)(Serial *self);
	bool (*getRxError)(Serial *self);
};

void serial_clear(Serial *self);
inline Uint16 serial_rxBufferStatus(Serial *self);
inline void serial_setSerialRxEnabled(Serial *self,bool status);
inline void serial_setSerialTxEnabled(Serial *self,bool status);
inline void serial_init(Serial *self);
void serial_transmitData(Serial *self,Uint16 * data, Uint16 size);
inline Uint16 serial_getRxBufferedWord(Serial *self);
inline bool serial_getRxError(Serial *self);
//define a function that returns a struct(Serial) type
Serial construct_Serial(SCIPort port,Uint32 baudrate,Uint16 bitsNumber);

#endif
