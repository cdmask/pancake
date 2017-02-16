#include "DSP2833x_Device.h"     // DSP2833x Header file Include File
#include "Serial.h"
#include "Log.h"
#include "ModbusSettings.h"
/////////////////////////////////////////////
// SciaRegs changed into ScicRegs
/////////////////////////////////////////////
// Clear flags of overflow
void serial_clear(){
	static unsigned short i, destroyFifo;

	SERIAL_DEBUG();

	// Reset Serial in case of error
	if(ScicRegs.SCIRXST.bit.RXERROR == true){
		ScicRegs.SCICTL1.bit.SWRESET=0;
	}

	// Clears FIFO buffer (if there is any data)
	for (i = ScicRegs.SCIFFRX.bit.RXFFST; i > 0; i--)
		destroyFifo = ScicRegs.SCIRXBUF.all;

	// Reset FIFO
	ScicRegs.SCIFFRX.bit.RXFIFORESET=1;
	ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;

	ScicRegs.SCICTL1.bit.SWRESET=1;

}

// Get how much data is at the RX FIFO Buffer
Uint16 serial_rxBufferStatus(){
	return ScicRegs.SCIFFRX.bit.RXFFST;
}

// Enable or disable RX (receiver)
void serial_setSerialRxEnabled(bool status){
	SERIAL_DEBUG();
	ScicRegs.SCICTL1.bit.RXENA = status;
}

// Enable or disable TX (trasmiter)
void serial_setSerialTxEnabled(bool status){
	SERIAL_DEBUG();
	ScicRegs.SCICTL1.bit.TXENA = status;

}
// Note: if you changed SCI port,you need to change the SCI register structure
// Initialize Serial (actually SCIA)
void serial_init(Serial *self){
	Uint32 baudreg; // SCI Baud-Select register value

	// START: GOT FROM InitScia() FUNCTION (TEXAS FILES) ////////////////////////////////////////
	EALLOW;

	/* Enable internal pull-up for the selected pins */
	// Pull-ups can be enabled or disabled disabled by the user.
	// This will enable the pull-ups for the specified pins.

	//GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
	//GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	 // Enable pull-up for GPIO29 (SCITXDA)
    // why do we need internal pull-up


	//485 SCI for the company dsp GPIO18 and GPIO19

	//GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;    // Enable pull-up for GPIO18 (SCITXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	 // Enable pull-up for GPIO19 (SCIRXDB)


	/* Set qualification for selected pins to asynch only */
	// Inputs are synchronized to SYSCLKOUT by default.
	// This will select asynch (no qualification) for the selected pins.
	//GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)

	//GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)

	/* Configure SCI-A pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be SCI functional pins.

	//GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
	//GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation

	//GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation
    //GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
    // changed according to system and interrupt.pdf(GPIO　MUX)
	//GPIO 20 to control the 485 chip receive/output enable
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;   // Configure GPIO20 as digital I/O
	GpioCtrlRegs.GPADIR.bit.GPIO20  = 1;   // Configure GPIO20 as output to control the 485 chip

	// For SCIC
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;    // Enable pull-up for GPIO63 (SCITXDC)

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)


	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

   // For SCIC
	EDIS;
	// END: GOT FROM InitScia() FUNCTION (TEXAS FILES) ////////////////////////////////////////
	// Number of bytes

	switch(self->bitsNumber) {
		case 8:
			ScicRegs.SCICCR.bit.SCICHAR = 0x7;
			break;
		case 7:
			ScicRegs.SCICCR.bit.SCICHAR = 0x6;
			break;
		default:
			ScicRegs.SCICCR.bit.SCICHAR = 0x7;
	}

	// Parity settings
	switch(self->parityType){
		case SERIAL_PARITY_EVEN:
			ScicRegs.SCICCR.bit.PARITYENA = 1;
			ScicRegs.SCICCR.bit.PARITY = 1;
			break;
		case SERIAL_PARITY_ODD:
			ScicRegs.SCICCR.bit.PARITYENA = 1;
			ScicRegs.SCICCR.bit.PARITY = 0;
			break;
		case SERIAL_PARITY_NONE:
			ScicRegs.SCICCR.bit.PARITYENA = 0;
			break;
		default:
			ScicRegs.SCICCR.bit.PARITYENA = 0;
	}

	// Baud rate settings - Automatic depending on self->baudrate
//	baudrate = (Uint32) (SysCtrlRegs.LOSPCP.bit.LSPCLK / (self->baudrate*8) - 1);
	//TODO
	baudreg = (Uint32) (LOW_SPEED_CLOCK / (self->baudrate*8) - 1);

	// Configure the High and Low baud rate registers
    //ScicRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
    //ScicRegs.SCILBAUD    =0x0044;
	ScicRegs.SCIHBAUD = (baudreg & 0xFF00) >> 8;
	ScicRegs.SCILBAUD = (baudreg & 0x00FF);

	// Enables TX and RX Interrupts
	ScicRegs.SCICTL2.bit.TXINTENA = 0;
	ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
	ScicRegs.SCICTL2.bit.RXBKINTENA = 0;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 0;

	// FIFO TX configurations
	ScicRegs.SCIFFTX.bit.TXFFIL = 1;	// Interrupt level
	ScicRegs.SCIFFTX.bit.SCIFFENA = 1;	// Enables FIFO
	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;	// Clear interrupt flag

	// FIFO: RX configurations
	ScicRegs.SCIFFRX.bit.RXFFIL = 1;	// Interrupt level
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// Clear interrupt flag
	ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;	// Clear overflow flag

	// FIFO: Control configurations
	ScicRegs.SCIFFCT.all=0x00;

	// Enable RX and TX and reset the serial
	ScicRegs.SCICTL1.bit.RXENA	 = 1;
	ScicRegs.SCICTL1.bit.TXENA	 = 1;
	ScicRegs.SCICTL1.bit.SWRESET = 1;

	// FIFO: Reset
	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
	ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	ScicRegs.SCIFFTX.bit.SCIRST = 0;
	ScicRegs.SCIFFTX.bit.SCIRST = 1;

	SERIAL_DEBUG();
}

// Transmit variable data based on passed size
void serial_transmitData(Uint16 *data, Uint16 size){          //TODO  how this works?
	static unsigned short i = 0;                               //
	SERIAL_DEBUG();                                            //

	for (i = 0; i < size; i++){
	    // make sure last data is sent before another data is done
	    //while (ScicRegs.SCICTL2.bit.TXEMPTY != true) ;
		ScicRegs.SCITXBUF= data[i];

		//if(i%4 == 0){//TODO why 4?

		//wait until TX buffer is empty before sending another byte
		//use this to make sure no data will be written out without sending

	    //while (ScicRegs.SCICTL2.bit.TXEMPTY != true) ;
		//}
	}

	// If you want to wait until the TX buffer is empty, uncomment line below
	//while (ScicRegs.SCICTL2.bit.TXEMPTY != true) ;
}

// Read data from buffer (byte per byte)
Uint16 serial_getRxBufferedWord(){
	SERIAL_DEBUG();

	// TODO: check if it is needed
	while (ScicRegs.SCIRXST.bit.RXRDY) ;//buffer cleared, ready to receive

	return ScicRegs.SCIRXBUF.all;
}

bool serial_getRxError(){
	SERIAL_DEBUG();

	return ScicRegs.SCIRXST.bit.RXERROR;
}

// Construct the Serial Module
Serial construct_Serial(){
	Serial serial;
//  change this to change SCI setting
    serial.baudrate=9600;
    serial.bitsNumber=8;
    serial.parityType = SERIAL_PARITY_NONE;
//

	serial.clear = serial_clear;
	serial.rxBufferStatus = serial_rxBufferStatus;
	serial.setSerialRxEnabled = serial_setSerialRxEnabled;
	serial.setSerialTxEnabled = serial_setSerialTxEnabled;
	serial.init = serial_init;
	serial.transmitData = serial_transmitData;
	serial.getRxBufferedWord = serial_getRxBufferedWord;
	serial.getRxError = serial_getRxError;

	serial.fifoWaitBuffer = 0;

	SERIAL_DEBUG();

	return serial;
}
