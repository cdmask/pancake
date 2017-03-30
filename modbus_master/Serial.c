#include "DSP2833x_Device.h"     // DSP2833x Header file Include File
#include "Serial.h"
//#include "Log.h"
#include "ModbusSettings.h"


/////////////////////////////////////////////
// SciaRegs changed into (*(self->SciRegs))
/////////////////////////////////////////////
// Clear flags of overflow
void serial_clear(Serial *self){
	static unsigned short i, destroyFifo;

	//SERIAL_DEBUG();

	// Reset Serial in case of error
	if((*(self->SciRegs)).SCIRXST.bit.RXERROR == true){
		(*(self->SciRegs)).SCICTL1.bit.SWRESET=0;
	}

	// Clears FIFO buffer (if there is any data)
	for (i = (*(self->SciRegs)).SCIFFRX.bit.RXFFST; i > 0; i--)
		destroyFifo = (*(self->SciRegs)).SCIRXBUF.all;

	// Reset FIFO
	(*(self->SciRegs)).SCIFFRX.bit.RXFIFORESET=1;
	(*(self->SciRegs)).SCIFFTX.bit.TXFIFOXRESET=1;

	(*(self->SciRegs)).SCICTL1.bit.SWRESET=1;

}

// Get how much data is at the RX FIFO Buffer
Uint16 serial_rxBufferStatus(Serial *self){
	return (*(self->SciRegs)).SCIFFRX.bit.RXFFST;
}

// Enable or disable RX (receiver)
void serial_setSerialRxEnabled(Serial *self,bool status){
	//SERIAL_DEBUG();
	(*(self->SciRegs)).SCICTL1.bit.RXENA = status;
}

// Enable or disable TX (trasmiter)
void serial_setSerialTxEnabled(Serial *self,bool status){
	//SERIAL_DEBUG();
	(*(self->SciRegs)).SCICTL1.bit.TXENA = status;

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
    // why do we need internal pull-up


	//485 SCI for the company dsp GPIO18 and GPIO19

    switch(self->Port) {
        case SCI_A:

            break;
        case SCI_B:
            GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;    // Enable pull-up for GPIO18 (SCITXDB)
            GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;  // Enable pull-up for GPIO19 (SCIRXDB)
            /* Set qualification for selected pins to asynch only */
            // Inputs are synchronized to SYSCLKOUT by default.
            // This will select asynch (no qualification) for the selected pins.
            //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
            GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
            /* Configure SCI-A pins using GPIO regs*/
            // This specifies which of the possible GPIO pins will be SCI functional pins.
            GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation
            GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
            // changed according to system and interrupt.pdf(GPIO　MUX)
            //GPIO 20 to control the 485 chip receive/output enable
            GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;   // Configure GPIO20 as digital I/O
            GpioCtrlRegs.GPADIR.bit.GPIO20  = 1;   // Configure GPIO20 as output to control the 485 chip
            break;
        case SCI_C:
            GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
            GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;    // Enable pull-up for GPIO63 (SCITXDC)

            GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

            GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
            GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation

            break;
    }

	// END: GOT FROM InitScia() FUNCTION (TEXAS FILES) ////////////////////////////////////////
	// Number of bytes

	switch(self->bitsNumber) {
		case 8:
			(*(self->SciRegs)).SCICCR.bit.SCICHAR = 0x7;
			break;
		case 7:
		    (*(self->SciRegs)).SCICCR.bit.SCICHAR = 0x6;
			break;
		default:
		    (*(self->SciRegs)).SCICCR.bit.SCICHAR = 0x7;
	}

	// Parity settings
	switch(self->parityType){
		case SERIAL_PARITY_EVEN:
			ScibRegs.SCICCR.bit.PARITYENA = 1;
			(*(self->SciRegs)).SCICCR.bit.PARITY = 1;
			break;
		case SERIAL_PARITY_ODD:
			(*(self->SciRegs)).SCICCR.bit.PARITYENA = 1;
			(*(self->SciRegs)).SCICCR.bit.PARITY = 0;
			break;
		case SERIAL_PARITY_NONE:
			(*(self->SciRegs)).SCICCR.bit.PARITYENA = 0;
			break;
		default:
			(*(self->SciRegs)).SCICCR.bit.PARITYENA = 0;
	}

	// Baud rate settings - Automatic depending on self->baudrate
//	baudrate = (Uint32) (SysCtrlRegs.LOSPCP.bit.LSPCLK / (self->baudrate*8) - 1);
	//TODO
	baudreg = (Uint32) (LOW_SPEED_CLOCK / (self->baudrate*8) - 1);

	// Configure the High and Low baud rate registers
    //(*(self->SciRegs)).SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
    //(*(self->SciRegs)).SCILBAUD    =0x0044;
	(*(self->SciRegs)).SCIHBAUD = (baudreg & 0xFF00) >> 8;
	(*(self->SciRegs)).SCILBAUD = (baudreg & 0x00FF);

	// Enables TX and RX Interrupts
	(*(self->SciRegs)).SCICTL2.bit.TXINTENA   = 0;
	(*(self->SciRegs)).SCIFFTX.bit.TXFFIENA   = 0;
	(*(self->SciRegs)).SCICTL2.bit.RXBKINTENA = 1;
	(*(self->SciRegs)).SCIFFRX.bit.RXFFIENA   = 0;

	// FIFO TX configurations
	(*(self->SciRegs)).SCIFFTX.bit.TXFFIL = 1;	// Interrupt level
	(*(self->SciRegs)).SCIFFTX.bit.SCIFFENA = 0;	// Enables FIFO
	(*(self->SciRegs)).SCIFFTX.bit.TXFFINTCLR = 1;	// Clear interrupt flag

	// FIFO: RX configurations
	(*(self->SciRegs)).SCIFFRX.bit.RXFFIL = 1;	// Interrupt level
	(*(self->SciRegs)).SCIFFRX.bit.RXFFINTCLR = 1;	// Clear interrupt flag
	(*(self->SciRegs)).SCIFFRX.bit.RXFFOVRCLR = 1;	// Clear overflow flag

	// FIFO: Control configurations
	(*(self->SciRegs)).SCIFFCT.all=0x00;

	// Enable RX and TX and reset the serial
	(*(self->SciRegs)).SCICTL1.bit.RXENA	 = 1;
	(*(self->SciRegs)).SCICTL1.bit.TXENA	 = 1;
	(*(self->SciRegs)).SCICTL1.bit.SWRESET = 1;

	// FIFO: Reset
	(*(self->SciRegs)).SCIFFRX.bit.RXFIFORESET = 1;
	(*(self->SciRegs)).SCIFFTX.bit.TXFIFOXRESET = 1;
	(*(self->SciRegs)).SCIFFTX.bit.SCIRST = 0;
	(*(self->SciRegs)).SCIFFTX.bit.SCIRST = 1;
EDIS;
	//SERIAL_DEBUG();
}

// Transmit variable data based on passed size
void serial_transmitData(Serial *self,Uint16 *data, Uint16 size){          //TODO  how this works?
	static unsigned short i = 0;                               //
	//SERIAL_DEBUG();                                            //

	for (i = 0; i < size; i++){
	    // make sure last data is sent before another data is done
	    while (ScibRegs.SCICTL2.bit.TXEMPTY != true) ;
		(*(self->SciRegs)).SCITXBUF= data[i];

		//if(i%4 == 0){//TODO why 4? m/

		//wait until TX buffer is empty before sending another byte
		//use this to make sure no data will be written out without sending

	    //while (ScibRegs.SCICTL2.bit.TXEMPTY != true) ;
		//}
	}

	// If you want to wait until the TX buffer is empty, uncomment line below
	//while (ScibRegs.SCICTL2.bit.TXEMPTY != true) ;
}

// Read data from buffer (byte per byte)
Uint16 serial_getRxBufferedWord(Serial *self){
	//SERIAL_DEBUG();

	// TODO: check if it is needed
	while (ScibRegs.SCIRXST.bit.RXRDY) ;//buffer cleared, ready to receive

	return (*(self->SciRegs)).SCIRXBUF.all;
}

bool serial_getRxError(Serial *self){
	//SERIAL_DEBUG();

	return (*(self->SciRegs)).SCIRXST.bit.RXERROR;
}

// Construct the Serial Module
Serial construct_Serial(SCIPort port,Uint32 baudRate,Uint16 bitsNumber){
    Serial serial;
    serial.Port = port;
    switch(port) {
        case SCI_A:
            serial.SciRegs = &SciaRegs;
            break;
        case SCI_B:
            serial.SciRegs = &ScibRegs;
            break;
        case SCI_C:
            serial.SciRegs = &ScicRegs;
            break;
        default:
            serial.SciRegs = &ScibRegs;
    }


//  change this to change SCI setting
    serial.baudrate   =baudRate;
    serial.bitsNumber =bitsNumber;
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

	//SERIAL_DEBUG();

	return serial;
}
