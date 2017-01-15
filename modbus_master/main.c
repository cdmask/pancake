#include "lookup_table.h"
#include "ModbusMaster.h"
#include "DSP2833x_GlobalPrototypes.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "calc_rod_length.h"

__interrupt void cpu_timer1_isr(void);


// 6dof data
 float32 test_sine_ref[500]={0};
 float32 real_sine_ref=0;
 float32 home_trans[3]={0,0,291.8};
 float32 home_orient[3]={0,0,0};
 float32 trans[3]={0,0,0},orient[3]={0,0,0};
 float32 rod_attach_P[18]  ={  140.9539, 26.0472,  -26.0472,   -140.9539, -114.9067,  114.9067,
                               51.3030, -147.7212, -147.7212,   51.3030,   96.4181,   96.4181,
                                   0,         0,         0,         0,         0,         0};

 float32 servo_attach_B[18]={ 234.9232, 191.5111, -191.5111, -234.9232, -43.4120,  43.4120,
                             -85.5050, -160.6969, -160.6969, -85.5050,   246.2019, 246.2019,
                                    0,         0,         0,        0,          0,        0};
 float32 length[6];            //this is the data to send to slave
 Uint16 scaled_length[6];
 Uint16 string[16]={0};


int i=0;
int interrupt_index=0;
int if_ref_updated=0;
//ModbusMaster mb;
Serial SCIB;
Uint16 *dataPtr;

void main()
{

	InitSysCtrl();
	DINT;


	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	   InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	   IER = 0x0000;
	   IFR = 0x0000;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
	// This function is found in DSP2833x_PieVect.c.
	   InitPieVectTable();

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	   EALLOW;  // This is needed to write to EALLOW protected registers
	   //PieVectTable.TINT0 = &cpu_timer0_isr;
	   PieVectTable.XINT13 = &cpu_timer1_isr;
	   //PieVectTable.TINT2 = &cpu_timer2_isr;
	   EDIS;    // This is needed to disable write to EALLOW protected registers

	// Step 4. Initialize the Device Peripheral. This function can be
	//         found in DSP2833x_CpuTimers.c
	   InitCpuTimers();   // For this example, only initialize the Cpu Timers

	#if (CPU_FRQ_150MHZ)
	// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
	// 150MHz CPU Freq, 1 second Period (in uSeconds)

	   //ConfigCpuTimer(&CpuTimer0, 150, 1000000);

	    // change this to work with SCI data transfer
	    // make this interrupt time a bit higher than the time it takes to transfer the data once
	   ConfigCpuTimer(&CpuTimer1, 150, 1000);


	   //ConfigCpuTimer(&CpuTimer2, 150, 1000000);
	#endif

	#if (CPU_FRQ_100MHZ)
	// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
	// 100MHz CPU Freq, 1 second Period (in uSeconds)


	   //ConfigCpuTimer(&CpuTimer1, 100, 2000);// 500 points, 2000us between two points, period 1s, f 1Hz


	   // change this to work with SCI data transfer
	   // make this interrupt time a bit higher than the time it takes to transfer the data once
	   ConfigCpuTimer(&CpuTimer1, 100, 10000);  // every 10ms one interrupt


	#endif
	// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
	// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
	// below settings must also be updated.

	   //Enable timer interrupt
	   //CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

	   // Enable timer interrupt and start timer!
	   CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0


	   //CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

	// Step 5. User specific code, enable interrupts:

	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
	// which is connected to CPU-Timer 1, and CPU int14, which is connected
	// to CPU-Timer 2:
	   //IER |= M_INT1;

	   IER |= M_INT13;// this is timer 1 interrupt

	   //IER |= M_INT14;

	// Enable TINT0 in the PIE: Group 1 interrupt 7
	   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	// Enable global Interrupts and higher priority real-time debug events:

	   EINT;   // Enable Global interrupt INTM
	   ERTM;   // Enable Global real-time interrupt DBGM

//TODO when did the timer start?
    // setup
	SCIB = construct_Serial();
	SCIB.init(&SCIB);

	GpioDataRegs.GPASET.bit.GPIO20 = 1; // set 485 chip enable

	home_orient[2]=0;
	for(i=0;i<12;i++)
	    string[i]=0;
	while(1)
	{

	}

}

__interrupt void cpu_timer1_isr(void)
{
   CpuTimer1.InterruptCount++;
   interrupt_index++;

   //change this to change the reference sine wave
   test_sine_ref[interrupt_index] = (float)(sine_table[interrupt_index])/4096;// use float without () throws error, expected an expression
   real_sine_ref    = (float)(sine_table[interrupt_index])/4096;
   if(interrupt_index==499)
       {
       interrupt_index=0;
       }
             // reference trans
              trans[0]=0;
              trans[1]=0;
              trans[2]=0;
             //
             // reference orient
              orient[0]=0;
              orient[1]=0;
              orient[2]=0;
             //

              trans[2]+=home_trans[2];
              //calculate length
              calc_rod_length(trans,orient);
              int index=0;
              //get the to be transmitted string
              dataPtr = scaled_length;
              for(i=0; i < 6; i++){
                              string[index++] = (*(dataPtr + i) & 0xFF00) >> 8;
                              string[index++] = (*(dataPtr + i) & 0x00FF);
              }

              // For test

              //string[12]=0x5a;
              //string[13]=0x5a;
              //string[14]=0x5a;
              //string[15]=0x5a;

             SCIB.transmitData(string,12);
   EDIS;
}
