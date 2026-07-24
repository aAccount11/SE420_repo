//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28377sSerial.h"
#include "F28377sADC.h"
#include "F28377sDAC.h"
#include "F28377sEPWM3A8A.h"
#include "F28377sEQep.h"
#include "lcd.h"
#include "SpiRAM.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

extern float SIMU_value1_32bit; // any float value
extern float SIMU_value2_32bit; // any float value
extern float SIMU_value3_16bit; // smaller float between -32 and 32 because 16bit
extern float SIMU_value4_16bit; // smaller float between -32 and 32 because 16bits

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint16_t UARTPrint = 0;
uint32_t time = 0;
extern uint16_t SpiRAM_total_data;

void main(void)
{

    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xS_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xS_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xS_DefaultIsr.c.
    // This function is found in F2837xS_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
	PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
	PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
	PieVectTable.DMA_CH4_INT = &DMA_ISR;
    PieVectTable.SPIA_RX_INT = &SPIA_isr;//spia

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xS_CpuTimers.c
    InitCpuTimers();
	
    // Configure CPU-Timer 0, 1, and 2 to interrupt every 1ms, 20ms, 40ms respectively:
    // 200MHz CPU Freq, 			Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 20000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	// Red LED, GPIO12
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);						
	GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);		
	GpioDataRegs.GPASET.bit.GPIO12 = 1;

	// Blue LED, GPIO13
    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);				
	GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPASET.bit.GPIO13 = 1;

	//LEDS
	//LED1  GPIO64
	GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPCSET.bit.GPIO64 = 1;
	//LED2  GPIO91
	GPIO_SetupPinMux(91, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPCSET.bit.GPIO91 = 1;
	//LED3  GPIO92
	GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPCSET.bit.GPIO92 = 1;
	//LED4  GPIO99
	GPIO_SetupPinMux(99, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(99, GPIO_OUTPUT, GPIO_PUSHPULL);
	GpioDataRegs.GPDSET.bit.GPIO99 = 1;
	
	//SWITCHES
	//SW1	GPIO41
	GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(41, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	//SW2	GPIO66
	GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(66, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	//SW3	GPIO73
	GPIO_SetupPinMux(73, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(73, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	//SW4	GPIO78
	GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(78, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
	
	InitSpiaGpio();
	InitEPwm3Gpio();
	InitEPwm8Gpio();

	//Configure the ADCs and power them up
	ConfigureADC();

	//Setup the ADCs for software conversions
	SetupADCSoftware();

	initEPwm3A();
    initEPwm7A();
	initEPwm8();
	setEPWM3A(0);
	setEPWM8A(0);
	initDACs();
	init_EQEPs();
	InitDma();         // Set up DMA for SPI configuration
    setupSpia();       // Initialize the SPI only
	//
    // Ensure DMA is connected to Peripheral Frame 2 bridge (EALLOW protected)
    //
    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;
    EDIS;


	init_serialSCIA(&SerialA,115200);  // Matlab COM
	
	init_serialSCIB(&SerialB,115200);  //Simulink COM
	
	init_serialSCIC(&SerialC,19200); // For Text LCD


	// ************ your Init code here  *******************


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT7;  //DMA
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEIER6.bit.INTx1 = 1; //spia
    // Enable dma ch5 in the PIE: Group 7 interrupt 4
    PieCtrlRegs.PIEIER7.bit.INTx4 = 1;   // Enable PIE Group 7, INT 4 (DMA CH4)
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;



    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

	init_lcd(90);
	
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            UART_printfLine(1,"Num T2:%ld",CpuTimer2.InterruptCount);
            UARTPrint = 0;
        }
    }
}



// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%250) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1;
    }
	

    // Example for using the saveData function to save date to the SPIRAM for data collection to Matlab
    //saveData(time*0.001,1.5*sin(2*PI*.5*time*.001),3*sin(2*PI*.5*time*.001),4*sin(2*PI*.5*time*.001),5*sin(2*PI*.5*time*.001));
    //time = time + 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}





