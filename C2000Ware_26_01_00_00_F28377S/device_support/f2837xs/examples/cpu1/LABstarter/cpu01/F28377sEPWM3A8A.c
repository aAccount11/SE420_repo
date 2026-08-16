/* 28335_EPWM3A.C:  F28377S DSP EPWM peripherals interface.
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <F28377sEPWM3A8A.h>

int16_t DAN28027Garbage = 0;

// this function has already been called for you in the main() function.  
// It sets up PWM3A with a 20KHz carrier frequency PWM signal.  
void initEPwm3A(void)
{
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;		//set epwm3 to upcount mode
	EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
	EPwm3Regs.TBPRD = 2500; //set epwm3 counter  20KHz
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
	EPwm3Regs.TBCTR = 0x0000;                  // Clear counter

	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;		//clear when counter = compareA
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;			//set when timer is 0
}
// This function sets PWM3A duty cycle given the float value between -10 and 10.  
// Where 
// -10 equates to 0% duty cycle
//   0 equates to 50% duty cycle
//  10 equates to 100% duty cycle
//  so for example if you pass 5.0 to this function EPWM3A will be set to 75% duty cycle
//  Example code
//  float myu = 0;
//  float Kpgain = 4.5;
//  float error = 0;
//
//	myu = Kpgain*error;
//
//	setEPWM3A(myu);

void initEPwm7A(void)
{
	EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;		//set epwm7 to upcount mode
	EPwm7Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
	EPwm7Regs.TBPRD = 2500; //set epwm7 counter  20KHz
	EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
	EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
	EPwm7Regs.TBCTR = 0x0000;                  // Clear counter

	EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;		//clear when counter = compareA
	EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;			//set when timer is 0
}

void initEPwm8(void)
{
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      //set epwm8 to upcount mode
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm8Regs.TBPRD = 2500; //set epwm8 counter  20KHz
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                  // Clear counter

    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0

    EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;
}

void setEPWM3A(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm3Regs.CMPA.bit.CMPA = (int)pwmVal;

}

void setEPWM7A(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm7Regs.CMPA.bit.CMPA = (int)pwmVal;

}



void setEPWM8A(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm8Regs.CMPA.bit.CMPA = (int)pwmVal;

}

void setEPWM8B(float u) {
    float pwmCountMax = 2500.0;
    float pwmVal = 0;

    if (u >  10) u =  10;
    if (u < -10) u = -10;

    pwmVal = u * (pwmCountMax / 20.0) + pwmCountMax / 2.0;

    EPwm8Regs.CMPB.bit.CMPB = (int)pwmVal;
}

void setF28027EPWM(float controleffort){
    int16_t EPwm1A_F28027 = 1500;
    int16_t EPwm1B_F28027 = 1500;  // Not used right now.  Furuta only needs one PWM
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort > 10) {
        controleffort = 10;
    }
    float value = (controleffort+10)*3000.0/20.0;
    EPwm1A_F28027 = (int16_t)value; 

    SpicRegs.SPIFFRX.bit.RXFFIL = 3;
    GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;
    SpicRegs.SPITXBUF = 0x00DA;
    SpicRegs.SPITXBUF = EPwm1A_F28027;
    SpicRegs.SPITXBUF = EPwm1B_F28027;

}

void setupSpic(void) //Call this function in main() somewhere after the DINT; line of code.
{
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0); // Set as GPIO65 and used as DAN28027 SS
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO65 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO65 = 1; //Initially Set GPIO65/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(69, GPIO_MUX_CPU1, 15); //Set GPIO69 pin to SPISIMOC
    //Not using SPISOMIC  Just need to set PWM values on F28027  
    GPIO_SetupPinMux(71, GPIO_MUX_CPU1, 15); //Set GPIO71 pin to SPICLKC

    EALLOW;
    GpioCtrlRegs.GPCPUD.bit.GPIO69 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO71 = 0;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO69 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO71 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpicRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpicRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpicRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpicRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpicRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpicRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpicRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt

    SpicRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpicRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpicRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpicRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpicRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpicRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpicRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpicRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpicRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL

    SpicRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip

    SpicRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset

    SpicRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpicRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    //SpicRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test

    SpicRegs.SPIFFRX.bit.RXFFIL =16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below


    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpicRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpicRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

__interrupt void SPIC_isr(void) {

    GpioDataRegs.GPCSET.bit.GPIO65 = 1; // Pull CS high for DAN28027

    DAN28027Garbage = SpicRegs.SPIRXBUF;
    DAN28027Garbage = SpicRegs.SPIRXBUF;
    DAN28027Garbage = SpicRegs.SPIRXBUF;

    SpicRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpicRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}
