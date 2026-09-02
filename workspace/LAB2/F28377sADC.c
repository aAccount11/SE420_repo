/* F28377sADC.c
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
#include <F28377sADC.h>
#define ADC_RESOLUTION_12BIT 0



// this function has already been called for you in the main() function.  
// It sets up ADC A and ADC B in 12bit conversion mode
//Write ADC configurations and power up the ADC for both ADC A and ADC B
// ADCB has both our ADC1 and ADC2 channels.
void ConfigureADC(void)
{
	EALLOW;

	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);

	EDIS;
}

// this function has already been called for you in the main() function.  
// It finished the setup for ADC B.  ADC1 and ADC2
void SetupADCSoftware(void)
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 99; //500ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//Select the channels to convert and end of conversion flag
	//ADCA
	EALLOW;
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin A2
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

//	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//ADCB
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin B1
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC2 will convert pin B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
//    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC3 will convert pin B3
//    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

#warn  Make sure to start ADCB0 ADCB1 and ADCB2
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2; //end of SOC2 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;
}


