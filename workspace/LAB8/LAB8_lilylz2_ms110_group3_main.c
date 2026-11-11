// #############################################################################
//  FILE:   LABstarter_main.c
//
//  TITLE:  Lab Starter
// #############################################################################

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

#define PI 3.1415926535897932384626433832795
#define TWOPI 6.283185307179586476925286766559
#define HALFPI 1.5707963267948966192313216916398

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCB_ISR(void);

extern float SIMU_value1_32bit; // any float value
extern float SIMU_value2_32bit; // any float value
extern float SIMU_value3_16bit; // smaller float between -32 and 32 because 16bit
extern float SIMU_value4_16bit; // smaller float between -32 and 32 because 16bits
void  SetupADCSOversampling(void);

// global variables
float adcb0result = 0.0;
float adcb1result = 0.0;
float adcb2result = 0.0;
float adcb0volt = 0.0;
float adcb1volt = 0.0;
float adcb2volt = 0.0;
float sumAdcb0volts = 0;
float sumAdcb1volts = 0;
float sumAdcb2volts = 0;
int16_t sampleCount = 0; // counts number of times ADC channels have been sampled
float avgAdcb0 = 0.0f;
float avgAdcb1 = 0.0f;
float avgAdcb2 = 0.0f;
int16_t SendBetterSerialPlotter = 0;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint16_t UARTPrint = 0;
uint32_t time = 0;
extern uint16_t SpiRAM_total_data;
uint32_t count100ms = 0;

// transfer function variables
float kp = 5.45875;
float ki = 82.5;
float kd = 0.1875;
float theta = 0.0f; // position
float theta_old = 0.0f;
float err = 0.0f;
float err_dot = 0.0f;
float err_dot_old = 0.0f;
float theta_dot = 0.0f; // velocity
float theta_dot_old = 0.0f;
float u = 0.0f;
float pp_control = 0.0f;;
float desired_theta = 0.0f;
float Ik = 0.0f;
float Ik_old = 0.0f;
float err_old = 0.0f;
float uk = 0.0f;
bool toggle = false;

float motorPos = 0.0f;
float referencePos = 0.0f;

float refK = 1.0; // Or use your reference that is stepping back and forth between 1.0 and 2.0
float refK_1 = 0.0;
float ref1filtK = 0.0;
float ref1filtK_1 = 0.0;
float ref2filtK = 0.0;
float ref2filtK_1 = 0.0;
float ref3filtK = 0.0;
float ref3filtK_1 = 0.0;
float ref4filtK = 0.0;
float ref4filtK_1 = 0.0;
float ref5filtK = 0.0;
float ref5filtK_1 = 0.0;
float ref6filtK = 0.0;
float ref6filtK_1 = 0.0;
float filt_a = 15.0;
float SamplePeriod = 0.001;
float filtCoeff1 = 0.0;
float filtCoeff2 = 0.0;


void SetupADCSOversampling(void) {
    uint16_t acqps;

    // determine minimum acquisition window (in SYSCLKS) based on resolution
    if (ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION) {
        acqps = 99; // 500ns
    } else {        // resolution is 16-bit
        acqps = 63; // 320ns
    }

    // Select the channels to convert and end of conversion flag
    // ADCA
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;     // SOC0 will convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; // sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;     // SOC1 will convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; // sample window is acqps + 1 SYSCLK cycles

    //	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
    //	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    //	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    // ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;     // SOC0 will convert pin B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; // sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;     // SOC1 will convert pin B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; // sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;     // SOC2 will convert pin B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; // sample window is acqps + 1 SYSCLK cycles
    //    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC3 will convert pin B3
    //    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles

// #warn Make sure to start ADCB0 ADCB1 and ADCB2
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2; // end of SOC2 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   // enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // make sure INT1 flag is cleared
    
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will cause SOC0
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will cause SOC1
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will cause SOC2
    
    EDIS;

}

void main(void) {

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
    EALLOW; // This is needed to write to EALLOW protected registers
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
    PieVectTable.SPIA_RX_INT = &SPIA_isr; // spia
    PieVectTable.SPIC_RX_INT = &SPIC_isr; // spic
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    EDIS; // This is needed to disable write to EALLOW protected registers

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

    // LEDS
    // LED1  GPIO64
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(64, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO64 = 1;
    // LED2  GPIO91
    GPIO_SetupPinMux(91, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO91 = 1;
    // LED3  GPIO92
    GPIO_SetupPinMux(92, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(92, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO92 = 1;
    // LED4  GPIO99
    GPIO_SetupPinMux(99, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(99, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO99 = 1;

    // SWITCHES
    // SW1	GPIO41
    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(41, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
    // SW2	GPIO66
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
    // SW3	GPIO73
    GPIO_SetupPinMux(73, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(73, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);
    // SW4	GPIO78
    GPIO_SetupPinMux(78, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(78, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);

    InitEPwm3Gpio();
    InitEPwm8Gpio();

    // Configure the ADCs and power them up
    ConfigureADC();

    // Setup the ADCs for software conversions
    // SetupADCSoftware();
    SetupADCSOversampling();
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0;            // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_FREEZE;   // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2;           // Select Event when equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;            // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0;            // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 3125U;                   // (1/16)ms sample. U for unsigned
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;            // enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // unfreeze, and enter up count mode
    EDIS;

    initEPwm3A();
    initEPwm7A();
    initEPwm8();
    setEPWM3A(0);
    setEPWM8A(0);
    initDACs();
    init_EQEPs();
    InitDma();   // Set up DMA for SPI configuration
    setupSpia(); // Initialize the SPI only
    setupSpic();
    //
    // Ensure DMA is connected to Peripheral Frame 2 bridge (EALLOW protected)
    //
    EALLOW;
    CpuSysRegs.SECMSEL.bit.PF2SEL = 1;

    EDIS;

    init_serialSCIA(&SerialA, 115200); // Matlab COM

    init_serialSCIB(&SerialB, 115200); // BetterSerialPlotter or Simulink COM

    init_serialSCIC(&SerialC, 19200); // For Text LCD

    // ************ your Init code here  *******************

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT6; // SPIA and SPIC
    IER |= M_INT7; // DMA
    IER |= M_INT8; // SCIC SCID
    IER |= M_INT9; // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1; // Enable the PIE block

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1; // spia rx
    PieCtrlRegs.PIEIER6.bit.INTx9 = 1; // spic rx
    // Enable dma ch5 in the PIE: Group 7 interrupt 4
    PieCtrlRegs.PIEIER7.bit.INTx4 = 1; // Enable PIE Group 7, INT 4 (DMA CH4)
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT; // Enable Global interrupt INTM
    ERTM; // Enable Global realtime interrupt DBGM

    init_lcd(90);

    // IDLE loop. Just sit and loop forever (optional):
    while (1) {
        if (UARTPrint == 1) {
            UART_printfLine(1, "Volt0:%.3f", avgAdcb2);
            UARTPrint = 0;
        }
        if (SendBetterSerialPlotter == 1 ) {
            serialB_printf(&SerialB,"%.4f %.4f %.4f\n\r",avgAdcb2, ref6filtK, uk);
            SendBetterSerialPlotter = 0;
        }
        // if (SendBetterSerialPlotter == 1 ) {
        //     serialB_printf(&SerialB,"%.4f %.4f %.4f %.4f\n\r",test1,test2,test3,test4);
        //     SendBetterSerialPlotter = 0;
        // }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP"); // Wait one cycle
    EINT;              // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......
    if(numSWIcalls % 5 == 0) {
        SendBetterSerialPlotter = 1;
    }
    
    if(  numSWIcalls % 2000 == 0 ){
        toggle = ! toggle;
        if( toggle == true ){
            desired_theta = 1.f;
        }
        else {
            desired_theta = 2.f;
        }
    }
    

    // filter on desired_theta
    refK = desired_theta;
    filtCoeff1 = (filt_a*SamplePeriod)/(2.0f+filt_a*SamplePeriod);
    filtCoeff2 = (2.0f-filt_a*SamplePeriod)/(2.0f+filt_a*SamplePeriod);
    ref1filtK = filtCoeff1*(refK+refK_1) + filtCoeff2*ref1filtK_1;
    ref2filtK = filtCoeff1*(ref1filtK+ref1filtK_1) + filtCoeff2*ref2filtK_1;
    ref3filtK = filtCoeff1*(ref2filtK+ref2filtK_1) + filtCoeff2*ref3filtK_1;
    ref4filtK = filtCoeff1*(ref3filtK+ref3filtK_1) + filtCoeff2*ref4filtK_1;
    ref5filtK = filtCoeff1*(ref4filtK+ref4filtK_1) + filtCoeff2*ref5filtK_1;
    ref6filtK = filtCoeff1*(ref5filtK+ref5filtK_1) + filtCoeff2*ref6filtK_1;
    ref6filtK_1 = ref6filtK;
    ref5filtK_1 = ref5filtK;
    ref4filtK_1 = ref4filtK;
    ref3filtK_1 = ref3filtK;
    ref2filtK_1 = ref2filtK;
    ref1filtK_1 = ref1filtK;
    refK_1 = refK;

    // controller
    theta = avgAdcb2;
    err = ref6filtK - theta;
    theta_dot = 400.0f  * (theta - theta_old) + .6f * theta_dot_old;
    Ik = (0.001 * (err + err_old) + 2.0*Ik_old) / 2.0 ;
    u = kp * err + ki * Ik - kd * theta_dot;

    // managing integral windup
    if( fabs( u ) > 10 ){
           Ik = Ik_old * .99;
    }

    // damper
    if( u > 10.0f ){
        u = 10.0;
    }
    else if(u < -10.0f){
        u = -10.0f;
    }


    
    Ik_old = Ik;
    err_old = err;

    uk = u;

    theta_dot_old = theta_dot;
    theta_old = theta;

    setEPWM8A(u);

    setDAC1(avgAdcb0);
    if (count100ms % 100 == 0) {
        UARTPrint = 1;
    }
    count100ms += 1;

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void) {
    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    // AdcbRegs.ADCSOCPRICTL.bit.RRPOINTER = 0x10; //Round-Robin pointer reset, soc0 next
    // AdcbRegs.ADCSOCFRC1.all |= 7; //start conversion of ADCINB0, ADCINB1 & ADCINB2

    if ((numTimer0calls % 250) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1;
    }



    // Example for using the saveData function to save date to the SPIRAM for data collection to Matlab
    // saveData(time*0.001,1.5*sin(2*PI*.5*time*.001),3*sin(2*PI*.5*time*.001),4*sin(2*PI*.5*time*.001),5*sin(2*PI*.5*time*.001));
    // time = time + 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void) { numTimer1calls++; }

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void) {
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;

    numTimer2calls++;

    if ((numTimer2calls % 50) == 0) {
        UARTPrint = 1;
    }
}

// adcb1 pie interrupt
__interrupt void ADCB_ISR(void) {
    adcb0result = AdcbResultRegs.ADCRESULT0;
    adcb1result = AdcbResultRegs.ADCRESULT1;
    adcb2result = AdcbResultRegs.ADCRESULT2;
    sampleCount++;
    sumAdcb0volts += -10.0 / 2048 * adcb0result + 10.0; // equation converting 12bit value to 10 to -10V voltages
    sumAdcb1volts +=
        (10.0 - 0.0) / (0.0 - 2048.0) * adcb1result + 10.0; // equation converting 12bit value to 10 to -10V voltages
    sumAdcb2volts += (3.0 - 0.0) / (4095.0 - 0.0) * adcb2result; // equation converting 12bit value to 0 to 3V voltages
    if (sampleCount == 16) {
        avgAdcb0 = sumAdcb0volts / 16.0; // oversampled value
        avgAdcb1 = sumAdcb1volts / 16.0; // oversampled value
        avgAdcb2 = sumAdcb2volts / 16.0; // oversampled value
        // Reset variables for next cycle of oversampling
        sampleCount = 0;
        sumAdcb0volts = 0;
        sumAdcb1volts = 0;
        sumAdcb2volts = 0;
        // This below line causes the SWI to run when priority permits
        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
    }
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/*
__interrupt void ADCB_ISR (void)
{
    adcb0result = AdcbResultRegs.ADCRESULT0;
    adcb1result = AdcbResultRegs.ADCRESULT1;
    adcb2result = AdcbResultRegs.ADCRESULT2;
    // Here convert ADCINB0, ADCINB1 and ADCINB2 to volts
    adcb0volt = -10.0/2048 * adcb0result + 10.0;
    adcb1volt = (10.0 - 0.0)/(0.0 - 2048.0) * adcb1result + 10.0;
    adcb2volt = (3.0-0.0)/(4095.0 - 0.0) * adcb2result;
    // Here write ADCINB0’s voltage value to DAC1 channel
    setDAC1(adcb0volt);
    // Print ADCINB0’s voltage to the text LCD every 100ms by setting UARTPrint to 1
    // and the function UART_printfLine is called in main()’s while(1) loop
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
*/
