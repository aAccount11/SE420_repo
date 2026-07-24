/* SERIAL.C: This code is designed to act as a low-level serial driver for
    higher-level programming.  Ideally, one could simply call init_serial()
    to initialize the serial port, then use serial_send("data", 4) to send
    an array of data (8-bit unsigned character strings).

    WRITTEN BY : Paul Miller <pamiller@uiuc.edu>
    $Id: serial.c,v 1.4 2003/08/08 16:08:56 paul Exp $
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <buffer.h>
#include <F28377sSerial.h>
#include <F2837xS_sci.h>
#include "SpiRAM.h"

serialSCIA_t SerialA;
serialSCIB_t SerialB; 
serialSCIC_t SerialC;

char RXAdata = 0;
char RXBdata = 0;
char RXCdata = 0;
uint32_t numRXA = 0;
uint32_t numRXB = 0;
uint32_t numRXC = 0;

float SIMU_value1_32bit = 0; // any float value
float SIMU_value2_32bit = 0; // any float value
float SIMU_value3_16bit = 0; // smaller float between -32 and 32 because 16bit
float SIMU_value4_16bit = 0; // smaller float between -32 and 3 because 16bits

// for SerialA
uint16_t init_serialSCIA(serialSCIA_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialA) {
        sci = &SciaRegs;
        s->sci = sci;
        init_bufferSCIA(&s->TX);

        GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
        GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
        GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_PUSHPULL);

    } else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialA) {
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
        IER |= (M_INT9);
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP9);

    }

    return 0;
}

void uninit_serialSCIA(serialSCIA_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialA) {
        PieCtrlRegs.PIEIER9.bit.INTx1 = 0;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 0;
        IER &= ~M_INT9;
    }
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCIA(serialSCIA_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCIA) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCIA_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}

// For SerialB
uint16_t init_serialSCIB(serialSCIB_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialB) {
        sci = &ScibRegs;
        s->sci = sci;
        init_bufferSCIB(&s->TX);

        GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 5);
        GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 5);
        GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_PUSHPULL);


    } else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialB) {
        PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx4 = 1;
        IER |= (M_INT9);
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP9);
    }

    return 0;
}

void uninit_serialSCIB(serialSCIB_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialB) {
        PieCtrlRegs.PIEIER9.bit.INTx3 = 0;
        PieCtrlRegs.PIEIER9.bit.INTx4 = 0;
        IER &= ~M_INT9;
    }

    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCIB(serialSCIB_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCIB) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCIB_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}

// for SerialC
uint16_t init_serialSCIC(serialSCIC_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialC) {
        sci = &ScicRegs;
        s->sci = sci;
        init_bufferSCIC(&s->TX);
        GPIO_SetupPinMux(90, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(90, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(89, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(89, GPIO_OUTPUT, GPIO_PUSHPULL);

    } else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialC) {
        PieCtrlRegs.PIEIER8.bit.INTx5 = 1;
        PieCtrlRegs.PIEIER8.bit.INTx6 = 1;
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP8);
        IER |= (M_INT8);

    } 

    return 0;
}

void uninit_serialSCIC(serialSCIC_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialC) {
        PieCtrlRegs.PIEIER8.bit.INTx5 = 0;
        PieCtrlRegs.PIEIER8.bit.INTx6 = 0;
        IER &= ~M_INT8;
    } 

    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCIC(serialSCIC_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCIC) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCIC_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}


/***************************************************************************
 * TXxINT_DATA_SENT()
 *
 * Executed when transmission is ready for additional data.  These functions
 * read the next char of data and put it in the TXBUF register for transfer.
 ***************************************************************************/
#ifdef _FLASH
#pragma CODE_SECTION(TXAINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXAINT_data_sent(void)
{
    char data;
    if (buf_readSCIA_1(&SerialA.TX,0,&data) == 0) {
        while ( (buf_readSCIA_1(&SerialA.TX,0,&data) == 0)
                && (SerialA.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCIA(&SerialA.TX, 1);
            SerialA.sci->SCITXBUF.all = data;
        }
    } else {
        SerialA.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialA.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


//for serialB
#ifdef _FLASH
#pragma CODE_SECTION(TXBINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXBINT_data_sent(void)
{
    char data;
    if (buf_readSCIB_1(&SerialB.TX,0,&data) == 0) {
        while ( (buf_readSCIB_1(&SerialB.TX,0,&data) == 0)
                && (SerialB.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCIB(&SerialB.TX, 1);
            SerialB.sci->SCITXBUF.all = data;
        }
    } else {
        SerialB.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialB.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


//for serialC
#ifdef _FLASH
#pragma CODE_SECTION(TXCINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXCINT_data_sent(void)
{
    char data;
    if (buf_readSCIC_1(&SerialC.TX,0,&data) == 0) {
        while ( (buf_readSCIC_1(&SerialC.TX,0,&data) == 0)
                && (SerialC.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCIC(&SerialC.TX, 1);
            SerialC.sci->SCITXBUF.all = data;
        }
    } else {
        SerialC.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialC.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}


extern uint16_t SpiRAM_transmit_done;
uint16_t SpiRAM_matlab_state = 0; //use for getting sci command
char SpiRAM_send_info[3];         // send "*, NUM_PACKET, NUM_VAR"
extern uint16_t SpiRAM_spi_state;
extern uint32_t SpiRAM_read_packet_count; // the location to read the packet
extern uint16_t SpiRAM_spi_read_count;    // number of total spi read
extern uint16_t SpiRAM_buf_len;           // number of things in the buffer
extern uint32_t SpiRAM_time_entered;      // for assigning address for the writing with DMA

// for SerialA
#ifdef _FLASH
#pragma CODE_SECTION(RXAINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXAINT_recv_ready(void)
{
    RXAdata = SciaRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXAdata & 0xC000)
    {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    }
    else
    {
        RXAdata = RXAdata & 0x00FF;
        numRXA++;
        // New Code to come
        if (SpiRAM_transmit_done)
        {
            // wait for 0xAA
            if (SpiRAM_matlab_state == 0)
            {
                if (RXAdata == 0xAA)
                {
                    SpiRAM_matlab_state = 1;
                }
                else if (RXAdata == 0xCC)
                {
                    SpiRAM_matlab_state = 10;
                }
                else
                {
                    SpiRAM_matlab_state = 0;
                }
            }
            else if (SpiRAM_matlab_state == 1)
            {
                // wait for 0x55
                if (RXAdata == 0x55)
                {
                    // Set to GPIO61 to manually move chip select high low when not using DMA.
                    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);  // Set as GPIO61
                    SpiRAM_matlab_state = 2;
                    // setup spi rx interrupt for getting data from spiram
                    SpiRAM_send_info[0] = '*';
                    SpiRAM_send_info[1] = NUM_PACKET;
                    SpiRAM_send_info[2] = NUM_VAR;
                    serial_sendSCIA(&SerialA, SpiRAM_send_info, 3);
                    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
                    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
                    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
                    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;    // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL
                    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
                    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
                    SpiRAM_spi_state = 1;
                    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;   // slave select low
                    SpiaRegs.SPITXBUF = 0x0540;             // write0x01 sequential mode(100 0000)
                }
                else
                {
                    SpiRAM_matlab_state = 0;
                }
            }
            else if (SpiRAM_matlab_state == 2)
            {
                if (RXAdata == 0xDD)
                {
                    // Set to GPIO61 to manually move chip select high low when not using DMA.
                    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);  // Set as GPIO61
                    SpiRAM_spi_state = 1;

                    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   // Write 0 to reset the FIFO pointer to zero, and hold in reset
                    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
                    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
                    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;      // Enable the RX FIFO Interrupt.  RXFFST >= RXFFIL
                    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;   // Re-enable receive FIFO operation
                    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;   // slave select low
                    SpiaRegs.SPITXBUF = 0x0540;             // write0x01 sequential mode(100 0000)

                    SpiRAM_read_packet_count += 1;
                    SpiRAM_spi_read_count = 0;
                    if (SpiRAM_read_packet_count == NUM_PACKET * NUM_VAR)
                    {
                        SpiRAM_read_packet_count = 0; // reset packet read location when done reading everything
                    }
                }
                else if (RXAdata == 0xBB)
                {
                    SpiRAM_matlab_state = 0;
                    SpiRAM_spi_state = 0;
                    SpiRAM_spi_read_count = 0;
                    SpiaRegs.SPIFFRX.bit.RXFFIL = 2;
                }
            }
            else if (SpiRAM_matlab_state == 10)
            { // get data command
                SpiRAM_matlab_state = 0;
                if (RXAdata == 0x55)
                {
                    SpiRAM_transmit_done = 0; // start transmitting
                    GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;  // Turn off status LED for SPIRAM transfer
                    SpiRAM_spi_state = 0;
                    SpiRAM_spi_read_count = 0;
                    SpiRAM_buf_len = 0;
                    SpiRAM_time_entered = 0;
                    SpiRAM_read_packet_count = 0; // reset packet read for next new packet
                }
            }
        }
    }

    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

char SIMU_databyte1 = 0;
int SIMU_Var1_fromSIMU_16bit = 0;
int SIMU_Var2_fromSIMU_16bit = 0;
int SIMU_Var3_fromSIMU_16bit = 0;
int SIMU_Var4_fromSIMU_16bit = 0;
int SIMU_Var5_fromSIMU_16bit = 0;
int SIMU_Var6_fromSIMU_16bit = 0;
int SIMU_Var7_fromSIMU_16bit = 0;

char SIMU_TXrawbytes[12];

long SIMU_Var1_toSIMU_32bit = 0;
long SIMU_Var2_toSIMU_32bit = 0;
long SIMU_Var3_toSIMU_32bit = 0;
long SIMU_Var4_toSIMU_32bit = 0;
long SIMU_Var5_toSIMU_32bit = 0;
long SIMU_Var6_toSIMU_32bit = 0;
long SIMU_Var7_toSIMU_32bit = 0;

long SIMU_Var1_toSIMU_16bit = 0;
long SIMU_Var2_toSIMU_16bit = 0;
long SIMU_Var3_toSIMU_16bit = 0;
long SIMU_Var4_toSIMU_16bit = 0;
long SIMU_Var5_toSIMU_16bit = 0;
long SIMU_Var6_toSIMU_16bit = 0;
long SIMU_Var7_toSIMU_16bit = 0;

int SIMU_beginnewdata = 0;
int SIMU_datacollect = 0;
int SIMU_Tranaction_Type = 0;
int SIMU_checkfirstcommandbyte = 0;

//for SerialB
#ifdef _FLASH
#pragma CODE_SECTION(RXBINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXBINT_recv_ready(void)
{
    RXBdata = ScibRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXBdata & 0xC000) {
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        RXBdata = RXBdata & 0x00FF;
        // Do something with recieved character
        numRXB ++;
        if (!SIMU_beginnewdata)
        { // Only true if have not yet begun a message
            if (SIMU_checkfirstcommandbyte == 1)
            {
                if (0xFF == (unsigned char)RXBdata)
                { // Check for start 2 bytes command = 32767 becuase assuming command will stay between -10000 and 10000
                    SIMU_checkfirstcommandbyte = 0;
                }
            }
            else
            {
                SIMU_checkfirstcommandbyte = 1;
                if (0x7F == (unsigned char)RXBdata)
                { // Check for start char

                    SIMU_datacollect = 0;  // amount of data collected in message set to 0
                    SIMU_beginnewdata = 1; // flag to indicate we are collecting a message

                    SIMU_Tranaction_Type = 2;

                    SIMU_Var1_toSIMU_32bit = SIMU_value1_32bit*10000;
                    SIMU_Var2_toSIMU_32bit = SIMU_value2_32bit*10000;
                    SIMU_Var1_toSIMU_16bit = SIMU_value3_16bit*1000;
                    SIMU_Var2_toSIMU_16bit = SIMU_value4_16bit*1000;
                    SIMU_TXrawbytes[3] = (char)((SIMU_Var1_toSIMU_32bit >> 24) & 0xFF);
                    SIMU_TXrawbytes[2] = (char)((SIMU_Var1_toSIMU_32bit >> 16) & 0xFF);
                    SIMU_TXrawbytes[1] = (char)((SIMU_Var1_toSIMU_32bit >> 8) & 0xFF);
                    SIMU_TXrawbytes[0] = (char)((SIMU_Var1_toSIMU_32bit)&0xFF);

                    SIMU_TXrawbytes[7] = (char)((SIMU_Var2_toSIMU_32bit >> 24) & 0xFF);
                    SIMU_TXrawbytes[6] = (char)((SIMU_Var2_toSIMU_32bit >> 16) & 0xFF);
                    SIMU_TXrawbytes[5] = (char)((SIMU_Var2_toSIMU_32bit >> 8) & 0xFF);
                    SIMU_TXrawbytes[4] = (char)((SIMU_Var2_toSIMU_32bit)&0xFF);

                    SIMU_TXrawbytes[8] = (char)(SIMU_Var1_toSIMU_16bit & 0xFF);
                    SIMU_TXrawbytes[9] = (char)((SIMU_Var1_toSIMU_16bit >> 8) & 0xFF);
                    SIMU_TXrawbytes[10] = (char)(SIMU_Var2_toSIMU_16bit & 0xFF);
                    SIMU_TXrawbytes[11] = (char)((SIMU_Var2_toSIMU_16bit >> 8) & 0xFF);

                    serial_sendSCIB(&SerialB, SIMU_TXrawbytes, 12);
                }
            }
        }
        else
        { // Filling data
            if (SIMU_Tranaction_Type == 2)
            {
                if (SIMU_datacollect == 0)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 1)
                {

                    SIMU_Var1_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;

                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 2)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 3)
                {

                    SIMU_Var2_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;

                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 4)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 5)
                {

                    SIMU_Var3_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 6)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 7)
                {

                    SIMU_Var4_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 8)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 9)
                {

                    SIMU_Var5_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 10)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 11)
                {
                    SIMU_Var6_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 12)
                {
                    SIMU_databyte1 = RXBdata;
                    SIMU_datacollect++;
                }
                else if (SIMU_datacollect == 13)
                {
                    SIMU_Var7_fromSIMU_16bit = ((int)RXBdata) << 8 | SIMU_databyte1;

                    SIMU_beginnewdata = 0; // Reset the flag
                    SIMU_datacollect = 0;  // Reset the number of chars collected
                    SIMU_Tranaction_Type = 0;
                }
            }
        }
    }

    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

// for SerialC
#ifdef _FLASH
#pragma CODE_SECTION(RXCINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXCINT_recv_ready(void)
{
    RXCdata = ScicRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXCdata & 0xC000)
    {
        ScicRegs.SCICTL1.bit.SWRESET = 0;
        ScicRegs.SCICTL1.bit.SWRESET = 1;
        ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;
        ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
    }
    else
    {
        RXCdata = RXCdata & 0x00FF;
        numRXC++;
    }

    ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

// SerialA only setup for Tera Term connection
char serial_printf_bufSCIA[BUF_SIZESCIA];

uint16_t serial_printf(serialSCIA_t *s, char *fmt, ...)
{
    va_list ap;

    va_start(ap,fmt);
    vsprintf(serial_printf_bufSCIA,fmt,ap);
    va_end(ap);

    return serial_sendSCIA(s,serial_printf_bufSCIA,strlen(serial_printf_bufSCIA));
}

//For Text LCD  SCIC only
char UART_printf_bufferSCIC[BUF_SIZESCIC];
void UART_vprintfLine(unsigned char line, char *format, va_list ap)
{
    char sendmsg[24];

    int i;

    vsprintf(UART_printf_bufferSCIC,format,ap);

    // Add header information and pad end of transfer with spaces to clear display
    sendmsg[0] = 0xFE;
    sendmsg[1] = 'G';
    sendmsg[2] = 1;
    sendmsg[3] = line;
    for (i=4;i<24;i++) {
        if (i >= strlen(UART_printf_bufferSCIC)+4) {
            sendmsg[i] = ' ';
        } else {
            sendmsg[i] = UART_printf_bufferSCIC[i-4];
        }
    }
    serial_sendSCIC(&SerialC,sendmsg,24);
}

void UART_printfLine(unsigned char line, char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    UART_vprintfLine(line,format,ap);
}

