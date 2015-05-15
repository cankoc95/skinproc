#include <xc.h>
#include "setup.h"

#if defined(__8x2Shell)
    const char select_list[(ROWS*COLUMNS-IGNORE_CELLS)*2] = {0,0,1,0,2,0,3,0,4,1,5,1,6,1,7,1};
#endif

void setupADC(void)
{

	// Set RB2 (AN2) as input
	_TRISB2 = 1;

	// --------------------------------
	// AD1CON1: ADC1 Control Register 1
	// --------------------------------
	AD1CON1bits.ADSIDL 	= 0;					// Continue in idle mode
	AD1CON1bits.ADDMABM = 1;					// Write DMA buffers in conv. order (DMA disabled)
	AD1CON1bits.AD12B 	= 1;					// 12-bit single channel
	AD1CON1bits.FORM 	= 0b00;					// Integer output
	AD1CON1bits.SSRC 	= 0b111;				// Auto-conversion (set time after sampling)
	AD1CON1bits.SIMSAM 	= 0;					// Disable simultaneous sampling
	AD1CON1bits.ASAM 	= 0;					// Sampling triggered by SAMP bit

	// --------------------------------
	// AD1CON2: ADC1 Control Register 2
	// --------------------------------
	AD1CON2bits.VCFG 	= 0b011;//0b000;				// Set AVDD and AVSS as A/D references
	AD1CON2bits.CSCNA 	= 0;					// Disable CH0 input scanning
	AD1CON2bits.CHPS 	= 0b00;					// Single channel (CH0)
	AD1CON2bits.SMPI 	= 0b0000;				// Incr. DMA pointer after every conv. (DMA disabled)
	AD1CON2bits.BUFM 	= 0;					// Fill DMA buffer from top (DMA disabled)
	AD1CON2bits.ALTS 	= 0;					// Alternate input disabled

	// --------------------------------
	// AD1CON3: ADC1 Control Register 3
	// --------------------------------
	AD1CON3bits.ADRC 	= 0;					// Run on system clock
	AD1CON3bits.SAMC 	= /*0b10000;*/0b00101;				// Auto sample time = 5*Tad (Rec. Min = 3Tad)
	AD1CON3bits.ADCS 	= /*0b0010000;*/0b0000110;			// Tad = 7*Tcy (note + 1) (Rec. Min = 5Tad)

	// --------------------------------
	// AD1CON4: ADC1 Control Register 4
	// --------------------------------
	AD1CON4bits.DMABL 	= 0b000;				// One word of buffer per AN input (DMA disabled)

	// ---------------------------------------------------
	// AD1CHS123: ADC1 Input Channel 1,2,3 Select Register
	// ---------------------------------------------------
	AD1CHS123 = 0;								// Don't care

	// ---------------------------------------------
	// AD1CHS0: ADC1 Input Channel 0 Select Register
	// ---------------------------------------------
	AD1CHS0bits.CH0NA 	= 0;					// CH0 negative input is VREFL
	AD1CHS0bits.CH0SA 	= 0b00010;				// CH0 positive input is AN2

	// --------------------------------------------
	// AD1CCSL: ADC1 Input Scan Select Register Low
	// --------------------------------------------
	AD1CSSL = 0;								// Don't care (input scan disabled)

	// ----------------------------------------------
	// AD1PCFGL: ADC1 Port Configuration Register Low
	// ----------------------------------------------
	AD1PCFGLbits.PCFG2 = 0;						// AN2 is analog

	// Activate ADC1
	AD1CON1bits.ADON = 1;

}


void setupTimer(unsigned char tmrPreScale, unsigned char tmrPerL, unsigned char tmrPerH)
{

	// -------------------------------
	// T1CON: Timer 1 Control Register
	// -------------------------------
	T1CONbits.TSIDL	= 0;				// Continue in idle mode
	T1CONbits.TGATE = 0;				// Gated time accumulation disabled
	T1CONbits.TCKPS = tmrPreScale; 		// Timer pre-scale factor
	T1CONbits.TSYNC = 0;				// Using internal clock, ignored
	T1CONbits.TCS = 0;					// Use internal clock

	// ----------------------------
	// PR1: Timer 1 Period Register
	// ----------------------------
	// F = (Fcy)/(PreScale*(Period + 1))
	PR1 = (unsigned int)(tmrPerL) + ((unsigned int)(tmrPerH)<<8); 			// Period

	// --------------------------------
	// T1IP: Timer 1 Interrupt Priority
	// --------------------------------
	_T1IP 	= 7;						// Highest priority

}


void setupUART(void)
{
	unsigned char rxChar = 0;

	// ---------------------------
	// U1MODE: UART1 Mode Register
	// ---------------------------
	U1MODEbits.USIDL 	= 0;				// Continue in idle mode
	U1MODEbits.IREN 	= 0;				// IrDA disabled
	U1MODEbits.RTSMD 	= 0;				// CTS/RTS flow control mode
	U1MODEbits.UEN 		= 0b00;				// U1Tx, U1Rx pins enabled and used
	U1MODEbits.WAKE 	= 0;				// Wake-up disabled
	U1MODEbits.LPBACK 	= 0;				// Loopback disabled
	U1MODEbits.ABAUD 	= 0;				// Auto-baud disabled
	U1MODEbits.URXINV 	= 0;				// U1RX idle state is 1
	U1MODEbits.BRGH 	= 0b01; //U1BRGHVAL;		// High-speed mode, 0 for lowspeed buspirate
	U1MODEbits.PDSEL 	= 0b00;				// 8-bit, no parity
	U1MODEbits.STSEL 	= 0;				// 1 stop bit

	// ----------------------------------------
	// U1STA: UART1 Status and Control Register
	// ----------------------------------------
	U1STAbits.UTXISEL1 	= 0;				// Interrupt when position opens up
	U1STAbits.UTXISEL0 	= 0;				//    in transmit buffer
	U1STAbits.UTXINV 	= 0;				// U1TX idle state is 1
	U1STAbits.UTXBRK	= 0;				// Sync break transmission disabled
	U1STAbits.URXISEL	= 0b00;				// Interrupt when character received
	U1STAbits.ADDEN		= 0;				// Address detect disabled

	// -------------------------------
	// U1BRG: UART1 Baud Rate Register
	// -------------------------------

	/*  High-Speed Mode:

		Desired Baud Rate = (Fcy)/(4 * (U1BRG + 1))
		U1BRG = (Fcy/Desired Baud Rate)/(4) - 1

		Low-Speed Mode:

		Desired Baud Rate = (Fcy)/(16 * (U1BRG + 1))
		U1BRG = (Fcy/Desired Baud Rate)/(16) - 1
	*/
	U1BRG = 86; //U1BRGVAL; //21 for buspirate; //86 for high speed;


	// Enable UART
	U1MODEbits.UARTEN 	= 1;				// UART1 Enabled
	U1STAbits.UTXEN		= 1;				// Transmit enabled

	// Clear receive buffer, if necessary
	while (U1STAbits.URXDA)
	{
		rxChar = U1RXREG;
	}
	U1STAbits.OERR = 0;

}


void setupSkin(void)
{

	// Enable multiplexer inputs
	_TRISE4 = 0; _TRISE5 = 0; _TRISE6 = 0; _TRISE7 = 0;
	_TRISD1 = 0; _TRISD2 = 0; _TRISD3 = 0; _TRISD4 = 0;

	// Enable multiplexers
	_TRISG6 = 0; _LATG6 = 1;
	_TRISD5 = 0; _LATD5 = 1;

        // Setup LEDs
        _TRISB12 = 0; LEDRED = 0;
        _TRISB13 = 0; LEDGREEN = 0;
        _TRISB14 = 0; LEDYELLOW = 0;

}

