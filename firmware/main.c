/* 
 * File:   main.c
 * Author: jgoldberg
 *
 * Created on February 11, 2014, 11:37 AM
 */

#include <stdio.h>
#include <stdlib.h>





/*
Main driver for SkinProc software.

jkarras@eecs
March 2013
*/


// ---------
// LIBRARIES
// ---------
#include <xc.h>
#include <utils.h>
#include <init_default.h>
#include <stopwatch.h>


// --------------------
// PREPROCESSOR DEFINES
// --------------------
#define LEDRED			_LATB12
#define LEDGREEN		_LATB13
#define LEDYELLOW		_LATB14


// --------------
// Sensor Defines
// --------------

// 6x1 Shell Column Sensor (TESTING CONFIGURATION)
#if defined(__6x1Shell)

	// Sensor Dimensions
	#define ROWS 			6
	#define COLUMNS 		1
	#define FIRSTROW		0
	#define FIRSTCOL		0

	// UART Configuration
	#define U1BRGVAL		9								// For 1 Mbps (HIGH SPEED)
	#define U1BRGHVAL		1								// High Speed Mode

#elif defined(__9x6Shell)

	// Sensor Dimensions
	#define ROWS 			9
	#define COLUMNS 		6
	#define FIRSTROW		0
	#define FIRSTCOL		0

	// UART Configuration
	#define U1BRGVAL		9								// For 1 Mbps (HIGH SPEED)
	#define U1BRGHVAL		1								// High Speed Mode

#elif defined(__5x8Hairs)

	// Sensor Dimensions
	#define ROWS 			5
	#define COLUMNS 		8
	#define FIRSTROW		0
	#define FIRSTCOL		0

	// UART Configuration
	#define U1BRGVAL		9								// For 1 Mbps (HIGH SPEED)
	#define U1BRGHVAL		1								// High Speed Mode

#elif defined(__2x7Bumper)

	// Sensor Dimensions
	#define ROWS 			2
	#define COLUMNS 		7
	#define FIRSTROW		0
	#define FIRSTCOL		0

	// UART Configuration
	#define U1BRGVAL		9								// For 1 Mbps (HIGH SPEED)
	#define U1BRGHVAL		1								// High Speed Mode

#endif


// 5x4 Hair Sensor
#if defined(__5x4Hairs)

	// Visualization Timer Configuration
	#define VIS_TMRVAL			24999					// For 200 Hz interrupt
	#define VIS_TMRPRESCALE		0b01					// Prescale = 1:8

	// Data Timer Configuration
	#define DATA_TMRVAL			49999					// For 100 Hz interrupt
	#define DATA_TMRPRESCALE	0b01					// Prescale = 1:8

	// 2k Hz Timer Configuration
	#define TWO_K_TMRVAL		19999					// For 2k Hz interrupt
	#define TWO_K_TMRPRESCALE	0b00					// Prescale = 1:1

	// Sensor Dimensions
	#define ROWS 			5
	#define COLUMNS 		4
	#define FIRSTROW		0
	#define FIRSTCOL		0

#endif


// U1BRGVAL		38										// For 64 Kbps (LOW SPEED)

// Calibration Parameters
#define FRAME_SAMPLES			50
#define FRAME_SAMPLE_DELAY_MS           50

#define PIXEL_HOLD_TIME_MS              10
#define PIXEL_SAMPLES                   50

#define PIXEL_SAMPLES_INT               3

// Filtering Parameters
#define FILTERLEN                       0
#define VISUALTHRESH                    3000
#define RESPTHRESH                      500
#define EDGE_PIXEL_SAMPLES              3
#define RISE_EDGETHRESH                 1500
#define FALL_EDGETHRESH                 500
#define EDGE_HISTORY                    5
#define TENTH_MM_SEC                    240000000
#define MAX_TENTH_MM_SEC                24000				// Maximum measurable speed (100Hz. frame rate)



// SkinProc States
#define IDLE				0
#define SCANNING			1

/*
#define VISUALSCAN			1
#define RUNNINGSCAN			2
#define TACTILERESP1		3
#define SPEEDESTIMATE		4
*/


// -------
// GLOBALS
// -------
unsigned int frame[ROWS][COLUMNS];
//unsigned char framePixels = ROWS * COLUMNS;
//unsigned char frameIndex;


unsigned int offset[ROWS][COLUMNS];
int sensor[ROWS][COLUMNS][FILTERLEN + 1];
int complete_frame[ROWS][COLUMNS];
unsigned char sensor_state_array[ROWS][COLUMNS];
unsigned char edge_type_array[ROWS][COLUMNS][EDGE_HISTORY];
unsigned char edge_idx_array[ROWS][COLUMNS];
unsigned long edge_time_array[ROWS][COLUMNS][EDGE_HISTORY];
unsigned int tactileState;

unsigned char row, col;

unsigned char state = IDLE;

unsigned char frameComplete;

// ---------------------
// FUNCTION DECLARATIONS
// ---------------------
void setupSkinProc(void);
void setupADC(void);
void setupTimer(unsigned char, unsigned char, unsigned char);
void setupUART(void);
void setupSkin(void);

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void handleT1Interrupt(void);
void handleT1SpeedInterrupt(void);

void advancePixel(void);
void adjustMux(void);

void samplePixel(unsigned char, unsigned char);
void sampleFrame(unsigned int);
void pollPixel(unsigned char, unsigned char, unsigned char, unsigned char);
void startScan(void);
void stopScan(void);


void calibrateSkin(void);
void returnCalibration(void);
void startTimer(void);
void stopTimer(void);
void pollColumn(unsigned char, unsigned char);

unsigned char rxChar = 0;

void main(void)
{
	
	unsigned char rxRow, rxCol;
	unsigned char sampPerL, sampPerH;
	unsigned char tmrPreScale, tmrPeriodL, tmrPeriodH;
	unsigned char rxDur, rxPer;

	unsigned char i, j;
	unsigned int sample;

	// Setup hardware
	setupSkinProc();


	/*
	Listen for commands:
	'A' -- Sample Pixel
	'B' -- Sample Frame
	'C' -- Poll Pixel
	'D' -- Set Scan Rate
	'E' -- Start Scan
	'F' -- Stop Scan


	--- OLD ---
	'C'	-- Calibrate Skin
	'G' -- Get Calibration
	'S' -- Start Visual Scan
	'R' -- State Running Scan
	'P' -- Sample Pixel
	'F' -- Sample Frame
	'B' -- Poll Column
	'T' -- Tactile Response #1 - Squeeze
	'U' -- Tactile Response #2
	'W' -- Tactile Response #3
	'L' -- Run Ground Speed Estimation
	*/

	// rxChar = 'L';

        LEDRED = 0;
        LEDGREEN = 0;
        LEDYELLOW = 0;
        delay_ms(1000);
        //rxChar = 'G';
	while (1)
	{

		// ----
		// IDLE
		// ----

		if (state == IDLE)
		{
                    //test TX
                    /*LEDRED = 1;
                    while (U1STAbits.UTXBF);
                    U1TXREG = (unsigned char)rxChar;
                    delay_ms(500);
                    LEDRED = 0;
                    delay_ms(1500);
                    rxChar = ~rxChar;
                    */
                    //test RX
                    /*
                        while (!U1STAbits.URXDA);
			rxChar = U1RXREG;
                        LEDRED = 0;
                        LEDGREEN = 1;
                        
                        if (rxChar == 'A') {
                            LEDYELLOW = 1;
                        }
                        
                        delay_ms(500);
                        LEDYELLOW = 0;
                        LEDGREEN = 0;
                        LEDRED = 1;
                     */

                        /*
                        //if (rxChar == 0x00) {
                        //    LEDRED = 0;
                        //    LEDGREEN = 1;
                        //}
                        while (U1STAbits.UTXBF);
                        rxChar = 'B';
                        U1TXREG = (unsigned char)rxChar;

                        LEDRED = 1;
                        */
                        //rxChar = 'G';
                        //delay_ms(5000);
                        while (!U1STAbits.URXDA);
			rxChar = U1RXREG;
                        

			switch (rxChar)
			{
				case 'A':	// Sample Pixel
							// Wait for row, column coordinates
							while (!U1STAbits.URXDA);
							rxRow = U1RXREG;
							while (!U1STAbits.URXDA);
							rxCol = U1RXREG;
							
                                                        //Nop();
                                                        //Nop();

                                                        samplePixel(rxRow, rxCol);
							break;

				case 'B':	// Sample Frame
							while (!U1STAbits.URXDA);
							sampPerL = U1RXREG;
							while (!U1STAbits.URXDA);
							sampPerH = U1RXREG;
							sampleFrame( (unsigned int)(sampPerL) + ((unsigned int)(sampPerH) << 8) );
							break;

				case 'C':	// Poll Pixel
							// Wait for row, column, duration, sample period
							while (!U1STAbits.URXDA);
							rxRow = U1RXREG;
							while (!U1STAbits.URXDA);
							rxCol = U1RXREG;
							while (!U1STAbits.URXDA);
							rxDur = U1RXREG;
							while (!U1STAbits.URXDA);
							rxPer = U1RXREG;
							pollPixel(rxRow, rxCol, rxDur, rxPer);
							break;

				case 'D':	// Set Scan Rate
							// Wait for pre-scale factor, period
							while (!U1STAbits.URXDA);
							tmrPreScale = U1RXREG;
							while (!U1STAbits.URXDA);
							tmrPeriodL = U1RXREG;
							while (!U1STAbits.URXDA);
							tmrPeriodH = U1RXREG;
							setupTimer(tmrPreScale, tmrPeriodL, tmrPeriodH);
							break;

				case 'E':	startScan();
							break;

				case 'F':	stopScan();
							break;
                                case 'G':	sendSize();
							break;
                                case 'T':       sendTestFrame();
                                                        break;
				default:	break;

				/*

				case 'C':	calibrateSkin();
							break;

				case 'G':	returnCalibration();
							break;

				case 'S':	state = VISUALSCAN;
							row = 0;
							col = 0;
							adjustMux();
							delay_us(500);
							setupTimer(VIS_TMRPRESCALE, VIS_TMRVAL);
							startTimer();
							LEDRED = 1;
							break;

				case 'R':	state = RUNNINGSCAN;
							tactileState = 0;
							row = 0;
							col = 0;
							adjustMux();
							delay_us(500);
							setupTimer(DATA_TMRPRESCALE, DATA_TMRVAL);
							startTimer();
							LEDRED = 1;
							break;






				case 'B':	// Poll Column
							// Wait for column, duration
							while (!U1STAbits.URXDA);
							rxCol = U1RXREG;
							while (!U1STAbits.URXDA);
							rxDur = U1RXREG;
							pollColumn(rxCol, rxDur);
							break;

				case 'T':	state = TACTILERESP1;
							row = 0;
							col = 0;
							adjustMux();
							delay_us(500);
							frameComplete = 0;
							tactileAccum = 0;
							setupTimer(DATA_TMRPRESCALE, DATA_TMRVAL);
							startTimer();
							LEDRED = 1;
							break;

				case 'L':	state = SPEEDESTIMATE;
							row = 0;
							col = 0;
							adjustMux();
							delay_us(500);
							frameComplete = 0;


							// -----------------------------
							// NOTE: CALIBRATE THIS LATER!!!
							// -----------------------------
							// Initialize sensor state array
							// 0 - Low
							// 1 - High
							for (i = 0; i < ROWS; ++i)
								for (j = 0; j < COLUMNS; ++j)
									sensor_state_array[i][j] = 0;

							// Initialize edge type, time arrays
							// 0 - No edge detected
							// 1 - Rising
							// 2 - Falling
							for (i = 0; i < ROWS; ++i)
								for (j = 0; j < COLUMNS; ++j)
									edge_idx_array[i][j] = 0;
									for (k = 0; k < EDGE_HISTORY; ++k)
									{
										edge_type_array[i][j][k] = 0;
										edge_time_array[i][j][k] = 0;
									}

							// Reset variables
							lastEstimate = 0;
							ceilingFlag = 0;
							ceilingEdge = 0;

							//setupTimer(TWO_K_TMRPRESCALE, TWO_K_TMRVAL);
							setupTimer(VIS_TMRPRESCALE, VIS_TMRVAL);

							swatchReset();
							swatchTic();

							startTimer();

							LEDRED = 1;
							break;

				default:	break;

				*/

			}
		}

		else
		{

			// --------
			// SCANNING
			// --------
			if (state == SCANNING)
			{
				// Wait for complete frame
				if (frameComplete)
				{
					for (i = 0; i < ROWS; ++i)
					{
						for (j = 0; j < COLUMNS; ++j)
						{
							sample =  frame[i][j];
							while (U1STAbits.UTXBF);
							U1TXREG = (unsigned char)(sample % 256);
							while (U1STAbits.UTXBF);
							U1TXREG = (unsigned char)(sample >> 8);
						}
					}
					frameComplete = 0;
				}

				// ------------
				// STOP COMMAND
				// ------------
				if(U1STAbits.URXDA)
				{
					rxChar = U1RXREG;

					if (rxChar == 'F')
					{
						stopScan();
					}

				} // end if stop command

			} // end if scanning


		} // End else (not IDLE)

	} // End while(1)

} // End main()


// ---------------
// Setup Functions
// ---------------


void setupSkinProc()
{
	unsigned char i;

	// Setup hardware
	SetupClock();
	SwitchClocks();
	setupADC();
	setupUART();
	setupSkin();
	swatchSetup();

	// Initialize state to IDLE
	state = IDLE;

	// Setup LEDs
	_TRISB12 = 0; LEDRED = 0;
	_TRISB13 = 0; LEDGREEN = 0;
	_TRISB14 = 0; LEDYELLOW = 0;

	// Toggle LEDs to signal completion
	for (i = 0; i < 5; ++i)
	{
		LEDYELLOW = 0; LEDRED = 1;
		delay_ms(100);

		LEDRED = 0; LEDGREEN = 1;
		delay_ms(100);

		LEDGREEN = 0; LEDYELLOW = 1;
		delay_ms(100);
	}
	LEDYELLOW = 0;

}


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
	U1MODEbits.BRGH 	= 0; //U1BRGHVAL;		// High-speed mode, 0 for lowspeed buspirate
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
	U1BRG = 21; //U1BRGVAL; //21 for buspirate; //86 for high speed;


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

/*
	// Initialize sensor array
	for (i = 0; i < ROWS; ++i)
		for (j = 0; j < COLUMNS; ++j)
		{
			offset[i][j] = 0;
			for (k = 0; k < FILTERLEN + 1; ++k)
				sensor[i][j][k] = 0;
		}

	// Initialize tactileState variable
	tactileState = 0;
*/

	// Initialize current row, column indices
	row = 0;
	col = 0;
	frameComplete = 0;

	// Adjust multiplexer inputs
	adjustMux();
	delay_us(500);
}


// ------------------
// Interrupt Routines
// ------------------


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
	handleT1Interrupt();

    // Clear timer 1 interrupt flag
    _T1IF = 0;
}


void handleT1Interrupt(void)
{
	unsigned int sample = 0;

	// Sample current pixel
	AD1CON1bits.SAMP = 1;
	while (!AD1CON1bits.DONE);
	sample = ADC1BUF0;
	AD1CON1bits.DONE = 0;

	frame[row][col] = sample;

	// Move onto next pixel (no delay -- handled by interrupt period)
	advancePixel();

}


// -------------------
// Multiplex Functions
// -------------------


void advancePixel(void)
{
	// Advance to next pixel
	if (col == COLUMNS - 1)
	{
		col = 0;
		if (row == ROWS - 1)
		{
			row = 0;
			// Raise flag to indicate completion of frame
			frameComplete = 1;
		}
		else
			row++;
	}
	else
		col++;

	// Adjust multiplexer inputs
	adjustMux();
}


void adjustMux(void)
{
	unsigned char colPins, rowPins;

	/*
	INPUT (SENSED) MULTIPLEXER:
	A[3] => RE4
	A[2] => RE5
	A[1] => RE6
	A[0] => RE7

	OUTPUT (DRIVE) MULTIPLEXER:
	A[3] => RD1
	A[2] => RD2
	A[1] => RD3
	A[0] => RD4
	*/

	// Apply pin offsets (to account for wiring)
	colPins = col + FIRSTCOL;
	rowPins = row + FIRSTROW;

	// Inputs (Columns -- Sensed)
	_LATE7 = colPins % 2;
	_LATE6 = (colPins >> 1) % 2;
	_LATE5 = (colPins >> 2) % 2;
	_LATE4 = (colPins >> 3) % 2;

	// Outputs (Rows -- Driven)
	_LATD4 = rowPins % 2;
	_LATD3 = (rowPins >> 1) % 2;
	_LATD2 = (rowPins >> 2) % 2;
	_LATD1 = (rowPins >> 3) % 2;

	// ----------------------------------------------
	// NOTE: NEED TO DELAY AFTER FUNCTION CALL
	//
	// Remember: low-pass filter on first amp stage
	// Need 500us delay to let signals settle down:
	//    delay_us(500);
	// ----------------------------------------------

}


// -------------------
// Accessory Functions
// -------------------


void samplePixel(unsigned char rowCoord, unsigned char colCoord)
{
	unsigned int sample = 0;

	LEDRED = 1;
        echoCommand();
	// Set multiplexers
	row = rowCoord;
	col = colCoord;
	adjustMux();

	// DELAY.. need to experiment with this
	delay_us(500);

	// Sample requested pixel
	AD1CON1bits.SAMP = 1;
	while (!AD1CON1bits.DONE);
	sample = ADC1BUF0;
	AD1CON1bits.DONE = 0;

	// Return result
        while (U1STAbits.UTXBF);
	U1TXREG = row;
        while (U1STAbits.UTXBF);
	U1TXREG = col;
	while (U1STAbits.UTXBF);
	U1TXREG = (unsigned char)(sample % 256);
	while (U1STAbits.UTXBF);
	U1TXREG = (unsigned char)(sample >> 8);

	LEDRED = 0;
}


void sampleFrame(unsigned int sampPer)
{
	unsigned int sample;
	unsigned int sampled_frame[ROWS][COLUMNS];

	LEDRED = 1;
        echoCommand();
	// Iterate over pixels
	for (row = 0; row < ROWS; ++row)
	{
		for (col = 0; col < COLUMNS; ++col)
		{
			// Adjust multiplexers
			adjustMux();

			// Delay for photointerrupters
            // NOTE: maximum delay for delay_us is 8191us.
			delay_us(sampPer);

			// Sample current pixel
			AD1CON1bits.SAMP = 1;
			while (!AD1CON1bits.DONE);
			sample = ADC1BUF0;
			AD1CON1bits.DONE = 0;

			sampled_frame[row][col] = sample;

		}
	}

	// Return frame
	for (row = 0; row < ROWS; ++row)
	{
		for (col = 0; col < COLUMNS; ++col)
		{
			
                        /*while (U1STAbits.UTXBF);
                        U1TXREG = (unsigned char) row;
                        delay_us(50);
                        while (U1STAbits.UTXBF);
                        U1TXREG = (unsigned char) col;
                        delay_us(50);*/
                        while (U1STAbits.UTXBF);
			U1TXREG = (unsigned char)(sampled_frame[row][col] % 256);
			//delay_us(50);
                        while (U1STAbits.UTXBF);
			U1TXREG = (unsigned char)(sampled_frame[row][col] >> 8);
                        //delay_us(50);
		}
	}
        
	row = 0; col = 0;
	LEDRED = 0;
}


void pollPixel(unsigned char rowCoord, unsigned char colCoord, unsigned char duration, unsigned char period)
{
	unsigned int totalSamples;
	if ((1000*duration)%period == 0)
		totalSamples = (1000*duration)/period;
	else
		totalSamples = (1000*duration)/period + 1;

	unsigned int sample, samples;

	LEDRED = 1;
        echoCommand();
        
        while (U1STAbits.UTXBF);
	U1TXREG = (unsigned char)(totalSamples % 256);
	while (U1STAbits.UTXBF);
	U1TXREG = (unsigned char)(totalSamples >> 8);
        
        // Set multiplexers
	// NOTE: Don't update mux for every sample, since sampling same pixel
	row = rowCoord;
	col = colCoord;
	adjustMux();
	delay_us(500);

        while (U1STAbits.UTXBF);
	U1TXREG = row;
        while (U1STAbits.UTXBF);
	U1TXREG = col;

	for (samples = 0; samples < totalSamples; ++samples)
	{
		// Sample requested pixel
		AD1CON1bits.SAMP = 1;
		while (!AD1CON1bits.DONE);
		sample = ADC1BUF0;
		AD1CON1bits.DONE = 0;

		// Return result
		while (U1STAbits.UTXBF);
		U1TXREG = (unsigned char)(sample % 256);
		while (U1STAbits.UTXBF);
		U1TXREG = (unsigned char)(sample >> 8);

		delay_ms(period);
	}

	row = 0; col = 0;
	LEDRED = 0;

}


void startScan(void)
{
	// Reset current row, column indices
	row = 0;
	col = 0;
	frameComplete = 0;

	// Adjust multiplexer inputs
	adjustMux();
	delay_us(500);

	// Clear timer, interrupt flag
	TMR1 = 0;
	_T1IF = 0;

	// Enable timer interrupt, start timer
	_T1IE = 1;
	T1CONbits.TON = 1;

	state = SCANNING;
	LEDRED = 1;
}


void stopScan(void)
{
	// Disable timer interrupt, stop timer
	_T1IE = 0;
	T1CONbits.TON = 0;

	state = IDLE;
	LEDRED = 0;
}

void sendSize(void)
{
    LEDRED = 1;
    echoCommand();
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char) ROWS;
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char) COLUMNS;
    delay_ms(500);
    LEDRED = 0;
}

void echoCommand(void)
{
    while (U1STAbits.UTXBF);
    U1TXREG = rxChar;
}

void sendTestFrame(void)
{
    LEDRED = 1;
    echoCommand();
    int x = 1;
    
    for (row = 0; row < ROWS; ++row)
	{
		for (col = 0; col < COLUMNS; ++col)
		{
			while (U1STAbits.UTXBF);
			U1TXREG = (unsigned char)(x);
                        ++x;
                        while (U1STAbits.UTXBF);
			U1TXREG = (unsigned char)(x);
                        ++x;
		}
	}
    echoCommand();
    
    /*
    int x;
    for (x = 0; x < 30; ++x)
    {
        while (U1STAbits.UTXBF);
        U1TXREG = (unsigned char)(x);
    }
    */
    x = 0;
    row = 0; col = 0;
    LEDRED = 0;
}


void calibrateSkin(void)
{
	unsigned int sample = 0;
	unsigned long pixelAccum = 0;
	unsigned long offsetAccum[ROWS][COLUMNS];
	unsigned char frameSample, i, j;

	// Signal beginning of calibration
	LEDRED = 1;

	// Initialize offset accumulation array
	for (i = 0; i < ROWS; ++i)
		for (j = 0; j < COLUMNS; ++j)
			offsetAccum[i][j] = 0;

	// Accumulate desired number of samples
	for (frameSample = 0; frameSample < FRAME_SAMPLES; ++frameSample)
	{
		for (row = 0; row < ROWS; ++row)
		{
			for (col = 0; col < COLUMNS; ++col)
			{
				// Adjust multiplexers
				adjustMux();
				delay_us(500);

				// Allow pixel to settle
				delay_ms(PIXEL_HOLD_TIME_MS);

				pixelAccum = 0;

				for(i = 0; i < PIXEL_SAMPLES; ++i)
				{
					// Sample current pixel
					AD1CON1bits.SAMP = 1;
					while (!AD1CON1bits.DONE);
					sample = ADC1BUF0;
					AD1CON1bits.DONE = 0;

					// Accumulate sample
					pixelAccum += sample;
				}

				// Average sample
				sample = (unsigned int)(pixelAccum / PIXEL_SAMPLES);

				// Accumulate sample
				offsetAccum[row][col] += sample;
			}
		}
		// Delay between samples
		delay_ms(FRAME_SAMPLE_DELAY_MS);
	}

	// Average samples
	for (i = 0; i < ROWS; ++i)
		for (j = 0; j < COLUMNS; ++j)
			offset[i][j] = (unsigned int)((offsetAccum[i][j]) / FRAME_SAMPLES);

	// Signal end of calibration
	LEDRED = 0;
}


void returnCalibration(void)
{
	LEDRED = 1;
	unsigned char i, j;

	for (i = 0; i < ROWS; ++i)
	{
		for (j = 0; j < COLUMNS; ++j)
		{
			while (U1STAbits.UTXBF);
			U1TXREG = (unsigned char)(offset[i][j] % 256);
			while (U1STAbits.UTXBF);
			U1TXREG = (unsigned char)(offset[i][j] >> 8);
			delay_ms(3);
		}
	}
	LEDRED = 0;
}


void startTimer(void)
{
	// Clear timer, interrupt flag
	TMR1 = 0;
	_T1IF = 0;

	// Enable timer interrupt, start timer
	_T1IE = 1;
	T1CONbits.TON = 1;
}


void stopTimer(void)
{
	// Disable timer interrupt, stop timer
	_T1IE = 0;
	T1CONbits.TON = 0;
}


void pollColumn(unsigned char colCoord, unsigned char duration)
{
	unsigned int samplePeriod = 100;
	unsigned int totalSamples = duration * 10;
	unsigned int samples;
	unsigned char rowCoord;

	for (samples = 0; samples < totalSamples; ++samples)
	{
		// Run through all rows in column
		for (rowCoord = 0; rowCoord < ROWS; ++rowCoord)
		{
			samplePixel(rowCoord, colCoord);
		}
		delay_ms (samplePeriod);
	}

}
