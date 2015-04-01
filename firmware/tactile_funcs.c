#include "tactile_funcs.h"
#include "utils.h"
#include "sclock.h"
#include "setup.h"

static unsigned char row, col;
static unsigned int frame[ROWS][COLUMNS];
static unsigned char clear2send = 0;
static unsigned char frameComplete = 0;
static unsigned long telemStartTime = 0; //Offset for time value when recording samples
static unsigned char mode = 0;
volatile int select_ind = 0;
int select_list[8*2] = {0,0,1,0,2,0,3,0,4,1,5,1,6,1,7,1};

void setMode(unsigned char m) {
    mode = m;
}

void samplePixel(unsigned char rowCoord, unsigned char colCoord)
{
        LEDRED = 1;
	unsigned int sample = 0;

        echoChar(mode);
        sendPayloadLength(4);

	// Set multiplexers
	row = rowCoord;
	col = colCoord;
	adjustMux();

	// DELAY.. need to experiment with this
	delay_us(100);

	// Sample requested pixel
        sample = sampleADC();

	// Return result
        echoChar(row);
        echoChar(col);
        sendSample(sample);

        LEDRED = 0;
}

void sampleFrame(unsigned int sampPer)
{
	unsigned int sample;
	unsigned int sampled_frame[ROWS][COLUMNS];

	LEDRED = 1;
        echoChar(mode);
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

                        sample = sampleADC();

			sampled_frame[row][col] = sample;

		}
	}

	// Return frame

        if (1) {// (SELECT_CELLS > 0) {
            sendPayloadLength(ROWS*COLUMNS+4);
            int i,j,k;
            for (k = 0; k < 8; ++k) {
                i = select_list[k*2];
                j = select_list[k*2+1];
                sample =  sampled_frame[i][j];
                sendSample(sample);
            }
        } else {
            sendPayloadLength(ROWS*COLUMNS*2+4);
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
                            sendSample(sampled_frame[row][col]);
                    }
            }
        }
        //send current time
        unsigned long time = sclockGetTime() - telemStartTime;
        while (U1STAbits.UTXBF);
        U1TXREG = (unsigned char)(time % 256);
        while (U1STAbits.UTXBF);
        U1TXREG = (unsigned char)((time >> 8) % 256);
        while (U1STAbits.UTXBF);
        U1TXREG = (unsigned char)((time >> 16) % 256);
        while (U1STAbits.UTXBF);
        U1TXREG = (unsigned char)((time >> 24) % 256);
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
        echoChar(mode);

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
                sample = sampleADC();

		// Return result
		sendSample(sample);

		delay_ms(period);
	}

	row = 0; col = 0;
	LEDRED = 0;

}

void sendSize()
{
    LEDRED = 1;
    echoChar(mode);
    sendPayloadLength(2);
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char) ROWS;
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char) COLUMNS;
    delay_ms(500);
    LEDRED = 0;
}

void echoChar(unsigned char command)
{
    while (U1STAbits.UTXBF);
    U1TXREG = command;
}

void sendPayloadLength(unsigned int len)
{
    while (len >= 255) {
        while (U1STAbits.UTXBF);
        U1TXREG = (char) 255;
        len = len - 255;
    }
    while (U1STAbits.UTXBF);
    U1TXREG = (char) len;
}

void sendTestFrame(void)
{
    LEDRED = 1;
    echoChar(mode);
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
    echoChar(mode);

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

void sendFullFrame() {
    int i,j,k;
    unsigned int sample;
    echoChar(TACTILE_MODE_B);
    if (1) {// (SELECT_CELLS > 0) {
        sendPayloadLength(ROWS*COLUMNS+4);
        for (k = 0; k < 8; ++k) {
            i = select_list[k*2];
            j = select_list[k*2+1];
            sample =  frame[i][j];
            sendSample(sample);
        }
    } else {
        sendPayloadLength(ROWS*COLUMNS*2+4);

        for (i = 0; i < ROWS; ++i) {
            for (j = 0; j < COLUMNS; ++j) {
                sample =  frame[i][j];
                sendSample(sample);
            }
        }
    }
    //send current time
    unsigned long time = sclockGetTime() - telemStartTime;
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char)(time % 256);
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char)((time >> 8) % 256);
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char)((time >> 16) % 256);
    while (U1STAbits.UTXBF);
    U1TXREG = (unsigned char)((time >> 24) % 256);


    clearCTS();
    frameComplete = 0;
    startTimer();
}

unsigned char fullFrame() {
    return frameComplete;
}

void setRC(unsigned char r, unsigned char c) {
    row = r;
    col = c;
}

unsigned int sampleADC() {
    // Sample requested pixel
    unsigned int sample;
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE);
    sample = ADC1BUF0;
    AD1CON1bits.DONE = 0;
    return sample;
}

void sendSample(unsigned int sample) {
    //break down sample into bytes
    echoChar((unsigned char)(sample % 256));
    echoChar((unsigned char)(sample >> 8));
}


unsigned char nextUARTByte() {
    while (!U1STAbits.URXDA);
    unsigned char rxChar = U1RXREG;
    return rxChar;
}

unsigned char checkforUARTByte() {
    if(U1STAbits.URXDA) {
        return 1;
    }
    return 0;

}

// -------------------
// Multiplex Functions
// -------------------


void advancePixel(void)
{
    if (SELECT_CELLS > 0) {
        select_ind += 2;

        if (select_ind > SELECT_CELLS*2 - 1) {
            select_ind = 0;
            frameComplete = 1;
            stopTimer();
        }
        row = (char) select_list[select_ind];
        col = (char) select_list[select_ind+1];
    } else {
	// Advance to next pixel
	if (col == COLUMNS - 1)
	{
		col = 0;
		if (row == ROWS - 1)
		{
			row = 0;
			// Raise flag to indicate completion of frame
			frameComplete = 1;
                        stopTimer();
		}
		else
			row++;
	}
	else
		col++;
    }
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





void startScan(void) {
	// Reset current row, column indices
        if (SELECT_CELLS > 0) {
            select_ind = 0;
            row = (char) select_list[select_ind];
            col = (char) select_list[select_ind+1];
        } else {
            row = 0;
            col = 0;
        }
	frameComplete = 0;

	// Adjust multiplexer inputs
	adjustMux();
	delay_us(500);

	
	LEDRED = 1;
        telemStartTime = sclockGetTime();
        startTimer();
}

void stopScan() {
        stopTimer();
	LEDRED = 0;
}




void startTimer(void)
{
        setupTimer(TWO_K_TMRPRESCALE, TWO_K_TMRVAL % 256, TWO_K_TMRVAL >> 8);
	//setupTimer(DATA_TMRPRESCALE, DATA_TMRVAL % 256, DATA_TMRVAL >> 8);
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

void clearCTS() {
    clear2send = 0;
}
void setCTS() {
    clear2send = 1;
}
unsigned char checkCTS() {
    return clear2send;
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
	// Sample current pixel
	frame[row][col] = sampleADC();

	// Move onto next pixel (no delay -- handled by interrupt period)
	advancePixel();
}