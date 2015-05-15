/* 
 * File:   setup.h
 * Author: jgoldberg
 *
 * Created on January 16, 2015, 2:19 PM
 */


#ifndef SETUP_H
#define	SETUP_H


// --------------------
// PREPROCESSOR DEFINES
// --------------------
#define LEDRED			_LATB12
#define LEDGREEN		_LATB13
#define LEDYELLOW		_LATB14
#define IGNORE_CELLS            0

#define TACTILE_MODE_A          0x41
#define TACTILE_MODE_B          0x42
#define TACTILE_MODE_C          0x43
#define TACTILE_MODE_D          0x44
#define TACTILE_MODE_E          0x45
#define TACTILE_MODE_F          0x46
#define TACTILE_MODE_G          0x47
#define TACTILE_MODE_T          0x54
#define CTS                     0x5A //'Z'


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

//8x2 grid for skinproc v0.1b
#elif defined(__8x2Shell)

	// Sensor Dimensions
	#define ROWS 			8
	#define COLUMNS 		2
	#define FIRSTROW		0
	#define FIRSTCOL		0

	// UART Configuration
	#define U1BRGVAL		9								// For 1 Mbps (HIGH SPEED)
	#define U1BRGHVAL		1								// High Speed Mode

        // Data Timer Configuration
	#define DATA_TMRVAL			49999					// For 100 Hz interrupt
	#define DATA_TMRPRESCALE	0b01                                            // Prescale = 1:8

        // 2k Hz Timer Configuration
	#define TWO_K_TMRVAL		19999					// For 2k Hz interrupt
	#define TWO_K_TMRPRESCALE	0b00					// Prescale = 1:1

        // 10k Hz Timer Configuration
	#define TEN_K_TMRVAL		3999					// For 10k Hz interrupt
	#define TEN_K_TMRPRESCALE	0b00					// Prescale = 1:1

        // ignore certain cells
        #define IGNORE_CELLS            8                                                 //how many cells are to be ignored
        extern const char select_list[(ROWS*COLUMNS-IGNORE_CELLS)*2];                //cells that are populated
                                                                                          //organized into (row,column),(row,column),...

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

// SkinProc States
#define IDLE				0
#define SCANNING			1

void setupADC(void);
void setupTimer(unsigned char tmrPreScale, unsigned char tmrPerL, unsigned char tmrPerH);
void setupUART(void);
void setupSkin(void);

#endif	/* SETUP_H */