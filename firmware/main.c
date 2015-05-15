/* 
 * File:   main.c
 * Author: jgoldberg
 *
 * Created on February 11, 2014, 11:37 AM
 */

//#include <stdio.h>
//#include <stdlib.h>

// ---------
// LIBRARIES
// ---------
#include <xc.h>
#include "utils.h"
#include "init_default.h"
#include "timer.h"
#include "sclock.h"
//#include "stopwatch.h"
#include "setup.h"
#include "tactile_funcs.h"




// -------
// GLOBALS
// -------

unsigned char state = IDLE;
unsigned char rxChar = 0;

int main() {
    unsigned char rxRow, rxCol;
    unsigned char sampPerL, sampPerH;
    unsigned char tmrPreScale, tmrPeriodL, tmrPeriodH;
    unsigned char rxDur, rxPer;
    unsigned char i;
    
    // SETUP COMMANDS

    // Setup hardware
    SetupClock();
    SwitchClocks();
    sclockSetup();
    setupADC();
    setupSkin();
    delay_ms(100);
    setupUART();

    // Initialize current row, column indices
    setRC(15,15); //initialize to empty pixel to save power

    // Adjust multiplexer inputs
    adjustMux();
    delay_us(500);

    // Initialize state to IDLE
    state = IDLE;

    // Toggle LEDs to signal completion
    for (i = 0; i < 5; ++i) {
        LEDYELLOW = 0; LEDRED = 1;
        delay_ms(50);

        LEDRED = 0; LEDGREEN = 1;
        delay_ms(50);

        LEDGREEN = 0; LEDYELLOW = 1;
        delay_ms(50);
    }
    LEDYELLOW = 0;

    // END SETUP COMMANDS

    /*
    Listen for commands:
    'A' -- Sample Pixel
    'B' -- Sample Frame
    'C' -- Poll Pixel
    'D' -- Set Scan Rate
    'E' -- Start Scan
    'F' -- Stop Scan
    */

    while (1) {
        // ----
        // IDLE
        // ----
        if (state == IDLE) {
            rxChar = nextUARTByte();
            LEDYELLOW = ~LEDYELLOW;
            setMode(rxChar);
            switch (rxChar) {
                case TACTILE_MODE_A:	// Sample Pixel
                    // Wait for row, column coordinates
                    rxRow = nextUARTByte();
                    rxCol = nextUARTByte();
                    clearCTS();
                    samplePixel(rxRow, rxCol);
                    break;
                case TACTILE_MODE_B:	// Sample Frame
                    sampPerL = nextUARTByte();
                    sampPerH = nextUARTByte();
                    clearCTS();
                    sampleFrame( (unsigned int)(sampPerL) + ((unsigned int)(sampPerH) << 8) );
                    break;
                case TACTILE_MODE_C:	// Poll Pixel
                    // Wait for row, column, duration, sample period
                    rxRow = nextUARTByte();
                    rxCol = nextUARTByte();
                    rxDur = nextUARTByte();
                    rxPer = nextUARTByte();
                    clearCTS();
                    pollPixel(rxRow, rxCol, rxDur, rxPer);
                    break;
                case TACTILE_MODE_D:	// Set Scan Rate
                    // Wait for pre-scale factor, period
                    tmrPreScale = nextUARTByte();
                    tmrPeriodL = nextUARTByte();
                    tmrPeriodH = nextUARTByte();
                    setupTimer(tmrPreScale, tmrPeriodL, tmrPeriodH);
                    break;
                case TACTILE_MODE_E:
                    startScan();
                    state = SCANNING;
                    setCTS();
                    break;
                case TACTILE_MODE_F:
                    stopScan();
                    state = IDLE;
                    break;
                case TACTILE_MODE_G:
                    sendSize();
                    break;
                case TACTILE_MODE_T:
                    sendTestFrame();
                    break;
                case CTS:
                    setCTS(); //clear to send
                    break;
                default:
                    break;
            }
        }

        // --------
        // SCANNING
        // --------
        if (state == SCANNING) {
            // Wait for complete frame
            if (fullFrame() && checkCTS()) {
                clearCTS();
                sendFullFrame();
                startTimer();
            }

            // ------------
            // STOP COMMAND
            // ------------
            if(checkforUARTByte()) {
                rxChar = nextUARTByte();
                if (rxChar == TACTILE_MODE_F) {
                    stopScan();
                    state = IDLE;
                } else if (rxChar == CTS) {
                    setCTS();
                }
            } // end if stop command
        } // end if scanning
    } // End while(1)
} // End main()