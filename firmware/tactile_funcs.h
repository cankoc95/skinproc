/* 
 * File:   tactile_funcs.h
 * Author: jgoldberg
 *
 * Created on March 31, 2015, 11:53 AM
 */

#ifndef TACTILE_FUNCS_H
#define	TACTILE_FUNCS_H


void samplePixel(unsigned char, unsigned char);
void sampleFrame(unsigned int);
void pollPixel(unsigned char, unsigned char, unsigned char, unsigned char);

void advancePixel(void);
void adjustMux(void);
void setRC(unsigned char r, unsigned char c);

void startScan();
void stopScan();

void startTimer(void);
void stopTimer(void);

void setMode(unsigned char m);
unsigned char fullFrame();
void clearCTS();
void setCTS();
unsigned char checkCTS();

void setStartTime();
void sendCurrentTime();
void sendTestFrame();
void sendFullFrame();
void sendPayloadLength(unsigned int);
void sendSize();
void sendSample(unsigned int sample);

void echoChar(unsigned char c);
unsigned char nextUARTByte();
unsigned char checkforUARTByte();

unsigned int sampleADC();

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void handleT1Interrupt(void);

#endif	/* TACTILE_FUNCS_H */

