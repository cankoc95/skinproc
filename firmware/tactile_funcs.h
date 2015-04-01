/* 
 * File:   tactile_funcs.h
 * Author: jgoldberg
 *
 * Created on March 31, 2015, 11:53 AM
 */

#ifndef TACTILE_FUNCS_H
#define	TACTILE_FUNCS_H

void setMode(unsigned char m);

void samplePixel(unsigned char, unsigned char);
void sampleFrame(unsigned int);
void pollPixel(unsigned char, unsigned char, unsigned char, unsigned char);
void sendSize();
void echoChar(unsigned char);
void sendPayloadLength(unsigned int);
void sendTestFrame();
void sendFullFrame();
unsigned char fullFrame();

unsigned int sampleADC();
void sendSample(unsigned int sample);
unsigned char nextUARTByte();
unsigned char checkforUARTByte();

void advancePixel(void);
void adjustMux(void);
void setRC(unsigned char r, unsigned char c);

void startScan();
void stopScan();

void startTimer(void);
void stopTimer(void);

void clearCTS();
void setCTS();
unsigned char checkCTS();

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void handleT1Interrupt(void);

#endif	/* TACTILE_FUNCS_H */

