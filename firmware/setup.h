/* 
 * File:   setup.h
 * Author: jgoldberg
 *
 * Created on January 16, 2015, 2:19 PM
 */

#ifndef SETUP_H
#define	SETUP_H

void setupADC(void);
void setupTimer(unsigned char tmrPreScale, unsigned char tmrPerL, unsigned char tmrPerH);
void setupUART(void);
void setupSkin(void);

#endif	/* SETUP_H */