/*
 * PPM.h
 *
 *  Created on: 2014Äê7ÔÂ20ÈÕ
 *      Author: Ljm
 */

#ifndef PPM_Encoder_H_
#define PPM_Encoder_H_

#include "board.h"

#define Freq 400

void PPMEncoder_Init();

void PPMEncoder_Update(int m1, int m2, int m3, int m4);

#endif /* PPM_H_ */
