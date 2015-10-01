/*
 * PPM_Receiver.h
 *
 *  Created on: 2014Äê7ÔÂ21ÈÕ
 *      Author: Ljm
 */

#ifndef PPM_RECEIVER_H_
#define PPM_RECEIVER_H_

#include "board.h"

typedef struct rxdata
{
	int32_t Throttle, Pitch, Roll, Yaw;
	bool SW1, SW2;
}rxData;

extern volatile int rxChannel_RAW[];
extern rxData rxCommand;

void Timer2AIntHandler(void);
void Timer2BIntHandler(void);
void Timer3AIntHandler(void);
void Timer3BIntHandler(void);

bool PPMReveiver_Transform();

void PPMReceiver_Init();

#endif /* PPM_RECEIVER_H_ */
