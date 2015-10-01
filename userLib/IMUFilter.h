/*
 * IMUFilter.h
 *
 *  Created on: 2014Äê8ÔÂ10ÈÕ
 *      Author: Ljm
 */

#ifndef IMUFILTER_H_
#define IMUFILTER_H_

#include "board.h"

#define acc_lpf_factor 		(4.0f)
#define att_lpf_factor		(4.0f)
#define acc_lpf_cutoff 		(5.0f)
#define att_lpf_cutoff		(5.0f)
#define M_PI 				(3.1415926f)
#define fc_acc   			(1.0f / (2 * M_PI * acc_lpf_cutoff))
#define fc_att				(1.0f / (2 * M_PI * att_lpf_cutoff))

typedef struct
{
	uint32_t  	AccTimeSum;
	uint16_t  	AccSumCount;
	float     	AccSum;
}ACC_FILTER;

typedef struct
{
	uint32_t 	AttTimeSum;
	uint16_t 	AttSumCount;
	float 	 	PitchSum;
	float 		RollSum;
}ATT_FILTER;

extern ACC_FILTER AccXFilter, AccYFilter, AccZFilter;
extern ATT_FILTER AttFilter, AttFilter2;

void AccZ_Filter(uint32_t deltaT);

void Att_Filter();

#endif /* IMUFILTER_H_ */
