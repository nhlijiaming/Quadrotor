/*
 * Sonar.h
 *
 *  Created on: 2014Äê7ÔÂ31ÈÕ
 *      Author: Ljm
 */

#ifndef SONAR_H_
#define SONAR_H_

#include "board.h"

void Sonar_Init();

void Sonar_Trig();

float Sonar_LimitationFilter(float currentAlt);

float Sonar_MedianFilter(float currentAlt);

//uint32_t Sonar_KalmanFilter(uint32_t Measure, bool available);

#endif /* SONAR_H_ */
