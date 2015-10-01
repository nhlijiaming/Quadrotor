/*
 * IMU.h
 *
 *  Created on: 2014Äê7ÔÂ22ÈÕ
 *      Author: Ljm
 */

#ifndef IMU_H_
#define IMU_H_

#include "board.h"

void IMU_Init();

bool IMU_Update(float ax,float ay,float az,float gx,float gy,float gz);

#endif /* IMU_H_ */
