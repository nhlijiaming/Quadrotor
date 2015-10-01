/*
 * PID.h
 *
 *  Created on: 2014Äê7ÔÂ23ÈÕ
 *      Author: Ljm
 */

#ifndef PID_H_
#define PID_H_

#include "board.h"

#define accVelScale 0.0610340322f

#define pos_cf_pos 0.5f
#define pos_cf_vel 0.8f

typedef struct
{
	float   P, I, D;
	float   iTerm;
	float   windupGuard;
	float   lastError;
	float   dTerm1;
	float   dTerm2;
} PID_Data;

typedef struct axispid_data
{
	float Pitch, Roll, Yaw;
}axisPID_Data;

extern PID_Data PID_Pitch, PID_Roll, PID_Yaw, PID_Altitude;
extern PID_Data PID_PosX, PID_PosY;
extern PID_Data PID_Attenuation, PID_AttenuationReal;

extern axisPID_Data axisPID;

extern float Velz;
extern float accAltz;

float PID_Update(PID_Data *pid, const float currentValue, const float setValue, const float more);

int32_t PID_AltitudeUpdate(PID_Data *pid, float AltHold, float dTime);

void PID_AltitudeUpdateSonarFusion(float sonarAlt, float dTime);

float PID_PositionXUpdate(PID_Data *pid, float SetPosX);

float PID_PositionYUpdate(PID_Data *pid, float SetPosY);

//float PID_AltitudeUpdate(PID_Data *pid, const float currentValue, const float setValue);

void PID_CleariTerm();

void PID_Clear();

void PID_AltitudeClear();

#endif /* PID_H_ */
