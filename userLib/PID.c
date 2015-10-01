/*
 * PID.c
 *
 *  Created on: 2014Äê7ÔÂ23ÈÕ
 *      Author: Ljm
 */

#include "PID.h"

PID_Data PID_Pitch, PID_Roll, PID_Yaw, PID_Altitude;
PID_Data PID_PosX, PID_PosY;
PID_Data PID_Attenuation, PID_AttenuationReal;
axisPID_Data axisPID;

float Velz = 0.0f;
float accAltz = 0.0f;
float sonarVel = 0.0f;
float sonarAlt = 0.0f;

float VelX = 0.0;
float accPosX = 0.0;
float VelY = 0.0;
float accPosY = 0.0;

float PID_Update(PID_Data *pid, const float currentValue, const float setValue, const float more)
{
#if 0
	float pError;
	float out;
	float gyro = more;

	pError = setValue - currentValue;

	pid->iTerm += pError;

	// windup guard
	if ( pid->iTerm > pid->windupGuard ) pid->iTerm = pid->windupGuard;
	if ( pid->iTerm < -pid->windupGuard ) pid->iTerm = -pid->windupGuard;

	out = (pid->P * pError) + (pid->I * pid->iTerm) + (pid->D * gyro);
	pid->lastError = pError;

	return out;
#else

#define fCut 20
#define PI 3.141592653f
#define RC (1.0f / ( 2.0f * PI * fCut))
	float pError;
//	float dTerm, dSum;
	float out;

	static uint64_t pTime = 0xFFFFFFFFFFFFFFFF;
	uint64_t cTime     	= ClockCounter_Get();
	uint64_t dTime		= ClockCounter_tomicro(pTime - cTime)*0.000001;
	pTime = cTime;

	pError = setValue - currentValue;

	pid->iTerm += pid->I *pError;

	// integrator windup guard
	if ( pid->iTerm > pid->windupGuard ) pid->iTerm = pid->windupGuard;
	if ( pid->iTerm < -pid->windupGuard ) pid->iTerm = -pid->windupGuard;

	// Discrete low pass filter, cuts out the
    // high frequency noise that can drive controller crazy
	pid->dTerm1 = more;
	pid->dTerm1 = pid->dTerm1 + (dTime / (RC + dTime)) * (pid->dTerm1 - pid->dTerm2);
	pid->dTerm2 = pid->dTerm1;

	out = (pid->P  * pError) + (pid->iTerm) + (pid->D  *pid->dTerm1);
	pid->lastError = pError;

	return out;
#endif
}

int32_t PID_AltitudeUpdate(PID_Data *pid, float AltHold, float dTime)
{
    float error;
    float setVel;
    float sonarPID;
    float dt;
    float accZ_tmp;
    float vel_acc;

    dt = AccZFilter.AccTimeSum * 1e-6f; // delta acc reading time in seconds

    if ( AccZFilter.AccSumCount == 0 ) return 0.0;
    // Integrator - velocity, cm/sec
    accZ_tmp = (float)AccZFilter.AccSum / (float)AccZFilter.AccSumCount;
	//accZ = (int32_t) accZ_tmp;
    vel_acc = accZ_tmp * accVelScale * dt;
    Velz += vel_acc;

    // Integrator - Altitude in cm
    accAltz += (vel_acc * 0.5f) * dt + Velz * dt;                        // integrate velocity to get distance (x= a/2 * t^2)

#if (debug)
//    Debugger_SendSeriesData(1,Velz);
//    Debugger_SendSeriesData(2,accAltz);
//    Debugger_SendSeriesData(3,sonarAlt);
//    Debugger_SendSeriesData(4,sonarVel);
//    Debugger_SendSeriesData(5,AltHold);
//    Debugger_SendSeriesData(6,Altitude_RAW);
#endif

	AccZFilter.AccSum = 0.0;
	AccZFilter.AccSumCount = 0;
	AccZFilter.AccTimeSum = 0.0;

	AttFilter.PitchSum = 0.0;
	AttFilter.RollSum = 0.0;
	AttFilter.AttSumCount = 0;

    error = constrainf(AltHold - accAltz, -300, 300);
    error = applyDeadbandf(error, 10);       	// remove small P parametr to reduce noise near zero position
    setVel = constrainf(5 * error, -100, +100); // limit velocity to +/- 100 cm/s

    // Velocity PID-Controller
    // P
    error = setVel - Velz;
    sonarPID = PID_Altitude.P * error;

    // I
    PID_Altitude.iTerm += PID_Altitude.I * error;
    PID_Altitude.iTerm = constrainf(PID_Altitude.iTerm, -PID_Altitude.windupGuard, PID_Altitude.windupGuard);
    sonarPID += PID_Altitude.iTerm;     // I in the range of +/-200

    // D
    sonarPID -= constrainf( PID_Altitude.D * (accZ_tmp + PID_Altitude.dTerm1) , -150, 150);
    PID_Altitude.dTerm1 = accZ_tmp;

    return sonarPID;
}

void PID_AltitudeUpdateSonarFusion(float alt, float dTime)
{
//#define cf_alt 0.965f
//#define cf_vel 0.985f
//period 60ms
#define sonar_cf_alt 0.5f
#define sonar_cf_vel 0.7f
//period 40ms
//#define cf_alt 0.3f
//#define cf_vel 0.5f
//period 20ms
//#define cf_alt 0.81f
//#define cf_vel 0.97f

#define acc_sonar_cutoff	(10.0f)
#define M_PI 				(3.1415926f)
#define fc_sonar   		(1.0f / (2 * M_PI * acc_sonar_cutoff))

//	float dT;
	float pitch;
	float roll;
    static float lastsonarAlt;
	//    int16_t tiltAngle = 10*max(abs(Att.Roll), abs(Att.Pitch));

    sonarAlt = alt;
//	dT = dTime;
//	sonarAlt = sonarAlt + (dT / (fc_sonar + dT)) * (alt - sonarAlt); // low pass filter

    if ( AttFilter.AttSumCount == 0 ) AttFilter.AttSumCount = 1;
    //    sonarAlt = sonarAlt * (900.0f - tiltAngle) / 900.0f;
    pitch = (float)AttFilter.PitchSum / (float)AttFilter.AttSumCount;
    roll = (float)AttFilter.RollSum / (float)AttFilter.AttSumCount;
    sonarAlt = sonarAlt * cosf(pitch*3.1415926/180) * cosf(roll*3.1415926/180);
    //    sonarVel = LevantDifferentiator(sonarAlt);		// unit: cm/s
    sonarVel = applyDeadbandf(sonarAlt - lastsonarAlt, 0.3) / dTime;
    lastsonarAlt = sonarAlt;

    sonarVel = constrainf(sonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    sonarVel = applyDeadbandf(sonarVel, 7);         // to reduce noise near zero

    accAltz = accAltz * sonar_cf_alt + sonarAlt * (1.0f - sonar_cf_alt);      // complementary filter for altitude estimation (baro & acc)
    Velz   =    Velz * sonar_cf_vel + sonarVel * (1.0f - sonar_cf_vel);

    // set vario
    sonarVario = applyDeadband(sonarVel, 5);
}

float PID_PositionXUpdate(PID_Data *pid, float SetPosX)
{
#if (0)
    float error;
    float PosXPID;
    float PosX;

    PosX = OpticalFlow.compensatedx;

    error = constrainf(SetPosX - PosX, -30, 30);	// 100cm
    error = applyDeadbandf(error, 3);      	 		// remove small P parametr to reduce noise near zero position

    PosXPID = PID_PosX.P * error;

    // I
    PID_PosX.iTerm += PID_PosX.I * error;
    PID_PosX.iTerm = constrainf(PID_PosX.iTerm, -PID_PosX.windupGuard, PID_PosX.windupGuard);
    PosXPID += PID_PosX.iTerm;     // I in the range of +/-200

    // D
    PosXPID += constrainf( PID_Altitude.D * (error - PID_PosX.dTerm1)*500 , -1.0, 1.0);
    PID_PosX.dTerm1 = error;

#if (debug)
    Debugger_SendSeriesData(7,PosX);
#endif

    return PosXPID;
#else
    float error;
    float setVel;
    float dt;
    float accX_tmp;
    float vel_acc;
    float PosXPID;
    float opticalflowPosX;
    float opticalflowVelX;

    dt = AccXFilter.AccTimeSum * 1e-6f; // delta acc reading time in seconds

    if ( AccXFilter.AccSumCount == 0 ) return 0.0;
    // Integrator - velocity, cm/sec
    accX_tmp = (float)AccXFilter.AccSum / (float)AccXFilter.AccSumCount;
    vel_acc = accX_tmp * accVelScale * dt;
    VelX += vel_acc;

    // Integrator - Altitude in cm
    accPosX += (vel_acc * 0.5f) * dt + VelX * dt;                        // integrate velocity to get distance (x= a/2 * t^2)

	opticalflowPosX = OpticalFlow.compensatedx;

	opticalflowVelX = OpticalFlow.x_cm / dt;
	opticalflowVelX = constrainf(opticalflowVelX, -300, 300);    // constrain baro velocity +/- 300cm/s
	opticalflowVelX = applyDeadbandf(opticalflowVelX, 8);         // to reduce noise near zero

	accPosX = accPosX * pos_cf_pos + opticalflowPosX * (1.0f - pos_cf_pos);      // complementary filter for altitude estimation (baro & acc)
	VelX   =    VelX * pos_cf_vel + opticalflowVelX * (1.0f - pos_cf_vel);

#if (debug)
//    Debugger_SendSeriesData(1,opticalflowVelX);
//    Debugger_SendSeriesData(2,opticalflowPosX);
//    Debugger_SendSeriesData(3,VelX);
//    Debugger_SendSeriesData(4,accPosX);
#endif

	AccXFilter.AccSum = 0.0;
	AccXFilter.AccSumCount = 0;
	AccXFilter.AccTimeSum = 0.0;

    error = constrainf(SetPosX - accPosX, -300, 300);
    error = applyDeadbandf(error, 5);       	// remove small P parametr to reduce noise near zero position
    setVel = constrainf(5 * error, -10, +10); // limit velocity to +/- 100 cm/s

    // Velocity PID-Controller
    // P
    error = setVel - VelX;
    error = applyDeadbandf(error, 2);      	 		// remove small P parametr to reduce noise near zero position

    PosXPID = PID_PosX.P * error;

    // I
    PID_PosX.iTerm += PID_PosX.I * error;
    PID_PosX.iTerm = constrainf(PID_PosX.iTerm, -PID_PosX.windupGuard, PID_PosX.windupGuard);
    PosXPID += PID_PosX.iTerm;     // I in the range of +/-200

    // D
    PosXPID -= constrainf( PID_Altitude.D * (accX_tmp + PID_Altitude.dTerm1) , -1.5, 1.5);
    PID_PosX.dTerm1 = accX_tmp;

#if (debug)
    Debugger_SendSeriesData(7,opticalflowPosX);
#endif

    return PosXPID;
#endif
}

float PID_PositionYUpdate(PID_Data *pid, float SetPosY)
{
#if (0)
    float error;
    float PosYPID;
    float PosY;

    PosY = OpticalFlow.compensatedy;

    error = constrainf(SetPosY - PosY, -50, 50);	// 100cm
    error = applyDeadbandf(error, 5);      	 		// remove small P parametr to reduce noise near zero position

    PosYPID = PID_PosY.P * error;

    // I
    PID_PosY.iTerm += PID_PosY.I * error;
    PID_PosY.iTerm = constrainf(PID_PosY.iTerm, -PID_PosY.windupGuard, PID_PosY.windupGuard);
    PosYPID += PID_PosY.iTerm;     // I in the range of +/-200

    // D
    PosYPID += constrainf( PID_Altitude.D * (error - PID_PosY.dTerm1)*500 , -1.0, 1.0);
    PID_PosY.dTerm1 = error;

#if (debug)
    Debugger_SendSeriesData(8,PosY);
#endif

    return PosYPID;
#else
    float error;
    float setVel;
    float dt;
    float accY_tmp;
    float vel_acc;
    float PosYPID;
    float opticalflowPosY;
    float opticalflowVelY;

    dt = AccYFilter.AccTimeSum * 1e-6f; // delta acc reading time in seconds

    if ( AccYFilter.AccSumCount == 0 ) return 0.0;
    // Integrator - velocity, cm/sec
    accY_tmp = (float)AccYFilter.AccSum / (float)AccYFilter.AccSumCount;
    vel_acc = accY_tmp * accVelScale * dt;
    VelY += vel_acc;

    // Integrator - Altitude in cm
    accPosY += (vel_acc * 0.5f) * dt + VelY * dt;                        // integrate velocity to get distance (Y= a/2 * t^2)

	opticalflowPosY = OpticalFlow.compensatedy;

	opticalflowVelY = OpticalFlow.y_cm / dt;
	opticalflowVelY = constrainf(opticalflowVelY, -300, 300);    // constrain baro velocity +/- 300cm/s
	opticalflowVelY = applyDeadbandf(opticalflowVelY, 8);         // to reduce noise near zero

	accPosY = accPosY * pos_cf_pos + opticalflowPosY * (1.0f - pos_cf_pos);      // complementary filter for altitude estimation (baro & acc)
	VelY   =    VelY * pos_cf_vel + opticalflowVelY * (1.0f - pos_cf_vel);

	AccYFilter.AccSum = 0.0;
	AccYFilter.AccSumCount = 0;
	AccYFilter.AccTimeSum = 0.0;

    error = constrainf(SetPosY - accPosY, -300, 300);
    error = applyDeadbandf(error, 5);       	// remove small P parametr to reduce noise near zero position
    setVel = constrainf(5 * error, -10, +10); // limit velocity to +/- 100 cm/s


#if (debug)
    Debugger_SendSeriesData(1,VelY);
    Debugger_SendSeriesData(2,accPosY);
    Debugger_SendSeriesData(3,opticalflowVelY);
    Debugger_SendSeriesData(4,opticalflowPosY);
    Debugger_SendSeriesData(5,error);
    Debugger_SendSeriesData(6,setVel);

#endif

    // Velocity PID-Controller
    // P
    error = setVel - VelY;
    error = applyDeadbandf(error, 2);      	 		// remove small P parametr to reduce noise near zero position

    PosYPID = PID_PosY.P * error;

    // I
    PID_PosY.iTerm += PID_PosY.I * error;
    PID_PosY.iTerm = constrainf(PID_PosY.iTerm, -PID_PosY.windupGuard, PID_PosY.windupGuard);
    PosYPID += PID_PosY.iTerm;     // I in the range of +/-200

    // D
    PosYPID += constrainf( PID_Altitude.D * (accY_tmp + PID_Altitude.dTerm1)/100 , -1.5, 1.5);
    PID_PosY.dTerm1 = accY_tmp;

#if (debug)
    Debugger_SendSeriesData(8,PosYPID);
#endif

    return PosYPID;
#endif
}

void PID_CleariTerm()
{
	PID_Pitch.iTerm = 0;
	PID_Roll.iTerm = 0;
	PID_Yaw.iTerm = 0;
}

void PID_Clear()
{
	PID_Pitch.lastError = 0;
	PID_Roll.lastError = 0;
	PID_Yaw.lastError = 0;
	PID_Altitude.lastError = 0;
	PID_PosX.lastError = 0;
	PID_PosY.lastError = 0;

	PID_Pitch.iTerm = 0;
	PID_Roll.iTerm = 0;
	PID_Yaw.iTerm = 0;
	PID_Altitude.iTerm = 0;
	PID_PosX.iTerm = 0;
	PID_PosY.iTerm = 0;

	PID_Pitch.dTerm1 = 0;
	PID_Roll.dTerm1 = 0;
	PID_Yaw.dTerm1 = 0;
	PID_Altitude.dTerm1 = 0;
	PID_PosX.dTerm1 = 0;
	PID_PosY.dTerm1 = 0;

	PID_Pitch.dTerm2 = 0;
	PID_Roll.dTerm2 = 0;
	PID_Yaw.dTerm2 = 0;
	PID_Altitude.dTerm2 = 0;
	PID_PosX.dTerm2 = 0;
	PID_PosY.dTerm2 = 0;

	Velz = 0.0;
	accAltz = 0.0;

	VelX = 0.0;
	accPosX = 0.0;

	VelY = 0.0;
	accPosY = 0.0;
}

void PID_AltitudeClear()
{
	PID_Altitude.lastError = 0;
	PID_Altitude.iTerm = 00;
	PID_Altitude.dTerm1 = 0;
	PID_Altitude.dTerm2 = 0;
	Velz = 0.0;
	accAltz = 0.0;
}
