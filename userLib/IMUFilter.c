/*
 * IMUFilter.c
 *
 *  Created on: 2014Äê8ÔÂ10ÈÕ
 *      Author: Ljm
 */
#include "IMUFilter.h"

ACC_FILTER AccXFilter, AccYFilter, AccZFilter;
ATT_FILTER AttFilter, AttFilter2;

// Rotate Estimated vector(s)
void rotateV(float v[], float att[])
{
    float mat[3][3];
    float v_tmp[3];
    float cosx, sinx, cosy, siny;
    float cosz, sinz;
    float cosxcosz, cosxsinz, sinxcosz, sinxsinz;

    v_tmp[0] = v[0];
    v_tmp[1] = v[1];
    v_tmp[2] = v[2];

    cosx = cosf(att[0]);
    sinx = sinf(att[0]);
    cosy = cosf(att[1]);
    siny = sinf(att[1]);
    cosz = cosf(att[2]);
    sinz = sinf(att[2]);

    cosxcosz = cosz * cosx;
    cosxsinz = sinz * cosx;
    sinxcosz = sinx * cosz;
    sinxsinz = sinx * sinz;

    mat[0][0] = cosxcosz + (sinxsinz * siny);
    mat[0][1] = sinz * cosy;
    mat[0][2] = (sinxcosz) - (cosxsinz * siny);

    mat[1][0] = -cosxsinz + (sinxcosz * siny);
    mat[1][1] = cosz * cosy;
    mat[1][2] = -siny * sinz - (cosxcosz * siny);

    mat[2][0] = -sinx * cosy;
    mat[2][1] = siny;
    mat[2][2] = cosx * cosy;

    v[0] = v_tmp[0] * mat[0][0] + v_tmp[1] * mat[0][1]+ v_tmp[2] * mat[0][2];
    v[1] = v_tmp[0] * mat[1][0] + v_tmp[1] * mat[1][1]+ v_tmp[2] * mat[1][2];
    v[2] = v_tmp[0] * mat[2][0] + v_tmp[1] * mat[2][1]+ v_tmp[2] * mat[2][2];
}

void AccZ_Filter(uint32_t deltaT)
{
	static float accLPF[3]={0.0};
	static float accSmooth[3] = {0.0};
	uint8_t i;
    float dT = 0;
    float accRAWSmooth[3];
	int32_t acc_raw[3];
	float att[3];

	acc_raw[0] = Accel.X;
	acc_raw[1] = Accel.Y;
	acc_raw[2] = Accel.Z;
	att[0] = Att.Roll*3.14159f/180.0f;
	att[1] = Att.Pitch*3.14159f/180.0f;
	att[2] = Att.Yaw*3.14159f/180.0f;

	for(i = 0 ; i < 3 ; i++)
	{
		// here is a first-order IIR low-pass filter
		// fc = 0.09pi  rad/sample
		// NUM=[1/factor 0]  DEN = [1 -1+1/factor]
		// 1 * y[i] = 1/factor * x[i] + 0 * x[i-1] - (1-1/factor) * y[i-1]
		accLPF[i] = (1.0f - (1.0f / acc_lpf_factor)) * accLPF[i] + (1.0f / acc_lpf_factor) * acc_raw[i];
		accRAWSmooth[i] = accLPF[i];
	}

	rotateV(accRAWSmooth, att);

	accRAWSmooth[0] -= Accel_Offset.X;
	accRAWSmooth[1] -= Accel_Offset.Y;
	accRAWSmooth[2] -= Accel_Offset.Z;

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    for(i = 0 ; i < 3 ; i++)
    {
    	accSmooth[i] = accSmooth[i] + (dT / (fc_acc + dT)) * (accRAWSmooth[i] - accSmooth[i]); // low pass filter
    }

    AccXFilter.AccSum += applyDeadband(accSmooth[0], 0);//40
    AccXFilter.AccTimeSum += deltaT;
    AccXFilter.AccSumCount ++;

    AccYFilter.AccSum += applyDeadband(accSmooth[1], 0);//40
    AccYFilter.AccTimeSum += deltaT;
    AccYFilter.AccSumCount ++;

    AccZFilter.AccSum += applyDeadband(accSmooth[2], 0);//40
    AccZFilter.AccTimeSum += deltaT;
    AccZFilter.AccSumCount ++;
}

void Att_Filter(uint32_t deltaT)
{
	static float attLPF[2];
    float dT = 0;

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

	// here is a first-order IIR low-pass filter
	// fc = 0.09pi  rad/sample
	// NUM=[1/factor 0]  DEN = [1 -1+1/factor]
	// 1 * y[i] = 1/factor * x[i] + 0 * x[i-1] - (1-1/factor) * y[i-1]
	attLPF[0] = attLPF[0] + (dT / (fc_att + dT)) * (Att.Pitch - attLPF[0]);
	attLPF[1] = attLPF[1] + (dT / (fc_att + dT)) * (Att.Roll  - attLPF[1]);
	AttFilter.PitchSum += attLPF[0];
	AttFilter.RollSum += attLPF[1];
	AttFilter.AttSumCount ++;

	AttFilter2.PitchSum += attLPF[0];
	AttFilter2.RollSum += attLPF[1];
	AttFilter2.AttSumCount ++;
}
