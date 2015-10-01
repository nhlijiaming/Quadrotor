/*
 * IMU.c
 *
 *  Created on: 2014年7月22日
 *      Author: Ljm
 */
#include "IMU.h"

float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
const float Kp = 2.0f;
const float Ki = 0.005f;
const float halfT = 0.0005f;

void IMU_Init()
{
    q0 =  1.0;
    q1 = q2 = q3 = 0.0;
    exInt = eyInt = ezInt = 0.0;
}

float invSqrt(float x) // an efficient way to compute 1/Sqrt(x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

bool IMU_Update(float ax,float ay,float az,float gx,float gy,float gz)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
  //
	//四元数乘法运算
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	//
	//归一化处理
	norm = invSqrt(ax*ax + ay*ay + az*az);
	if(norm==0) return 0;
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
  //
	//建立小四轴坐标系
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//
	//坐标系和重力叉积运算
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	//
	//比例运算
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	//
	//陀螺仪融合
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	//
	//解微分方程
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	//
	//归一化处理
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(norm==0) return 0;
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	//
	//欧拉角转换
	Att.Roll = asin( -2 * q1q3 + 2 * q0q2 ) * 57.295780f;
	Att.Pitch = atan2( 2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * 57.295780f;
    //yaw=yaw-gz*bs004_mpu6050_gyro_scale;
	Att.Yaw = atan2( 2 * q1q2 + 2*q0q3, -2 * q2q2 - 2 * q3q3 + 1) * 57.295780f;
	//
	return 1;
}
