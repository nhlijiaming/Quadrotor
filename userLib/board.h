/*
 * board.h
 *
 *  Created on: 2014Äê7ÔÂ23ÈÕ
 *      Author: Ljm
 */

#ifndef BOARD_H_
#define BOARD_H_

#define debug 1
//#define USE_UART0					// UART data out via USB on board
#define USE_UART1					// UART data out via blutbooth
//#define USE_MPU9250_DMP				// use dmp in mpu9250 instead of calculation
//#define GYRO_CALIBRATION_ENABLE	// calibrate gyro when start up
//#define USE_MAG					// use magnetometer or not
#define EEPROM_STARTADDRESS 0x400	// start address of data in eeprom
#define rxControl (false)

#define RadtoDeg 0.0174532925199f
#define MOTOR_LIMITATION 1000

#define PIDMIX(X,Y,Z) (Throttle + axisPID.Roll * (X) + axisPID.Pitch * (Y) + axisPID.Yaw * (Z))

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "inc/hw_adc.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "ClockCounter.h"
#include "Debugger.h"
#include "EEPROM.h"
//#include "IMU.h"
#include "IMUFilter.h"
#include "Interface.h"
#include "MadgwickAHRS.h"
#include "MPU9250.h"
#include "OLED.h"
#include "OpticalFlow.h"
#include "PID.h"
#include "PPM_Receiver.h"
#include "PPM_Encoder.h"
//#include "MPU9250DMP/MPU9250DMP.h"
#include "Sonar.h"
#include "Voltmeter.h"

extern uint64_t beginTime;

extern uint32_t CPUUsage;
extern float Altitude;
extern float Altitude_RAW;
extern uint32_t Motor[];

extern bool armed;
extern bool Att_CalibrationMode;
extern uint32_t AltitudeNotAvailableCount;
extern int32_t sonarVario;
extern int32_t accZ;

extern bool launching;
extern float setPitch, setRoll;

extern bool AltitudeHold;
extern bool PositionHold;
extern float AltitudeSet;
extern float AltitudeSetRaw;

extern float setPosX;
extern float setPosY;
extern float ready_setPosX;
extern float ready_setPosY;

typedef struct att
{
	float Pitch, Roll, Yaw;
}Att_Data;

extern Att_Data Att, Att_Offset;

extern bool Update_1000Hz;
extern bool Update_500Hz;
extern bool Update_200Hz;
extern bool Update_100Hz;
extern bool Update_50Hz;
extern bool Update_10Hz;
extern bool Update_5Hz;
extern bool Update_1Hz;

int32_t constrain(int32_t amt, int32_t low, int32_t high);

float constrainf(float amt, float low, float high);

int32_t applyDeadband(int32_t value, int32_t deadband);

float applyDeadbandf(float value, float deadband);

#endif /* BOARD_H_ */
