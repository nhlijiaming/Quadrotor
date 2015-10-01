/*
 * EEPROM.c
 *
 *  Created on: 2014Äê7ÔÂ26ÈÕ
 *      Author: Ljm
 */
#include "EEPROM.h"

void EEPROM_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    EEPROMInit();
}

void EEPROM_Write()
{
	uint32_t sum;
	uint32_t len;
	uint8_t i;
	uint32_t data[Data_Len];

	len = Data_Len;
	data[ 0] = PID_Roll.P * 100;
	data[ 1] = PID_Roll.I * 1000;
	data[ 2] = PID_Roll.D * 100;
	data[ 3] = PID_Pitch.P * 100;
	data[ 4] = PID_Pitch.I * 1000;
	data[ 5] = PID_Pitch.D * 100;
	data[ 6] = PID_Yaw.P * 100;
	data[ 7] = PID_Yaw.I * 1000;
	data[ 8] = PID_Yaw.D * 100;
	data[ 9] = PID_Altitude.P * 100;
	data[10] = PID_Altitude.I * 10000;
	data[11] = PID_Altitude.D * 1000;
	data[12] = PID_PosX.P * 100;
	data[13] = PID_PosX.I * 10000;
	data[14] = PID_PosX.D * 1000;
	data[15] = PID_Attenuation.P * 100;
	data[16] = PID_Attenuation.I * 1000;
	data[17] = PID_Attenuation.D * 100;
	data[18] = (int32_t)(Att_Offset.Roll * 1000);
	data[19] = (int32_t)(Att_Offset.Pitch * 1000);
	data[20] = (int32_t)(Gyro_Offset.X);
	data[21] = (int32_t)(Gyro_Offset.Y);
	data[22] = (int32_t)(Gyro_Offset.Z);
	data[23] = (int32_t)(Accel_Offset.X);
	data[24] = (int32_t)(Accel_Offset.Y);
	data[25] = (int32_t)(Accel_Offset.Z);
	data[26] = (int32_t)(Xs * 1000);
	data[27] = (int32_t)(Ys * 1000);
	data[28] = (int32_t)(Xb * 1000);
	data[29] = (int32_t)(Yb * 1000);

	sum = len;
	for( i = 0 ; i < len ; i++ )
		sum += *(data+i);
	EEPROMProgram(&len,  EEPROM_STARTADDRESS, 4);
    EEPROMProgram(data, EEPROM_STARTADDRESS + 4, 4*len );
    EEPROMProgram(&sum,  EEPROM_STARTADDRESS + 4 + 4*len, 4);
}

void EEPROM_Read()
{
	uint32_t len, check, sum;
	uint8_t i;
	uint32_t data[Data_Len];

	PID_Pitch.P = 2.0;
	PID_Pitch.I = 0.0;
	PID_Pitch.D = 40.0;
	PID_Pitch.windupGuard = 50;
	PID_Roll.P = 2.0;
	PID_Roll.I = 0.0;
	PID_Roll.D = 40.0;
	PID_Roll.windupGuard = 50;
	PID_Yaw.P = 7.0;
	PID_Yaw.I = 0.0;
	PID_Yaw.D = 5.0;
	PID_Yaw.windupGuard = 50;
	PID_Altitude.P = 0.5;
	PID_Altitude.I = 0.007;
	PID_Altitude.D = 0.008;
	PID_Altitude.windupGuard = 500;
	PID_PosX.P = 0.2;
	PID_PosX.I = 0.0;
	PID_PosX.D = 0.1;
	PID_PosX.windupGuard = 5.0;
	PID_PosY.P = 0.2;
	PID_PosY.I = 0.0;
	PID_PosY.D = 0.1;
	PID_PosY.windupGuard = 5.0;
	PID_Attenuation.P = 0.0;
	PID_Attenuation.I = 0.0;
	PID_Attenuation.D = 0.0;
	Att_Offset.Roll = 2.492;
	Att_Offset.Pitch = -1.885;
	Gyro_Offset.X = 0;
	Gyro_Offset.Y = 0;
	Gyro_Offset.Z = 0;
	Accel_Offset.X = 0;
	Accel_Offset.Y = 0;
	Accel_Offset.Z = 0;
	Xs = 1.0;
	Ys = 1.0;
	Xb = 0.0;
	Yb = 0.0;

	EEPROMRead(&len, EEPROM_STARTADDRESS, 4);
	if (len != Data_Len)
	{
		UARTprintf("EEPROM length error.\nPID parameter reset to defult.\n");
		EEPROM_Write();
		return ;
	}
	EEPROMRead(data, EEPROM_STARTADDRESS + 4, 4*len );
	EEPROMRead(&sum, EEPROM_STARTADDRESS + 4 + 4*len, 4);

	check = len;
	for( i = 0 ; i < len ; i++)
		check += *(data+i);
	if (check == sum)
	{
		UARTprintf("EEPROM data loaded.\n");
		PID_Roll.P  = 		(float)data[0] / 100;
		PID_Roll.I  = 		(float)data[1] / 1000;
		PID_Roll.D  = 		(float)data[2] / 100;
		PID_Pitch.P = 		(float)data[3] / 100;
		PID_Pitch.I = 		(float)data[4] / 1000;
		PID_Pitch.D = 		(float)data[5] / 100;
		PID_Yaw.P   = 		(float)data[6] / 100;
		PID_Yaw.I   = 		(float)data[7] / 1000;
		PID_Yaw.D   = 		(float)data[8] / 100;
		PID_Altitude.P = 	(float)data[9] / 100;
		PID_Altitude.I = 	(float)data[10] / 10000;
		PID_Altitude.D = 	(float)data[11] / 1000;
		PID_PosX.P = 		(float)data[12] / 100;
		PID_PosX.I = 		(float)data[13] / 10000;
		PID_PosX.D = 		(float)data[14] / 1000;
		PID_PosY.P = 		(float)data[12] / 100;
		PID_PosY.I = 		(float)data[13] / 10000;
		PID_PosY.D = 		(float)data[14] / 1000;
		PID_Attenuation.P = (float)data[15] / 100;
		PID_Attenuation.I = (float)data[16] / 1000;
		PID_Attenuation.D = (float)data[17] / 100;
		Att_Offset.Roll = 	(float)((int32_t)data[18]) / 1000;
		Att_Offset.Pitch = 	(float)((int32_t)data[19]) / 1000;
		Gyro_Offset.X = 	((int32_t)data[20]);
		Gyro_Offset.Y = 	((int32_t)data[21]);
		Gyro_Offset.Z = 	((int32_t)data[22]);
		Accel_Offset.X = 	((int32_t)data[23]);
		Accel_Offset.Y = 	((int32_t)data[24]);
		Accel_Offset.Z = 	((int32_t)data[25]);
		Xs = 				(float)data[26] / 1000;
		Ys = 				(float)data[27] / 1000;
		Xb = 				(float)data[28] / 1000;
		Yb = 				(float)data[29] / 1000;

	}
	else
	{
		UARTprintf("EEPROM data check error.\nPID parameter reset to defult.\n");
		EEPROM_Write();
	}
}
