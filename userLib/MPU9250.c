/*
 * MPU9250.c
 *
 *  Created on: 2014年7月21日
 *   Modify on: 2014年7月22日
 *      Author: Ljm
 *
 */
#include "MPU9250.h"

#define MPU9250_InitRegNum 14

Sensor Accel, Gyro, Mag;
Sensor Accel_Offset, Gyro_Offset, Mag_ASA;

int32_t Xmax, Xmin;
int32_t Ymax, Ymin;
float Xs, Ys;
float Xb, Yb;

void MPU9250_Delay_nop( uint32_t nCnt )
{
  while(nCnt--) {

  }
}

void MPU9250_WriteReg(uint32_t address, uint32_t data)
{
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);	// activate CS (low)
	while (SSIBusy(SSI0_BASE)) ;
	SSIDataPut(SSI0_BASE, address);					// send address
    while (SSIBusy(SSI0_BASE)) ;
    HWREG(SSI0_BASE + SSI_O_DR);
    SSIDataPut(SSI0_BASE, data);					// send the data
    while (SSIBusy(SSI0_BASE)) ;
    HWREG(SSI0_BASE + SSI_O_DR);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // deactivate CS (high)
}

uint32_t MPU9250_ReadReg(uint32_t address)
{
	uint32_t data;
	while (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE)	// clear FIFO
		HWREG(SSI0_BASE + SSI_O_DR);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);	// activate CS (low)
	while (SSIBusy(SSI0_BASE)) ;
	SSIDataPut(SSI0_BASE, 0x80 | address);			// send address
    while (SSIBusy(SSI0_BASE)) ;
	HWREG(SSI0_BASE + SSI_O_DR);
    SSIDataPut(SSI0_BASE, 0xFF);					// get the data
    while (SSIBusy(SSI0_BASE)) ;
    SSIDataGet(SSI0_BASE, &data);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // deactivate CS (high)
	return data;
}

void MPU9250_WriteAuxReg(uint32_t address, uint32_t data)
{
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0C);          // Set AK8963_I2C_ADDR = 7'b000_1100
	MPU9250_Delay_nop(10000);
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, address);     	// Set Write Reg
	MPU9250_WriteReg(MPU6500_I2C_SLV0_DO, data);   	// Read Data
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Read
	MPU9250_Delay_nop(10000);
}

uint32_t MPU9250_ReadAuxReg(uint32_t address)
{
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
	//MPU9250_Delay_nop(100000);
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, address);     	// Set Write Reg
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Read
	MPU9250_Delay_nop(10000);
	return MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00);   	// Read Data
}

void MPU9250_SetParameter( void )
{
	uint8_t i = 0;
	uint32_t MPU6500_Init_Data[MPU9250_InitRegNum][2] = {
			{0x80, MPU6500_PWR_MGMT_1},     // REG107 Reset Device
			{0x01, MPU6500_PWR_MGMT_1},     // REG107 Clock Source
			{0x00, MPU6500_PWR_MGMT_2},     // REG108 Enable Acc & Gyro
			{0x04, MPU6500_CONFIG},         // REG26 FCHOICE=11 DLPF=2 Gyro Bandwidth 92Hz Rates 1K
			{0x18, MPU6500_GYRO_CONFIG},    // REG27 +-2000dps
			{0x00, MPU6500_ACCEL_CONFIG},   // REG28 +-2G
			{0x03, MPU6500_ACCEL_CONFIG_2}, // REG29 Set Acc Data Rates ACCEL Bandwidth 92Hz Rates 1K
			{0x30, MPU6500_INT_PIN_CFG},    // REG55 BYPASS_EN
			{0x40, MPU6500_I2C_MST_CTRL},   // REG36 I2C Speed 348 kHz
			{0x20, MPU6500_USER_CTRL},      // REG106 Enable AUX

			// Set Slave to Read AK8963
			{0x8C, MPU6500_I2C_SLV0_ADDR},  // REG37 AK8963_I2C_ADDR ( 7'b000_1100 )
			{0x00, MPU6500_I2C_SLV0_REG},   // REG38 AK8963_WIA ( 0x00 )
			{0x81, MPU6500_I2C_SLV0_CTRL},  // REG39 Enable
			{0x01, MPU6500_I2C_MST_DELAY_CTRL}	//REG103
	};

	for(i=0; i<MPU9250_InitRegNum; i++) {
		MPU9250_WriteReg(MPU6500_Init_Data[i][1], MPU6500_Init_Data[i][0]);
		MPU9250_Delay_nop(10000);
	}
}

void MPU9250_GetAccel()
{
	int16_t temp;
	temp = MPU9250_ReadReg(MPU6500_ACCEL_XOUT_H) << 8;
	temp += MPU9250_ReadReg(MPU6500_ACCEL_XOUT_L);
	Accel.X = (int32_t)temp;
	temp = MPU9250_ReadReg(MPU6500_ACCEL_YOUT_H) << 8;
	temp += MPU9250_ReadReg(MPU6500_ACCEL_YOUT_L);
	Accel.Y = (int32_t)temp;
	temp = MPU9250_ReadReg(MPU6500_ACCEL_ZOUT_H) << 8;
	temp += MPU9250_ReadReg(MPU6500_ACCEL_ZOUT_L);
	Accel.Z = (int32_t)temp;
}

void MPU9250_GetGyro()
{
	int16_t temp;
	temp = MPU9250_ReadReg(MPU6500_GYRO_XOUT_H) << 8;
	temp += MPU9250_ReadReg(MPU6500_GYRO_XOUT_L);
	Gyro.X = (int32_t)temp;
	temp = MPU9250_ReadReg(MPU6500_GYRO_YOUT_H) << 8;
	temp += MPU9250_ReadReg(MPU6500_GYRO_YOUT_L);
	Gyro.Y = (int32_t)temp;
	temp = MPU9250_ReadReg(MPU6500_GYRO_ZOUT_H) << 8;
	temp += MPU9250_ReadReg(MPU6500_GYRO_ZOUT_L);
	Gyro.Z = (int32_t)temp;

	Gyro.X += Gyro_Offset.X;
	Gyro.Y += Gyro_Offset.Y;
	Gyro.Z += Gyro_Offset.Z;
}

void MPU6050_GyroCalibration()
{
    int cycles = 7000;
    int i;
    int32_t x, y, z;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0 );
    Gyro_Offset.X = 0;
    Gyro_Offset.Y = 0;
    Gyro_Offset.Z = 0;
    x = 0;
    y = 0;
    z = 0;
    Update_1000Hz = false;
    for(i = 0 ; i < cycles ; i++)
    {
        while (!Update_1000Hz);
        Update_1000Hz = false;
    	MPU9250_GetGyro();
    	x += Gyro.X;
    	y += Gyro.Y;
    	z += Gyro.Z;
    }
    Gyro_Offset.X = -x / cycles;
    Gyro_Offset.Y = -y / cycles;
    Gyro_Offset.Z = -z / cycles;
    EEPROM_Write();
	BEEP_On();
	Update_5Hz = false;
	while (!Update_5Hz);
	BEEP_Off();
    return ;
}

//void MPU9250_GetMag()
//{
//#ifdef USE_MAG
//	int16_t temp;
//	int16_t temp2;
//
//	temp = MPU9250_ReadAuxReg(AK8963_ST1);
//	if (temp & 0x01)	// data ready
//	{
//		temp = MPU9250_ReadAuxReg(AK8963_HXL);
//		temp2 = MPU9250_ReadAuxReg(AK8963_HXH)<<8;
//		temp2 += temp;
//		Mag.X = (int32_t) (temp2);
//		temp = MPU9250_ReadAuxReg(AK8963_HYL);
//		temp2 = MPU9250_ReadAuxReg(AK8963_HYH)<<8;
//		temp2 += temp;
//		Mag.Y = (int32_t) (temp2);
//		temp = MPU9250_ReadAuxReg(AK8963_HZL);
//		temp2 = MPU9250_ReadAuxReg(AK8963_HZH)<<8;
//		temp2 += temp;
//		Mag.Z = (int32_t) (temp2);
//
//		temp = MPU9250_ReadAuxReg(AK8963_ST2);
//		if (temp & 0x08)	// magnetic sensor overflow occurred
//		{
//			Mag.X = 0;
//			Mag.Y = 0;
//			Mag.Z = 0;
//		}
//	}
//#endif
//}

bool MPU9250_GetMag()
{
	bool available;
	available = false;

#ifdef USE_MAG
	int16_t temp;
	int16_t temp2;
	int16_t temp3;

	temp = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00);	// read AK8963_ST1
	if (temp & 0x01)
	{
		available = true;

		temp = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_01);
		temp2 = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_02)<<8;
		temp3 = temp + temp2;
		Mag.X = (int32_t) (temp3);
		temp = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_03);
		temp2 = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_04)<<8;
		temp3 = temp + temp2;
		Mag.Y = (int32_t) (temp3);
		temp = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_05);
		temp2 = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_06)<<8;
		temp3 = temp + temp2;
		Mag.Z = (int32_t) (temp3);
		temp = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_07);

		Mag.X = Xs * Mag.X + Xb;
		Mag.Y = Ys * Mag.Y + Yb;
//		Mag.Z =

		if (temp & 0x08)	// magnetic sensor overflow occurred
		{
			Mag.X = 0;
			Mag.Y = 0;
			Mag.Z = 0;
			available = false;
		}
		temp = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_08);
	}

	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);     	// Set Write Reg
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x89);          // Start Read
#endif
	return available;
}

void MPU9250_GetMagAdjValue()
{
#ifdef USE_MAG
	Mag_ASA.X = MPU9250_ReadAuxReg(AK8963_ASAX);
	Mag_ASA.Y = MPU9250_ReadAuxReg(AK8963_ASAY);
	Mag_ASA.Z = MPU9250_ReadAuxReg(AK8963_ASAZ);
#endif
}

void MPU9250_MagAdjust()
{
#ifdef USE_MAG
	Mag.X = Mag.X * ((Mag_ASA.X - 128) / 256 + 1);
	Mag.Y = Mag.Y * ((Mag_ASA.Y - 128) / 256 + 1);
	Mag.Z = Mag.Z * ((Mag_ASA.Z - 128) / 256 + 1);
#endif
}

void MPU6050_MagCalibration()
{
    int cycles = 500;
    int i;

    Xmin = 32767;
    Xmax = -32767;
    Ymin = 32767;
    Ymax = -32767;

    Xs = 1.0;
    Ys = 1.0;
    Xb = 0.0;
    Yb = 0.0;

    for(i = 0 ; (i < cycles)&&(!SW1_Pressed()) ; i++)
    {
        while (!MPU9250_GetMag())
        {
            Update_50Hz = false;
            while (!Update_50Hz);
        }

        Xmax = max(Xmax, Mag.X);
        Xmin = min(Xmin, Mag.X);

        Ymax = max(Ymax, Mag.Y);
        Ymin = min(Ymin, Mag.Y);
    }

    Xs = 1.0; // let X axis to be scaled as one
    Ys = (float)(Xmax - Xmin)/(float)(Ymax - Ymin);

    Xb = Xs * (0.5 * (Xmax - Xmin) - Xmax);
    Yb = Ys * (0.5 * (Ymax - Ymin) - Ymax);
    EEPROM_Write();
    return ;
}

void MPU9250_Init()
{
	uint32_t Device_ID;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI0_BASE);

    //
    // Display the example setup on the console.
    //
    UARTprintf("\nPower up.\n");

    // MPU-6500 Device ID check and initial
    Device_ID = 0x0;
    Device_ID = MPU9250_ReadReg(MPU6500_WHO_AM_I);
    if(Device_ID == MPU6500_Device_ID)
    {
    	UARTprintf("MPU6500 Device ID: 0x%X\n--- MPU6500 Initialization done.\n", Device_ID);
    }
    else
    {
    	OLED_Print(0,0,"MPU6500 ERR");
    	UARTprintf("MPU6500 Device ID: 0x%X\n--- MPU6500 Initialization error.\n", Device_ID);
    	while (1);
    }
	MPU9250_SetParameter();

#ifdef USE_MAG
    // AK9875 Device ID check
    // if AK9875 ID check check failed with no reason, re-power-up the system.
	MPU9250_WriteAuxReg(AK8963_CNTL2, 0x01);	//soft reset
	MPU9250_Delay_nop(100000);

    Device_ID = 0x00;
    Device_ID = MPU9250_ReadAuxReg(AK8963_WIA);

    if(Device_ID == AK8963_Device_ID)
    {
    	UARTprintf("AK8963 Device ID: 0x%X\n--- AK8963 Initialization done.\n", Device_ID);
    	LED_Blue_Off();
    }
    else
    {
    	UARTprintf("AK8963 Device ID: 0x%X\n--- AK8963 Initialization error.\n", Device_ID);
    	OLED_Print(0,0,"AK8963 Init ERR");
    	LED_Red_Off();
    	LED_Green_Off();
    }
    MPU9250_GetMagAdjValue();
    MPU9250_WriteAuxReg(AK8963_CNTL1, 0x16);	// 16-bits, Rate 100Hz
    //MPU9250_WriteAuxReg(AK8963_CNTL1, 0x2);	// 14-bits, Rate 8Hz
    Device_ID = MPU9250_ReadAuxReg(AK8963_CNTL1);
#else
    UARTprintf("AK8963 is disabled.\n");
#endif
}
