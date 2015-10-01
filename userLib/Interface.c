/*
 * Interface.c
 *
 *  Created on: 2014Äê8ÔÂ14ÈÕ
 *      Author: Ljm
 */

#include "Interface.h"

void Interface_Init()
{
	// RGB LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3); // PF1-3 : LED RBG

	// SW
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // PF0 PF4 : SW1 SW2
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_7); // PF0 PF4 : SW1 SW2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5); // PE4 PE3 PA6 PE3 : SW3-6
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	// BEEP init
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); // PA6
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);
}

int8_t Acc_CalFlag, Gyro_CalFlag, Mag_CalFlag, Flight, Mode;
float PointAng;
float PointDis;
float MagAngle;

void ParameterChange(uint8_t index, float delta)
{
    switch (index)
    {
        case  0: Acc_CalFlag += delta; break;
        case  1: Gyro_CalFlag += delta; break;
        case  2: Mag_CalFlag += delta; break;
        case  3:  break;
        case  4: Mode += delta; break;
        case  5: AltitudeSet += delta; break;
        case  6: PointAng += delta; break;
        case  7: PointDis += delta; break;
        case  8: Flight += delta; break;
        case  9: PointDis += delta; break;
        case 10: PointAng += delta; break;
        default: break;
    }
    if (Flight<0) Flight = 1;
    if (Flight>1) Flight = 0;

    if (Mode < 0) Mode = 3;
    if (Mode == 4) Mode ++;
    if (Mode > 5) Mode = 0;

    if (PointAng<0) PointAng += 360;
    if (PointAng>360) PointAng -= 360;

    if (Gyro_CalFlag)
    {
    	Update_1Hz = false;
    	while (!Update_1Hz);
    	Update_1Hz = false;
    	while (!Update_1Hz);
    	MPU6050_GyroCalibration();
    	Gyro_CalFlag = 0;
    }
    if (Mag_CalFlag)
    {
    	Update_1Hz = false;
    	while (!Update_1Hz);
    	Update_1Hz = false;
    	while (!Update_1Hz);
    	MPU6050_MagCalibration();
    	Mag_CalFlag = 0;
    }
}

void AdjustParameter()
{
	int i;
    uint8_t index = 0;
    const uint8_t hint[][15] = {"Acc_Cal","Gyro_Cal","Mag_Cal","Mag","Mode","Altitude","PointAng","PointDis","Flight !!LIGHT",""};
    uint8_t max_index=0;
    char ss[15];
    float value;

    while (hint[max_index+1][0]) max_index++;

    AltitudeSet = 65;
    while (!Flight)
    {
        OLED_Clear();
        OLED_Print(0,0,"Adjust Parameter");
        OLED_Print(20, 2, (byte*)hint[index]);

        MPU9250_GetMag();
        MagAngle = atan2f((double)Mag.Y, (double)Mag.X)*(180/3.1415926);
        if (MagAngle<0) MagAngle += 360.0;

        switch (index)
        {
            case  0: value = (double)Acc_CalFlag; break;
            case  1: value = (double)Gyro_CalFlag; break;
            case  2: value = (double)Mag_CalFlag; break;
            case  3: value = (double)MagAngle; break;
            case  4: value = (double)Mode; break;
            case  5: value = (double)AltitudeSet; break;
            case  6: value = (double)PointAng; break;
            case  7: value = (double)PointDis; break;
            case  8: value = (double)Flight; break;
            case  9: value = (double)PointDis; break;
            case 10: value = (double)PointAng; break;
            default: break;
        }

        sprintf(ss, "%.1f", value);
        OLED_Print(20, 4, (byte*)ss);
        if (SW2_Pressed())       //Down
        {
            if (index > 0) index--; else index = max_index;
            SysCtlDelay(SysCtlClockGet()/100);
            while (SW2_Pressed());
            SysCtlDelay(SysCtlClockGet()/100);
        }
        if (SW1_Pressed())       //Up
        {
            if (index < max_index) index++; else index = 0;
            SysCtlDelay(SysCtlClockGet()/100);
            while (SW1_Pressed());
            SysCtlDelay(SysCtlClockGet()/100);
        }
        if (SW3_Pressed())       //-15
        {
            ParameterChange(index,-15);
            SysCtlDelay(SysCtlClockGet()/100);
            while (SW3_Pressed());
            SysCtlDelay(SysCtlClockGet()/100);
        }
        if (SW4_Pressed())       //-1
        {
            ParameterChange(index,-1);
            SysCtlDelay(SysCtlClockGet()/100);
            while (SW4_Pressed());
            SysCtlDelay(SysCtlClockGet()/100);
        }
        if (SW5_Pressed())       //+1
        {
            ParameterChange(index,1);
            SysCtlDelay(SysCtlClockGet()/100);
            while (SW5_Pressed());
            SysCtlDelay(SysCtlClockGet()/100);
        }
        if (SW6_Pressed())       //+15
        {
            ParameterChange(index,15);
            SysCtlDelay(SysCtlClockGet()/100);
            while (SW6_Pressed());
            SysCtlDelay(SysCtlClockGet()/100);
        }

    	Update_50Hz = false;
    	while (!Update_50Hz);

    }

    if(Mode==0) return ;

    MagAngle = 0.0;
    for(i = 1 ; i <= 10 ; i++)
    {
    	Update_50Hz = false;
    	while (!Update_50Hz);
    	MPU9250_GetMag();
    	MagAngle += atan2f((double)Mag.Y, (double)Mag.X)*(180/3.1415926);
    }
    MagAngle /= 10;
    PointAng += MagAngle;

    for(i = 1 ; i <= 3 ; i++)
    {
    	BEEP_On();
    	Update_1Hz = false;
    	while (!Update_1Hz);
    	BEEP_Off();
    	Update_1Hz = false;
    	while (!Update_1Hz);
    }

    for(i = 1 ; i <= 10 ; i++)
    {
    	BEEP_On();
    	Update_5Hz = false;
    	while (!Update_5Hz);
    	BEEP_Off();
    	Update_10Hz = false;
    	while (!Update_10Hz);
    }

	Update_1Hz = false;
	while (!Update_1Hz);
	Update_1Hz = false;
	while (!Update_1Hz);

	BEEP_On();
	Update_5Hz = false;
	while (!Update_5Hz);
	BEEP_Off();
	Update_10Hz = false;
	while (!Update_10Hz);
	BEEP_On();
	Update_5Hz = false;
	while (!Update_5Hz);
	BEEP_Off();
	Update_10Hz = false;
	while (!Update_10Hz);

	AltitudeSetRaw = AltitudeSet;

	if (Mode == 1)
	{
		ready_setPosX = PointDis * sinf(PointAng/180*3.1415926);
		ready_setPosY = - PointDis * cosf(PointAng/180*3.1415926);
		setPosX = 0.0;
		setPosY = 0.0;
		armed = true;
		AltitudeHold = true;
		PositionHold = true;
		launching = true;
		beginTime = ClockCounter_Get();
	}

	if (Mode == 2)
	{
		setPosX = 0.0;
		setPosY = 0.0;
		armed = true;
		AltitudeHold = true;
		PositionHold = true;
		beginTime = ClockCounter_Get();
	}

	if (Mode == 3)
	{
		ready_setPosX = PointDis * sinf(PointAng/180*3.1415926);
		ready_setPosY = - PointDis * cosf(PointAng/180*3.1415926);
		setPosX = 0.0;
		setPosY = 0.0;
		armed = true;
		AltitudeHold = true;
		PositionHold = true;
		launching = true;
		beginTime = ClockCounter_Get();
	}

	if (Mode == 5)
	{
		armed = true;
		AltitudeHold = true;
		PositionHold = true;
		launching = true;
		beginTime = ClockCounter_Get();
	}
//	OLED_Clear();
//	OLED_Print(0, 0, "mag");
//	angle = atan2f((double)Mag.Y, (double)Mag.X)*(180/3.1415926)+180 ;
//	sprintf(ss, "%.1f", angle);
//	OLED_Print(0, 2, ss);
}
