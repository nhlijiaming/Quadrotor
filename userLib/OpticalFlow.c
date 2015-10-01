/*
 * OpticalFlow.c
 *
 *  Created on: 2014Äê8ÔÂ14ÈÕ
 *      Author: Ljm
 */

#include "OpticalFlow.h"

OpticalFlow_Data OpticalFlow;
float conv_factor, radians_to_pixels;


void delay_us(int n)
{
	int i;
	for(i = 4*n; i > 0 ; i--);
}

void ADNS3080_WriteReg(uint32_t address, uint32_t data)
{
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);			// activate CS (low)
	delay_us(1);
	while (SSIBusy(SSI0_BASE)) ;
	SSIDataPut(SSI0_BASE, 0x80 | address);					// send address
    while (SSIBusy(SSI0_BASE)) ;
    delay_us(80);
    HWREG(SSI0_BASE + SSI_O_DR);
    SSIDataPut(SSI0_BASE, data);							// send the data
    while (SSIBusy(SSI0_BASE)) ;
    HWREG(SSI0_BASE + SSI_O_DR);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); 	// deactivate CS (high)
	delay_us(1);
}

uint32_t ADNS3080_ReadReg(uint32_t address)
{
	uint32_t data;
	while (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE)	// clear FIFO
		HWREG(SSI0_BASE + SSI_O_DR);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);		// activate CS (low)
	delay_us(1);
	while (SSIBusy(SSI0_BASE)) ;
	SSIDataPut(SSI0_BASE, address);						// send address
    while (SSIBusy(SSI0_BASE)) ;
    delay_us(80);
	HWREG(SSI0_BASE + SSI_O_DR);
    SSIDataPut(SSI0_BASE, 0x3F);						// get the data
    while (SSIBusy(SSI0_BASE)) ;
    SSIDataGet(SSI0_BASE, &data);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // deactivate CS (high)
    delay_us(1);
	return data;
}

void OpticalFlow_GetMotion()
{
//	float pitch, roll;
//
//    if ( AttFilter2.AttSumCount == 0 ) AttFilter2.AttSumCount = 1;
//    pitch = (float)AttFilter2.PitchSum / (float)AttFilter2.AttSumCount;
//    roll = (float)AttFilter2.RollSum / (float)AttFilter2.AttSumCount;
//    AttFilter2.AttSumCount = 0;
//    AttFilter2.AttTimeSum = 0;
//    AttFilter2.PitchSum = 0.0;
//    AttFilter2.RollSum = 0.0;

	OpticalFlow.motion = ADNS3080_ReadReg(0x02);
	OpticalFlow.dx = (int8_t)ADNS3080_ReadReg(0x03);
	OpticalFlow.dy = (int8_t)ADNS3080_ReadReg(0x04);
	OpticalFlow.squal = ADNS3080_ReadReg(0x05);

	if (OpticalFlow.squal >= 10 && abs(Att.Pitch)<30.0 && abs(Att.Roll)<30.0)
	{
//		OpticalFlow.change_x = OpticalFlow.dx - (diff_roll * radians_to_pixels);
//		OpticalFlow.change_y = OpticalFlow.dy - (-diff_pitch * radians_to_pixels);

		OpticalFlow.change_x = OpticalFlow.dx;
		OpticalFlow.change_y = OpticalFlow.dy;

		OpticalFlow.x_cm = -OpticalFlow.change_x * accAltz * conv_factor;
		OpticalFlow.y_cm = OpticalFlow.change_y * accAltz * conv_factor;

		OpticalFlow.x += OpticalFlow.x_cm;
		OpticalFlow.y += OpticalFlow.y_cm;

		OpticalFlow.compensatedx = OpticalFlow.x;// + 0.015 * Att.Roll * accAltz;
		OpticalFlow.compensatedy = OpticalFlow.y;// + 0.015 * Att.Pitch * accAltz;
	}
}

float OpticalFlow_Distance(float x, float y)
{
	return sqrt((OpticalFlow.compensatedx-x) * (OpticalFlow.compensatedx-x) + (OpticalFlow.compensatedy-y) * (OpticalFlow.compensatedy-y));
}

void OpticalFlow_Init()
{
	int id, inv_id;
	int regVal;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
//    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
//    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
//    GPIOPinConfigure(GPIO_PA4_SSI0RX);
//    GPIOPinConfigure(GPIO_PA5_SSI0TX);
//    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
//    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8);
//    SSIEnable(SSI0_BASE);

	id = ADNS3080_ReadReg(0x00);
	inv_id = ADNS3080_ReadReg(0x3F);
	if (id == 0x17 && inv_id == 0xE8 )
	{
		regVal = ADNS3080_ReadReg(ADNS3080_EXTENDED_CONFIG);
		regVal = (regVal & ~0x01) | 0x01;  //fixed frame rate
		ADNS3080_WriteReg(ADNS3080_EXTENDED_CONFIG, regVal);

		// set frame period to 12000 (0x2EE0)//Frame Rate = Clock Frequency(24M) / Register value(12000) = 2000
		ADNS3080_WriteReg(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
		ADNS3080_WriteReg(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x2E);

		// set 1600 resolution bit
		regVal = ADNS3080_ReadReg(ADNS3080_CONFIGURATION_BITS);
		regVal |= 0x10;
		ADNS3080_WriteReg(ADNS3080_CONFIGURATION_BITS, regVal);

		// calculate conversion_factors
	    conv_factor = ((1.0f / (float)(ADNS3080_PIXELS_X * AP_OPTICALFLOW_ADNS3080_SCALER_1600)) * 2.0f * tanf(AP_OPTICALFLOW_ADNS3080_08_FOV / 2.0f));// 0.00615
	    radians_to_pixels = (ADNS3080_PIXELS_X * AP_OPTICALFLOW_ADNS3080_SCALER_1600) / AP_OPTICALFLOW_ADNS3080_08_FOV;// 162.99

	    conv_factor *= 2.5;
	    radians_to_pixels /= 2.5;

		UARTprintf("Optical Flow ADNS-3080 initialzation done.");
	}
	else
	{
		UARTprintf("Optical Flow ADNS-3080 initialzation error.");
		OLED_Print(0,0,"ADNS-3080 Init ERR");
		LED_Red_Off();
		while (1);
	}
}
