/*
 * i2c.c
 *
 *  Created on: 2014Äê7ÔÂ24ÈÕ
 *      Author: Ljm
 */

#include "i2c.h"

int stm32l_get_clock_ms(unsigned long *count)
{
	return *count;
}

void Delay(unsigned long num_ms)
{
	uint32_t i;
	while (num_ms)
	{
		num_ms --;
		for(i = 6700 ; i > 0 ; i--);
	}
}

void SPI_Init()
{
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
}


uint8_t Sensors_I2C_WriteRegister_swap(uint8_t saddr, uint8_t addr, uint8_t len, uint8_t *data)
{
	uint8_t i;
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);	// activate CS (low)
	while (SSIBusy(SSI0_BASE)) ;
	SSIDataPut(SSI0_BASE, addr);					// send address
    while (SSIBusy(SSI0_BASE)) ;
    HWREG(SSI0_BASE + SSI_O_DR);
    for( i = 0 ; i < len ; i ++)
    {
    	SSIDataPut(SSI0_BASE, data[i]);					// send the data
    	while (SSIBusy(SSI0_BASE)) ;
    	HWREG(SSI0_BASE + SSI_O_DR);
    }
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // deactivate CS (high)
	return 0;
}

uint8_t Sensors_I2C_ReadRegister_swap(uint8_t saddr, uint8_t addr, uint8_t len, uint8_t *data)
{
	uint8_t i;
	uint32_t temp;
	while (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE)	// clear FIFO
		HWREG(SSI0_BASE + SSI_O_DR);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);	// activate CS (low)
	while (SSIBusy(SSI0_BASE)) ;
	SSIDataPut(SSI0_BASE, 0x80 | addr);			// send address
    while (SSIBusy(SSI0_BASE)) ;
	HWREG(SSI0_BASE + SSI_O_DR);
    for( i = 0 ; i < len ; i ++)
    {
    	SSIDataPut(SSI0_BASE, 0xFF);					// get the data
    	while (SSIBusy(SSI0_BASE)) ;
    	SSIDataGet(SSI0_BASE, &temp );
    	data[i] = temp;
    }
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // deactivate CS (high)
	return 0;
}
