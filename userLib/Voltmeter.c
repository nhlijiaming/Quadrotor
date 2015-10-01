/*
 * Voltmeter.c
 *
 *  Created on: 2014Äê7ÔÂ22ÈÕ
 *      Author: Ljm
 */

#include "Voltmeter.h"

void Voltmeter_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC0_BASE, 0);
    ADCIntClear(ADC0_BASE, 0);
}

uint32_t Voltmeter_Get()
{
	uint32_t adc_value;
	ADCProcessorTrigger(ADC0_BASE, 0);
	while (!ADCIntStatus(ADC0_BASE, 0, false));
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceDataGet(ADC0_BASE, 0, &adc_value);
	return adc_value;
}
