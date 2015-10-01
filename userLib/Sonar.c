/*
 * Sonar.c
 *
 *  Created on: 2014Äê7ÔÂ31ÈÕ
 *      Author: Ljm
 */
#include "Sonar.h"

void Sonar_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // PE1
}

void Sonar_Trig()
{
	uint8_t i;
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
	for( i = 70 ; i > 0 ; i-- ); // 10us
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
}

/* limit the amplitude wave filter
 * dAlt <= 20  		Gain = 1.0
 * 20 < dAlt <= 50  Gain = 1.0 ~ 0.2
 * dAlt <  50		Gain = 0.2
 */
float Sonar_LimitationFilter(float currentAlt)
{
#define threshold 10.0
	float k;
	float dAlt;
	static float Alt = 0.0;

	dAlt = abs(currentAlt - Alt);
	if (dAlt < threshold)
	{
		Alt = currentAlt;
	}
	else
	{
		k = 0.0;
		if (dAlt < 50.0)
		{
			k = 1.0 - (dAlt - threshold) / 30;
		}
		k = max(k, 0.2);
		Alt = Alt + k * (currentAlt - Alt);
	}
	return Alt;
}

float Sonar_MedianFilter(float currentAlt)
{
	static float AltData[3]={0.0};
	float tmp;
	float sort[3];

	AltData[0] = AltData[1];
	AltData[1] = AltData[2];
	AltData[2] = currentAlt;

	sort[0] = AltData[0];
	sort[1] = AltData[1];
	sort[2] = AltData[2];

	if (sort[0] > sort[1]) { tmp = sort[0]; sort[0] = sort[1]; sort[1] = tmp; }
	if (sort[0] > sort[2]) { tmp = sort[0]; sort[0] = sort[2]; sort[2] = tmp; }
	if (sort[1] > sort[2]) { tmp = sort[1]; sort[1] = sort[2]; sort[2] = tmp; }

	return sort[1];

}

//uint8_t ExcitationCovariance=2;
//uint8_t MeasureCovariance=40;
//uint32_t Sonar_KalmanFilter(uint32_t Measure, bool available)
//{
//	static float EstimateCovariance = 10;
//	static float EstimateValue = 0;
////	static float Last1=0, Last2=0, Last3=0;
//	float KalmanGain;
//
//	EstimateCovariance = sqrt( EstimateCovariance * EstimateCovariance + ExcitationCovariance * ExcitationCovariance);
//	KalmanGain = EstimateCovariance * sqrt( 1 / ( EstimateCovariance * EstimateCovariance + MeasureCovariance * MeasureCovariance ));
//	if (!available) KalmanGain = 0;
//
//	// prediction have second precision
////	EstimateValue = (3*Last3 - 3*Last2 + Last1) + KalmanGain * ( Measure - EstimateValue );
////	Last1 = Last2;
////	Last2 = Last3;
////	Last3 = EstimateValue;
//
//	EstimateValue = EstimateValue + KalmanGain * ( Measure - EstimateValue );
//	EstimateCovariance = sqrt( (1 - KalmanGain) * EstimateCovariance * EstimateCovariance);
//	return (uint32_t)EstimateValue;
//}


