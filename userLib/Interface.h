/*
 * Interface.h
 *
 *  Created on: 2014Äê8ÔÂ14ÈÕ
 *      Author: Ljm
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include "board.h"

#define LED_Red_On() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1)
#define LED_Blue_On() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2)
#define LED_Green_On() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3)

#define LED_Red_Off() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0)
#define LED_Blue_Off() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0)
#define LED_Green_Off() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0)

#define LED_Red_Toggle() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1))
#define LED_Blue_Toggle() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
#define LED_Green_Toggle() GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))

#define LED_Red_IsOn() GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)
#define LED_Blue_IsOn() GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)
#define LED_Green_IsOn() GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)

#define SW1_Pressed() (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
#define SW2_Pressed() (0)//!GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_7))
#define SW3_Pressed() (!GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4))
#define SW4_Pressed() (!GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_5))
#define SW5_Pressed() (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
#define SW6_Pressed() (!GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3))

#define BEEP_On() GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);//&(~(debug<<6)))
#define BEEP_Off() GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0);

extern int8_t Mode;
extern float PointAng;
extern float PointDis;

void Interface_Init();

void AdjustParameter();

#endif /* INTERFACE_H_ */
