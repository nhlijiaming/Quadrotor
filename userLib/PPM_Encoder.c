/*
 * PPM.c
 *
 *  Created on: 2014Äê7ÔÂ20ÈÕ
 *      Author: Ljm
 */
#include "PPM_Encoder.h"

const float T = 1000.0 / Freq;
uint32_t PPM_Period;

void PPMEncoder_Init()
{
	PPM_Period = (uint32_t) (80000000 / Freq);

	//PB4 - PB7: PWN OUT - T1CCP0 T1CCP1 T0CCP0 T0CCP1

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //************* configure TIMER0
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB7_T0CCP1);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    TimerPrescaleSet(TIMER0_BASE, TIMER_BOTH, PPM_Period >> 16);
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, PPM_Period & 0xFFFF);

    TimerControlLevel(TIMER0_BASE, TIMER_BOTH, true);		//PWM invert

    //************* configure TIMER1
    GPIOPinConfigure(GPIO_PB4_T1CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PB5_T1CCP1);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5);

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    TimerPrescaleSet(TIMER1_BASE, TIMER_BOTH, PPM_Period >> 16);
    TimerLoadSet(TIMER1_BASE, TIMER_BOTH, PPM_Period & 0xFFFF);

    TimerControlLevel(TIMER1_BASE, TIMER_BOTH, true);		//PWM invert

    PPMEncoder_Update(0,0,0,0);
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

void PPMEncoder_Update(int m1, int m2, int m3, int m4) //m1~m4 : 0~1000
{
    uint32_t ccr1, ccr2, ccr3, ccr4;

    // limit the ranges of m1 to m4
    if (m1 <   0) m1 = 0;
    if (m1 > 1000) m1 = 1000;
    if (m2 <   0) m2 = 0;
    if (m2 > 1000) m2 = 1000;
    if (m3 <   0) m3 = 0;
    if (m3 > 1000) m3 = 1000;
    if (m4 <   0) m4 = 0;
    if (m4 > 1000) m4 = 1000;

    // 0 - 100 change to 1ms - 2ms
    ccr1 = (uint32_t) ((1000 + m1) * (PPM_Period / 1000 / T));
    ccr2 = (uint32_t) ((1000 + m2) * (PPM_Period / 1000 / T));
    ccr3 = (uint32_t) ((1000 + m3) * (PPM_Period / 1000 / T));
    ccr4 = (uint32_t) ((1000 + m4) * (PPM_Period / 1000 / T));

    //set the register
    HWREG(TIMER1_BASE + TIMER_O_TAPMR) = ccr1 >> 16;
    HWREG(TIMER1_BASE + TIMER_O_TAMATCHR) = ccr1 & 0xFFFF;

    HWREG(TIMER1_BASE + TIMER_O_TBPMR) = ccr2 >> 16;
    HWREG(TIMER1_BASE + TIMER_O_TBMATCHR) = ccr2 & 0xFFFF;

    HWREG(TIMER0_BASE + TIMER_O_TAPMR) = ccr3 >> 16;
    HWREG(TIMER0_BASE + TIMER_O_TAMATCHR) = ccr3 & 0xFFFF;

    HWREG(TIMER0_BASE + TIMER_O_TBPMR) = ccr4 >> 16;
    HWREG(TIMER0_BASE + TIMER_O_TBMATCHR) = ccr4 & 0xFFFF;

    return ;
}
