/*
 * ClockCounter.c
 *
 *  Created on: 2014Äê7ÔÂ22ÈÕ
 *      Author: Ljm
 */
#include "ClockCounter.h"

void ClockCounter_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet64(WTIMER0_BASE, 0xFFFFFFFFFFFFFFFF);	// 64-bit timer, reload time: 230584300921s = 2668799day (almost never)
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

//uint64_t ClockCounter_Get()
//{
//	return TimerValueGet64(WTIMER0_BASE);
//}
