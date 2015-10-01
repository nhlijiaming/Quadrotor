/*
 * ClockCounter.h
 *
 *  Created on: 2014��7��22��
 *      Author: Ljm
 */

#ifndef CLOCKCOUNTER_H_
#define CLOCKCOUNTER_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#define ClockCounter_Get() TimerValueGet64(WTIMER0_BASE)
#define ClockCounter_tomicro(a) ((a)/80)

void ClockCounter_Init();

#endif /* CLOCKCOUNTER_H_ */
