/*
 * PPM_Receiver.c
 *
 *  Created on: 2014Äê7ÔÂ21ÈÕ
 *      Author: Ljm
 */

#include "PPM_Receiver.h"

volatile int Timer2A_MatchValue=0, Timer2A_deltaMatchValue=0;
volatile int Timer2B_MatchValue=0, Timer2B_deltaMatchValue=0;
volatile int Timer3A_MatchValue=0, Timer3A_deltaMatchValue=0;
volatile int Timer3B_MatchValue=0, Timer3B_deltaMatchValue=0;
volatile int WTimer1A_MatchValue=0, WTimer1A_deltaMatchValue=0;
volatile int WTimer1B_MatchValue=0, WTimer1B_deltaMatchValue=0;
volatile int rxChannel_RAW[8]={0};	// raw data, in us

rxData rxCommand;

void Timer2AIntHandler(void)// channel 1
{
//	if (HWREG(TIMER2_BASE + TIMER_O_MIS) & 0x0004)			// CAEMIS = 1, TimerA capture event interrupt occurs
//	{
//		TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);
//		if (Timer2A_MatchValue != 0)							//captured a negative edge and compute the pulse time
//		{
//			Timer2A_deltaMatchValue = Timer2A_MatchValue - TimerValueGet(TIMER2_BASE, TIMER_A);	// compute the pulse time in sysClocks
//			Timer2A_deltaMatchValue &= (1 << 24) - 1;		// ignore the sign bit
//			rxChannel_RAW[0] = Timer2A_deltaMatchValue / 80;		// compute the pulse time in us
//			HWREG(TIMER2_BASE + TIMER_O_CTL) &= ~(0x0004);	// capture positive edges
//			Timer2A_MatchValue = 0;
//		}
//		else
//		{
//			Timer2A_MatchValue = TimerValueGet(TIMER2_BASE, TIMER_A);
//			HWREG(TIMER2_BASE + TIMER_O_CTL) |= 0x0004;		// capture negative edges
//		}
//	}
}

void Timer2BIntHandler(void)// channel 2
{
//	if (HWREG(TIMER2_BASE + TIMER_O_MIS) & 0x0400)			// CBEMIS = 1, TimerA capture event interrupt occurs
//	{
//		TimerIntClear(TIMER2_BASE, TIMER_CAPB_EVENT);
//		if (Timer2B_MatchValue != 0)							//captured a negative edge and compute the pulse time
//		{
//			Timer2B_deltaMatchValue = Timer2B_MatchValue - TimerValueGet(TIMER2_BASE, TIMER_B);	// compute the pulse time in sysClocks
//			Timer2B_deltaMatchValue &= (1 << 24) - 1;		// ignore the sign bit
//			rxChannel_RAW[1] = Timer2B_deltaMatchValue / 80;		// compute the pulse time in us
//			HWREG(TIMER2_BASE + TIMER_O_CTL) &= ~(0x0C00);	// capture positive edges
//			Timer2B_MatchValue = 0;
//		}
//		else
//		{
//			Timer2B_MatchValue = TimerValueGet(TIMER2_BASE, TIMER_B);
//			HWREG(TIMER2_BASE + TIMER_O_CTL) |= 0x0C00;		// capture negative edges
//		}
//	}
}

void Timer3AIntHandler(void)// channel 3
{
//	if (HWREG(TIMER3_BASE + TIMER_O_MIS) & 0x0004)			// CAEMIS = 1, TimerA capture event interrupt occurs
//	{
//		TimerIntClear(TIMER3_BASE, TIMER_CAPA_EVENT);
//		if (Timer3A_MatchValue != 0)							//captured a negative edge and compute the pulse time
//		{
//			Timer3A_deltaMatchValue = Timer3A_MatchValue - TimerValueGet(TIMER3_BASE, TIMER_A);	// compute the pulse time in sysClocks
//			Timer3A_deltaMatchValue &= (1 << 24) - 1;		// ignore the sign bit
//			rxChannel_RAW[2] = Timer3A_deltaMatchValue / 80;		// compute the pulse time in us
//			HWREG(TIMER3_BASE + TIMER_O_CTL) &= ~(0x0004);	// capture positive edges
//			Timer3A_MatchValue = 0;
//		}
//		else
//		{
//			Timer3A_MatchValue = TimerValueGet(TIMER3_BASE, TIMER_A);
//			HWREG(TIMER3_BASE + TIMER_O_CTL) |= 0x0004;		// capture negative edges
//		}
//	}
}

void Timer3BIntHandler(void) //channel 4
{
//	if (HWREG(TIMER3_BASE + TIMER_O_MIS) & 0x0400)			// CBEMIS = 1, TimerA capture event interrupt occurs
//	{
//		TimerIntClear(TIMER3_BASE, TIMER_CAPB_EVENT);
//		if (Timer3B_MatchValue != 0)							//captured a negative edge and compute the pulse time
//		{
//			Timer3B_deltaMatchValue = Timer3B_MatchValue - TimerValueGet(TIMER3_BASE, TIMER_B);	// compute the pulse time in sysClocks
//			Timer3B_deltaMatchValue &= (1 << 24) - 1;		// ignore the sign bit
//			rxChannel_RAW[3] = Timer3B_deltaMatchValue / 80;		// compute the pulse time in us
//			HWREG(TIMER3_BASE + TIMER_O_CTL) &= ~(0x0C00);	// capture positive edges
//			Timer3B_MatchValue = 0;
//		}
//		else
//		{
//			Timer3B_MatchValue = TimerValueGet(TIMER3_BASE, TIMER_B);
//			HWREG(TIMER3_BASE + TIMER_O_CTL) |= 0x0C00;		// capture negative edges
//		}
//	}
}

void WTimer1AIntHandler(void)		// SONAR Receiver
{
	if (HWREG(WTIMER1_BASE + TIMER_O_MIS) & 0x0004)			// CAEMIS = 1, TimerA capture event interrupt occurs
	{
		TimerIntClear(WTIMER1_BASE, TIMER_CAPA_EVENT);
		if (WTimer1A_MatchValue != 0)							//captured a negative edge and compute the pulse time
		{
			WTimer1A_deltaMatchValue = WTimer1A_MatchValue - TimerValueGet(WTIMER1_BASE, TIMER_A);	// compute the pulse time in sysClocks
			WTimer1A_deltaMatchValue &= (1 << 24) - 1;		// ignore the sign bit
			Altitude_RAW = WTimer1A_deltaMatchValue * 0.0002155172413f;								// compute distance in cm   1 / 58 / 80 = 0.0002155172413
			rxChannel_RAW[4] = (uint32_t) Altitude_RAW;
			HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(0x0004);	// capture positive edges
			WTimer1A_MatchValue = 0;
		}
		else
		{
			WTimer1A_MatchValue = TimerValueGet(WTIMER1_BASE, TIMER_A);
			HWREG(WTIMER1_BASE + TIMER_O_CTL) |= 0x0004;		// capture negative edges
		}
	}
}

void WTimer1BIntHandler(void)
{
//	if (HWREG(WTIMER1_BASE + TIMER_O_MIS) & 0x0400)			// CBEMIS = 1, TimerA capture event interrupt occurs
//	{
//		TimerIntClear(WTIMER1_BASE, TIMER_CAPB_EVENT);
//		if (WTimer1B_MatchValue != 0)							//captured a negative edge and compute the pulse time
//		{
//			WTimer1B_deltaMatchValue = WTimer1B_MatchValue - TimerValueGet(WTIMER1_BASE, TIMER_B);	// compute the pulse time in sysClocks
//			WTimer1B_deltaMatchValue &= (1 << 24) - 1;		// ignore the sign bit
//			rxChannel_RAW[5] = WTimer1B_deltaMatchValue / 80;		// compute the pulse time in us
//			HWREG(WTIMER1_BASE + TIMER_O_CTL) &= ~(0x0C00);	// capture positive edges
//			WTimer1B_MatchValue = 0;
//		}
//		else
//		{
//			WTimer1B_MatchValue = TimerValueGet(WTIMER1_BASE, TIMER_B);
//			HWREG(WTIMER1_BASE + TIMER_O_CTL) |= 0x0C00;		// capture negative edges
//		}
//	}
}

bool PPMReveiver_Transform()
{
	if (rxChannel_RAW[0] > 800 && rxChannel_RAW[0] < 2200)	// channel 0
	{
		rxCommand.Roll = rxChannel_RAW[0] - 1500;
		if (rxCommand.Roll < -500) rxCommand.Roll = -500;
		if (rxCommand.Roll > 500) rxCommand.Roll = 500;
	}
	else
		return false;

	if (rxChannel_RAW[1] > 800 && rxChannel_RAW[1] < 2200)	// channel 1
	{
		rxCommand.Pitch = rxChannel_RAW[1] - 1500;
		if (rxCommand.Pitch < -500) rxCommand.Pitch = -500;
		if (rxCommand.Pitch > 500) rxCommand.Pitch = 500;
	}
	else
		return false;

	if (rxChannel_RAW[2] > 800 && rxChannel_RAW[2] < 2200)  //channel 2
	{
		rxCommand.Throttle = rxChannel_RAW[2] - 1000;
		if (rxCommand.Throttle < 0) rxCommand.Throttle = 0;
		if (rxCommand.Throttle > 1000) rxCommand.Throttle = 1000;
	}
	else
		return false;

	if (rxChannel_RAW[3] > 800 && rxChannel_RAW[3] < 2200)  //channel 3
	{
		rxCommand.Yaw = rxChannel_RAW[3] - 1500;
		if (rxCommand.Yaw < -500) rxCommand.Yaw = -500;
		if (rxCommand.Yaw > 500) rxCommand.Yaw = 500;
	}
	else
		return false;

	if (rxChannel_RAW[5] > 800 && rxChannel_RAW[5] < 2200)  //channel 5
	{
		if (rxChannel_RAW[5] < 1500) rxCommand.SW1 = false; else rxCommand.SW1 = true;
	}
	else
		return false;

	return true;	// successful
}

void PPMReceiver_Init()
{
	//PB0 - PB3: T2CCP0 T2CCP1 T3CCP0 T3CCP1
	// PC6 PC7 : WT1CCP0 WT1CCP1

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

//    //************* configure TIMER2
//    GPIOPinConfigure(GPIO_PB0_T2CCP0);
//    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);
//    GPIOPinConfigure(GPIO_PB1_T2CCP1);
//    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_1);
//
//    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//
//    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
//    TimerControlEvent(TIMER2_BASE, TIMER_BOTH, TIMER_EVENT_NEG_EDGE);	// positive edges trigger event
//
//    TimerPrescaleSet(TIMER2_BASE, TIMER_BOTH, 0xFF);					// set the prescale value(8 most significant bits)
//    TimerLoadSet(TIMER2_BASE, TIMER_BOTH, 0xFFFF);
//
//    // enable processor interrupts
//    IntMasterEnable();
//    // Configure the Timer2A interrupt for capture match
//    TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);
//    TimerIntEnable(TIMER2_BASE, TIMER_CAPB_EVENT);
//    // enable Timer2A interrupt
//    IntEnable(INT_TIMER2A);
//    IntEnable(INT_TIMER2B);
//
//    //************* configure TIMER3
//    GPIOPinConfigure(GPIO_PB2_T3CCP0);
//    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);
//    GPIOPinConfigure(GPIO_PB3_T3CCP1);
//    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_3);
//
//    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
//
//    TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
//    TimerControlEvent(TIMER3_BASE, TIMER_BOTH, TIMER_EVENT_NEG_EDGE);	// positive edges trigger event
//
//    TimerPrescaleSet(TIMER3_BASE, TIMER_BOTH, 0xFF);					// set the prescale value(8 most significant bits)
//    TimerLoadSet(TIMER3_BASE, TIMER_BOTH, 0xFFFF);
//
//    // Configure the Timer3A interrupt for capture match
//    TimerIntEnable(TIMER3_BASE, TIMER_CAPA_EVENT);
//    TimerIntEnable(TIMER3_BASE, TIMER_CAPB_EVENT);
//    // enable Timer3A interrupt
//    IntEnable(INT_TIMER3A);
//    IntEnable(INT_TIMER3B);

    //************* configure WTIMER1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //************* configure WTIMER1
    GPIOPinConfigure(GPIO_PC6_WT1CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PC7_WT1CCP1);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_7);

    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
    TimerControlEvent(WTIMER1_BASE, TIMER_BOTH, TIMER_EVENT_NEG_EDGE);	// positive edges trigger event

    TimerLoadSet(WTIMER1_BASE, TIMER_BOTH, 0xFFFFFFFF);	// 64-bit wtimer split into two 32-bit timer

    // enable processor interrupts
    IntMasterEnable();
    // Configure the Timer2A interrupt for capture match
    TimerIntEnable(WTIMER1_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(WTIMER1_BASE, TIMER_CAPB_EVENT);
    // enable Timer2A interrupt
    IntEnable(INT_WTIMER1A);
    IntEnable(INT_WTIMER1B);


//    TimerEnable(TIMER2_BASE, TIMER_BOTH);
//    TimerEnable(TIMER3_BASE, TIMER_BOTH);
    TimerEnable(WTIMER1_BASE, TIMER_BOTH);
}
