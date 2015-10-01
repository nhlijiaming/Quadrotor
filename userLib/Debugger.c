/*
 * Debugger.c
 *
 *  Created on: 2014Äê7ÔÂ23ÈÕ
 *      Author: Ljm
 */

#include "Debugger.h"

uint8_t Debugger_receiveData, Debugger_lastData;
uint8_t Debugger_RxFIFO[50];
uint8_t Debugger_RxFIFOCount = 0;

bool Debugger_SendPIDDataFlag;
bool Debugger_ParserFlag;

uint8_t Debugger_TxFIFO[50];

uint8_t UARTTxFIFO[TXFIFO_LENGTH+2];
uint8_t UARTTxFIFO_Head, UARTTxFIFO_Tail;

#ifdef USE_UART0
#define UART_BASE UART0_BASE
#endif

#ifdef USE_UART1
#define UART_BASE UART1_BASE
#endif

void UARTIntHandler(void)
{
	//if (UARTCharsAvail(UART_BASE))
	if (!(HWREG(UART_BASE + 0x00000018) & 0x00000010))
	{
		//UARTIntClear(UART_BASE, UART_INT_RX);
		HWREG(UART_BASE + 0x00000044) = UART_INT_RX;
		//Debugger_receiveData = UARTCharGetNonBlocking(UART_BASE);
		Debugger_receiveData = HWREG(UART_BASE + 0x00000000);
		//if (Debugger_receiveData == -1) return;
		if (Debugger_lastData == 0xAA && Debugger_receiveData == 0xAF)
		{
			Debugger_RxFIFOCount = 2;
			Debugger_RxFIFO[0] = 0xAA;
			Debugger_RxFIFO[1] = 0xAF;
		}
		else
		{
			Debugger_RxFIFO[Debugger_RxFIFOCount ++] = Debugger_receiveData;
			Debugger_RxFIFO[Debugger_RxFIFOCount]    = 0;
			if (Debugger_RxFIFOCount > 40) Debugger_RxFIFOCount = 0;
			if (Debugger_RxFIFOCount > 3 && Debugger_RxFIFOCount == (Debugger_RxFIFO[3] + 4 + 1))
				Debugger_ParserFlag = true;
		}
		Debugger_lastData = Debugger_receiveData;
	}

//	//if (!UARTBusy(UART_BASE))
//	if ( !(HWREG(UART_BASE + 0x00000018) & 0x00000008) )
//	{
//		//UARTIntClear(UART_BASE, UART_INT_TX);
//		HWREG(UART_BASE + 0x00000044) = UART_INT_TX;
//		if (UARTTxFIFO_Tail != UARTTxFIFO_Head)
//		{
//			//IntDisable(INT_UART1);
//			//HWREG(0xE000E180) = 1 << (INT_UART1 - 16);
//			HWREG(0xE000E180) = 1 << 6;
//			//r = UARTCharPutNonBlocking(UART_BASE, UARTTxFIFO[ UARTTxFIFO_Tail++ ]);
//			r = !(HWREG(UART_BASE + 0x00000018) & 0x00000020);
//			if (r)
//			{
//				HWREG(UART_BASE + 0x00000000) = UARTTxFIFO[ UARTTxFIFO_Tail++ ];
//				if (UARTTxFIFO_Tail > TXFIFO_LENGTH) UARTTxFIFO_Tail = 0;
//			}
//			else
//			{
//				UARTTxFIFO_Tail --;
//			}
//			//IntEnable(INT_UART1);
//			//HWREG(0xE000E100) = 1 << (INT_UART1 - 16);
//			HWREG(0xE000E100) = 1 << 6;
//		}
//	}
}

void UARTTxFIFOIn(uint8_t data)
{
	if (UARTTxFIFO_Head+1 == UARTTxFIFO_Tail) return ;
	UARTTxFIFO[ UARTTxFIFO_Head ] = data;
	UARTTxFIFO_Head ++;
	if (UARTTxFIFO_Head > TXFIFO_LENGTH) UARTTxFIFO_Head = 0;
}

void Debugger_SendRCData()
{
	int count;
	int sum;
	int i;
	int temp = 1500;
	count = 0;

	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x03;
	Debugger_TxFIFO[count++]=0;
	Debugger_TxFIFO[count++]=BYTE1(rxChannel_RAW[2]);	// throttle
	Debugger_TxFIFO[count++]=BYTE0(rxChannel_RAW[2]);
	Debugger_TxFIFO[count++]=BYTE1(rxChannel_RAW[3]);	// yaw
	Debugger_TxFIFO[count++]=BYTE0(rxChannel_RAW[3]);
	Debugger_TxFIFO[count++]=BYTE1(rxChannel_RAW[0]);	// roll
	Debugger_TxFIFO[count++]=BYTE0(rxChannel_RAW[0]);
	Debugger_TxFIFO[count++]=BYTE1(rxChannel_RAW[1]);	// pitch
	Debugger_TxFIFO[count++]=BYTE0(rxChannel_RAW[1]);
	Debugger_TxFIFO[count++]=BYTE1(rxChannel_RAW[4]);	// altitude set
	Debugger_TxFIFO[count++]=BYTE0(rxChannel_RAW[4]);
	Debugger_TxFIFO[count++]=BYTE1(rxChannel_RAW[5]);	// aux channel
	Debugger_TxFIFO[count++]=BYTE0(rxChannel_RAW[5]);
	Debugger_TxFIFO[count++]=BYTE1(AltitudeNotAvailableCount);
	Debugger_TxFIFO[count++]=BYTE0(AltitudeNotAvailableCount);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = 1000 + CPUUsage;
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_SendStatusData()
{
	int count;
	int sum;
	int i;
	int temp = 1500;
	count = 0;

	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x01;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (Att.Roll*100);			// roll
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Att.Pitch*100);			// pitch
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Att.Yaw*100);				// yaw
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (sonarVario*100);			// altitude
	Debugger_TxFIFO[count++]=BYTE3(temp);
	Debugger_TxFIFO[count++]=BYTE2(temp);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	if (!armed) temp = (int) (0xA0); else temp = (int) (0xA1);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_SendSensorData()
{
	int count;
	int sum;
	int i;
	int temp = 1500;
	count = 0;

	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x02;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (Accel.X);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Accel.Y);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Accel.Z);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Gyro.X);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Gyro.Y);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Gyro.Z);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Mag.X);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Mag.Y);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Mag.Z);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
	//Debugger_SendSeriesData(6,atan2((double)Mag.Y, (double)Mag.X)*(180/3.1415926)+180 );
}

void Debugger_SendVoltageData()
{
	int count;
	int sum;
	int i;
	int temp = 1500;
	count = 0;

	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x05;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (370);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (1080);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (1320);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_SendMotorData()
{
	int count;
	int sum;
	int i;
	int temp = 1500;
	count = 0;

	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x06;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (Motor[0]);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Motor[1]);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Motor[2]);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Motor[3]);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0x0123);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0x4567);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0x89AB);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0xCDEF);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_SendPIDData()
{
	int count;
	int sum;
	int i;
	int temp = 1500;

	// PID1
	count = 0;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x10;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (PID_Roll.P*100);					// Roll
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Roll.I*1000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Roll.D*100);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Pitch.P*100);					// Pitch
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Pitch.I*1000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Pitch.D*100);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Yaw.P*100);					// Yaw
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Yaw.I*1000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Yaw.D*100);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);

	// PID2
	count = 0;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x11;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (PID_Altitude.P*100);					// altitude
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Altitude.I*10000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Altitude.D*1000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_PosX.P*100);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_PosX.I*10000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_PosX.D*1000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);

	// PID5
	count = 0;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x14;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (Att_Offset.Roll*1000);					// attitude offset
	if (temp < 0) temp = 10000 - temp;
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Att_Offset.Pitch*1000);
	if (temp < 0) temp = 10000 - temp;
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);

	// PID6
	count = 0;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x15;
	Debugger_TxFIFO[count++]=0;
	temp = (int) (PID_Attenuation.P*100);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Attenuation.I*10000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (PID_Attenuation.D*1000);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (0);
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);

	Debugger_SendAttOffset();
}

void Debugger_SendAttOffset()
{
	int count;
	int sum;
	int i;
	int temp = 1500;
	count = 0;

	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0xAA;
	Debugger_TxFIFO[count++]=0x16;
	Debugger_TxFIFO[count++]=0;

	temp = (int) (Att_Offset.Roll*1000);					// Roll
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);
	temp = (int) (Att_Offset.Pitch*1000);				// Pitch
	Debugger_TxFIFO[count++]=BYTE1(temp);
	Debugger_TxFIFO[count++]=BYTE0(temp);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_SendCheck(uint16_t check)
{
	uint8_t sum;
	uint8_t i;
	Debugger_TxFIFO[0] = 0xAA;
	Debugger_TxFIFO[1] = 0xAA;
	Debugger_TxFIFO[2] = 0xF0;
	Debugger_TxFIFO[3] = 3;
	Debugger_TxFIFO[4] = 0xBA;

	Debugger_TxFIFO[5] = BYTE1(check);
	Debugger_TxFIFO[6] = BYTE0(check);

	sum = 0;
	for( i = 0 ; i < 7 ; i ++)
		sum += Debugger_TxFIFO[i];
	Debugger_TxFIFO[7] = (uint8_t) (sum);

	for(i = 0 ; i <= 7 ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_SendSeriesData(uint8_t n, float data)
{
	int count;
	int sum;
	int i;
	count = 0;
	Debugger_TxFIFO[count++] = 0xAA;
	Debugger_TxFIFO[count++] = 0xAA;
	Debugger_TxFIFO[count++] = 0xF0 + n;
	Debugger_TxFIFO[count++] = 0;

	Debugger_TxFIFO[count++]=BYTE3(data);
	Debugger_TxFIFO[count++]=BYTE2(data);
	Debugger_TxFIFO[count++]=BYTE1(data);
	Debugger_TxFIFO[count++]=BYTE0(data);

	Debugger_TxFIFO[3] = count-4;

	sum = 0;
	for(i = 0 ; i < count ; i ++)
		sum += Debugger_TxFIFO[i];

	Debugger_TxFIFO[count++] = (uint8_t) (sum);

	for(i = 0 ; i < count ; i ++)
		UARTTxFIFOIn(Debugger_TxFIFO[i]);
	UARTIntHandler();
}

void Debugger_Parser(uint8_t n)
{
	uint8_t sum = 0;
	uint8_t i;

	for (i = 0 ; i < n-1 ; i ++)
		sum += Debugger_RxFIFO[i];
	if ( !( sum == Debugger_RxFIFO[ n-1 ] ) ) return ; // check sum
	if( !( Debugger_RxFIFO[0] == 0xAA && Debugger_RxFIFO[1] == 0xAF ) ) return ;		// check header

	if( Debugger_RxFIFO[2] == 0x01 )		//COMMAND
	{
		if( Debugger_RxFIFO[4] == 0x01)
		{
		    if (abs(Att.Pitch)>10.0 || abs(Att.Roll)>10.0)
		    {
		        Att_Offset.Roll = 0;
		        Att_Offset.Pitch = 0;
		    	Debugger_SendAttOffset();
		    	EEPROM_Write();
		    }
		    else
		    {
		    	Att_CalibrationMode = true;
		    }
		}
		if( Debugger_RxFIFO[4] == 0x02)
#ifdef USE_MPU9250_DMP
			run_self_test();
#else
		MPU6050_GyroCalibration();
		MadgwickAHRSInit();
#endif

		//		if( Debugger_RxFIFO[4] == 0x03)
		//		{
		//			MPU6050_CalOff_Acc();
		//			MPU6050_CalOff_Gyr();
		//		}
		if( Debugger_RxFIFO[4] == 0x04)		// calibration mag
		{
			MPU6050_MagCalibration();
		}
	}

	if( Debugger_RxFIFO[2] == 0x02 )		// send back pid parameters
	{
		if( Debugger_RxFIFO[4] == 0x01)
			Debugger_SendPIDDataFlag = true;
//		if( Debugger_RxFIFO[4] == 0x02)
//			;
	}

	if( Debugger_RxFIFO[2] == 0x10 )								//PID1
	{
		PID_Roll.P  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 4] << 8) | Debugger_RxFIFO[ 5])) / 100;
		PID_Roll.I  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 6] << 8) | Debugger_RxFIFO[ 7])) / 1000;
		PID_Roll.D  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 8] << 8) | Debugger_RxFIFO[ 9])) / 100;
		PID_Pitch.P = 1.0 * ((uint16_t) ((Debugger_RxFIFO[10] << 8) | Debugger_RxFIFO[11])) / 100;
		PID_Pitch.I = 1.0 * ((uint16_t) ((Debugger_RxFIFO[12] << 8) | Debugger_RxFIFO[13])) / 1000;
		PID_Pitch.D = 1.0 * ((uint16_t) ((Debugger_RxFIFO[14] << 8) | Debugger_RxFIFO[15])) / 100;
		PID_Yaw.P   = 1.0 * ((uint16_t) ((Debugger_RxFIFO[16] << 8) | Debugger_RxFIFO[17])) / 100;
		PID_Yaw.I   = 1.0 * ((uint16_t) ((Debugger_RxFIFO[18] << 8) | Debugger_RxFIFO[19])) / 1000;
		PID_Yaw.D   = 1.0 * ((uint16_t) ((Debugger_RxFIFO[20] << 8) | Debugger_RxFIFO[21])) / 100;
		Debugger_SendCheck( sum );

		EEPROM_Write();
		PID_CleariTerm();
		PID_Clear();
	}

	if( Debugger_RxFIFO[2] == 0x11 )								//PID2
	{
		PID_Altitude.P  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 4] << 8) | Debugger_RxFIFO[ 5])) / 100;
		PID_Altitude.I  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 6] << 8) | Debugger_RxFIFO[ 7])) / 10000;
		PID_Altitude.D  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 8] << 8) | Debugger_RxFIFO[ 9])) / 1000;
		PID_PosX.P  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[10] << 8) | Debugger_RxFIFO[11])) / 100;
		PID_PosX.I  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[12] << 8) | Debugger_RxFIFO[13])) / 10000;
		PID_PosX.D  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[14] << 8) | Debugger_RxFIFO[15])) / 1000;
		PID_PosY.P  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[10] << 8) | Debugger_RxFIFO[11])) / 100;
		PID_PosY.I  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[12] << 8) | Debugger_RxFIFO[13])) / 10000;
		PID_PosY.D  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[14] << 8) | Debugger_RxFIFO[15])) / 1000;
		Debugger_SendCheck( sum );

		EEPROM_Write();
		PID_CleariTerm();
		PID_Clear();
	}

	if( Debugger_RxFIFO[2] == 0x12 )								//PID3
	{
		Debugger_SendCheck( sum );
	}

	if( Debugger_RxFIFO[2] == 0x13 )								//PID4
	{
		Debugger_SendCheck( sum );
	}

	if( Debugger_RxFIFO[2] == 0x14 )								//PID5: PID8-10
	{
		Att_Offset.Roll = (float)((uint16_t) ((Debugger_RxFIFO[ 4] << 8) | Debugger_RxFIFO[ 5])) / 1000;
		Att_Offset.Pitch = (float)((uint16_t) ((Debugger_RxFIFO[ 6] << 8) | Debugger_RxFIFO[ 7])) / 1000;
		if (Att_Offset.Roll > 10.0) Att_Offset.Roll = -(Att_Offset.Roll - 10.0);
		if (Att_Offset.Pitch > 10.0) Att_Offset.Pitch = -(Att_Offset.Pitch - 10.0);
		Debugger_SendCheck( sum );
		EEPROM_Write();
	}

	if( Debugger_RxFIFO[2] == 0x15 )								//PID6: PID11-12
	{
		PID_Attenuation.P = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 4] << 8) | Debugger_RxFIFO[ 5])) / 100;
		PID_Attenuation.I  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 6] << 8) | Debugger_RxFIFO[ 7])) / 10000;
		PID_Attenuation.D  = 1.0 * ((uint16_t) ((Debugger_RxFIFO[ 8] << 8) | Debugger_RxFIFO[ 9])) / 1000;

		EEPROM_Write();
		Debugger_SendCheck( sum );
	}
}
