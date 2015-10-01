#include "userLib/board.h"

Att_Data Att, Att_Offset;
struct
{
	int32_t X, Y, Z;
	uint8_t CountSum;
}GyroSum;

bool Update_1000Hz;
bool Update_500Hz;
bool Update_200Hz;
bool Update_100Hz;
bool Update_50Hz;
bool Update_10Hz;
bool Update_5Hz;
bool Update_1Hz;

uint64_t deltaTime1000Hz, executionTime1000Hz, previousTime1000Hz;
uint64_t deltaTime500Hz, executionTime500Hz, previousTime500Hz;
uint64_t deltaTime200Hz, executionTime200Hz, previousTime200Hz;
uint64_t deltaTime100Hz, executionTime100Hz, previousTime100Hz;
uint64_t deltaTime50Hz, executionTime50Hz, previousTime50Hz;
uint64_t deltaTime10Hz, executionTime10Hz, previousTime10Hz;
uint64_t deltaTime5Hz, executionTime5Hz, previousTime5Hz;
uint64_t deltaTime1Hz, executionTime1Hz, previousTime1Hz;
uint32_t sumTime, CPUUsage;

uint64_t beginTime;

uint32_t Throttle;
float Altitude;
float Altitude_RAW;
uint32_t AltitudeThrottle;
uint32_t AltitudeNotAvailableCount;
int32_t sonarVario;

uint32_t Motor[4];

uint8_t stop;

bool armed = false;
bool Att_CalibrationMode = false;
bool AltitudeHold = true;
bool PositionHold = false;
bool launching = false;
float AltitudeSet = 0.0;
float AltitudeSetRaw = 0.0;
float setPosX = 0.0;
float setPosY = 0.0;
float ready_setPosX = 0.0;
float ready_setPosY = 0.0;
uint16_t ModeCount = 0;
uint8_t ModeFourActivator = 0;

float setRoll, setPitch;


void Timer4AIntHandler(void)
{
	static int count=0;
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	count ++;

	Update_1000Hz = true;
	if (count % 2 == 0) Update_500Hz = true;
	if (count % 5 == 0) Update_200Hz = true;
	if (count %10 == 0) Update_100Hz = true;
	if (count %20 == 0) Update_50Hz = true;
	if (count %100 == 0) Update_10Hz = true;
	if (count %200 == 0) Update_5Hz = true;
	if (count >= 1000) { Update_1Hz = true; count = 0;}
}

void TickTock_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER4_BASE, TIMER_A, 80000);
	IntMasterEnable();
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER4A);
	TimerEnable(TIMER4_BASE, TIMER_A);
}

void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    //UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTStdioConfig(1, 115200, 80000000);
	UARTFIFODisable(UART1_BASE);

	IntMasterEnable();
//	UARTIntEnable(UART1_BASE, UART_INT_TX | UART_INT_RX);
	UARTIntEnable(UART1_BASE, UART_INT_RX);
	IntEnable(INT_UART1);


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	//UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, 80000000);
//	UARTFIFODisable(UART0_BASE);

	IntMasterEnable();
	UARTIntEnable(UART0_BASE, UART_INT_RX);
	IntEnable(INT_UART0);
}

void Att_Calibration()
{
#define cycles 3000
    static float roll=0.0, pitch=0.0;
    static int32_t acc[3] = {0.0};
    static int i=0;

    i++;
    roll += Att.Roll;
    pitch += Att.Pitch;
    acc[0] += Accel.X;
    acc[1] += Accel.Y;
    acc[2] += Accel.Z;

    if (i==cycles)
    {
    	i = 0;
    	Att_CalibrationMode = false;
    	Att_Offset.Roll = -roll / cycles;
    	Att_Offset.Pitch = -pitch / cycles;
    	Accel_Offset.X = acc[0] / cycles;
    	Accel_Offset.Y = acc[1] / cycles;
    	Accel_Offset.Z = acc[2] / cycles;
    	roll = 0.0;
    	pitch = 0.0;
    	acc[0] = 0.0;
    	acc[1] = 0.0;
    	acc[2] = 0.0;
    	Debugger_SendAttOffset();
    	EEPROM_Write();
		BEEP_On();
		Update_5Hz = false;
		while (!Update_5Hz);
		BEEP_Off();
    }
}

int main(void)
{
#define acc_throttle_cutoff	(2.0f)
#define M_PI 				(3.1415926f)
#define fc_throttle   		(1.0f / (2 * M_PI * acc_throttle_cutoff))
	uint8_t i;
	uint8_t armed_unlocker;
	uint64_t currentTime;
	uint64_t flightTime;
	uint8_t SonarUpdateCount;
	bool s;
	bool landing;
	bool magAvailable;
	bool lastSW1;

	uint32_t ThrottleRAW;
	float ThrottleLPF;
	float dT;
	float dist;

	float k;

	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;

	float roll, pitch, yaw;
	float rollLPF, pitchLPF, yawLPF;

	//set system clock to 80M
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);
	InitConsole();

	// LED init
	Interface_Init();
	OLED_Init();
//	while (1)
//	{
//		i = SW1_Pressed();
//		i = SW2_Pressed();
//		i = SW3_Pressed();
//		i = SW4_Pressed();
//		i = SW5_Pressed();
//		i = SW6_Pressed();
//	}
	LED_Red_On();
	LED_Green_On();
	LED_Blue_On();

	//FPUEnable();
	//FPULazyStackingEnable();
	//FPUStackingEnable();

	EEPROM_Init();
	ClockCounter_Init();
	PPMReceiver_Init();
	PPMEncoder_Init();
	TickTock_Init();
	Sonar_Init();

	MPU9250_Init();
//	MPU9250_Dmp_Init();
	OpticalFlow_Init();
	EEPROM_Read();
	OLED_Print(10,0,"open  +++");
	OLED_Print(10,2,"the   ---");
	OLED_Print(10,4,"light ***");
	OLED_Print(10,6,"first !!!");

	// wait for 2 seconds
	Update_1Hz = false;
	while (!Update_1Hz);
	Update_1Hz = false;
	while (!Update_1Hz);
	// check throttle if it's at minimum
	PPMReveiver_Transform();
	if (rxCommand.Throttle > 50)
	{
		OLED_Print(38,0,"throttle high");
		while (1) ;
	}
	// throttle checked, ready to unarmed
	armed = false;

	LED_Red_Off();
	LED_Green_Off();

#ifdef GYRO_CALIBRATION_ENABLE
	UARTprintf("Gyrometer Calibrating.\n");
    MPU6050_GyroCalibration();
    UARTprintf("Gyrometer Calibration done.\n");
#else
    UARTprintf("Gyrometer Calibration disable.\n");
#endif

    UARTprintf("Working.\n");

    if (!rxControl) AdjustParameter();
    LED_Blue_Off();

    // initizalize flags
    armed_unlocker = 0;
    AltitudeThrottle = 0;
    AltitudeNotAvailableCount = 0;
    SonarUpdateCount = 1;
    ThrottleLPF = 0.0;
    lastSW1 = false;
    landing = false;

    Update_1000Hz = false;
    Update_500Hz = false;
    Update_200Hz = false;
    Update_100Hz = false;
    Update_50Hz = false;
    Update_10Hz = false;
    Update_5Hz = false;
    Update_1Hz = false;

    previousTime1000Hz = ClockCounter_Get();
    previousTime500Hz = ClockCounter_Get();
    previousTime200Hz = ClockCounter_Get();
    previousTime100Hz = ClockCounter_Get();
    previousTime50Hz = ClockCounter_Get();
    previousTime10Hz = ClockCounter_Get();
    previousTime5Hz = ClockCounter_Get();
    previousTime1Hz = ClockCounter_Get();
    // ************ main loop ************
    Sonar_Trig();
	while(1)
	{

		if (Update_1000Hz)
		{
			Update_1000Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime1000Hz    = ClockCounter_tomicro(previousTime1000Hz - currentTime);
			previousTime1000Hz = currentTime;

			executionTime1000Hz =  ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime1000Hz;
		}

		if (Update_500Hz && !Update_100Hz)
		{
			Update_500Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime500Hz    = ClockCounter_tomicro(previousTime500Hz - currentTime);
			previousTime500Hz = currentTime;

//			MPU9250_GetDMPData();

			MPU9250_GetAccel();
			MPU9250_GetGyro();
			magAvailable = MPU9250_GetMag();
			//MPU9250_MagAdjust();

			ax = Accel.X;
			ay = Accel.Y;
			az = Accel.Z;

			gx = Gyro.X * MPU9250G_2000dps * RadtoDeg;
			gy = Gyro.Y * MPU9250G_2000dps * RadtoDeg;
			gz = Gyro.Z * MPU9250G_2000dps * RadtoDeg;
			GyroSum.X += Gyro.X;
			GyroSum.Y += Gyro.Y;
			GyroSum.Z += Gyro.Z;
			GyroSum.CountSum ++;

			mx = Mag.X;
			my = Mag.X;
			mz = Mag.X;
			MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
			if (Att_CalibrationMode)
				Att_Calibration();
			else
			{
				Att.Pitch += Att_Offset.Pitch;
				Att.Roll += Att_Offset.Roll;
			}

			if (abs(Att.Pitch)>70 || abs(Att.Roll)>70) armed = false;

			AccZ_Filter(deltaTime500Hz);
			Att_Filter();

			executionTime500Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime500Hz;
		}

		if (Update_200Hz)
		{
			Update_200Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime200Hz    = ClockCounter_tomicro(previousTime200Hz - currentTime);
			previousTime200Hz = currentTime;

			executionTime200Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime200Hz;
		}

		if (Update_100Hz)
		{
			Update_100Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime100Hz    = ClockCounter_tomicro(previousTime100Hz - currentTime);
			previousTime100Hz = currentTime;

			s = PPMReveiver_Transform();
			if (!s)
			{
				rxCommand.Throttle = 0;
				rxCommand.Pitch = 0;
				rxCommand.Roll = 0;
				rxCommand.Yaw = 0;
				rxCommand.SW1 = false;
				rxCommand.SW2 = false;
			}

			// Mode select : Manual or Auto-Altitude
			if (AltitudeHold)
			{
				ThrottleRAW = AltitudeThrottle;
			}
			else
			{
				ThrottleRAW = rxCommand.Throttle * 1;
			}

			if (rxControl) PositionHold = rxCommand.SW1;
			if (PositionHold)
			{
				roll = setRoll;
				pitch = setPitch;
				yaw = 0.0;
			}
			else
			{
				if (rxControl)
				{
					pitch = -rxCommand.Pitch * 0.1;
					roll = rxCommand.Roll * 0.1;
				}
				yaw = 0.0;//-rxCommand.Yaw * 0.1
			}

			dT = deltaTime100Hz * 1e-6;

//			ThrottleLPF = ThrottleLPF + (dT / (fc_throttle + dT)) * (ThrottleRAW - ThrottleLPF); // low pass filter
//			pitchLPF 	= pitchLPF + (dT / (fc_throttle + dT)) * (pitch - pitchLPF);
//			rollLPF 	= rollLPF + (dT / (fc_throttle + dT)) * (roll - rollLPF);
//			yawLPF 	= yawLPF + (dT / (fc_throttle + dT)) * (yaw - yawLPF);
			ThrottleLPF = ThrottleRAW;
			pitchLPF = pitch;
			rollLPF = roll;
			yawLPF = yaw;

//		    Debugger_SendSeriesData(9,pitchLPF);
//		    Debugger_SendSeriesData(10,rollLPF);

			Throttle 	= (uint32_t) ThrottleLPF;

			// Throttle PID Attenuation
			if (Throttle <= 500)
			{
				PID_AttenuationReal.P = 1.0;
				PID_AttenuationReal.I = 1.0;
				PID_AttenuationReal.D = 1.0;
			}
			else
			{
				k = ((float)Throttle - 500.0) / 500.0;
				PID_AttenuationReal.P = 1.0 - PID_Attenuation.P * k;
				PID_AttenuationReal.I = 1.0 - PID_Attenuation.I * k;
				PID_AttenuationReal.D = 1.0 - PID_Attenuation.D * k;
			}

			if (GyroSum.CountSum)
			{
				// update pid
				// **** keep the positive direction of gyro and axisPID are same
				//			axisPID.Pitch = PID_Update(&PID_Pitch, Att.Pitch, -rxCommand.Pitch * 0.1, -gx);
				//			axisPID.Roll =  PID_Update(&PID_Roll,  Att.Roll,   rxCommand.Roll * 0.1,  -gy);
				//			axisPID.Yaw =   PID_Update(&PID_Yaw,   Att.Yaw,   -rxCommand.Yaw * 0.1,   -gz);
				axisPID.Pitch = PID_Update(&PID_Pitch, 						Att.Pitch, pitchLPF, -((float)GyroSum.X/(float)GyroSum.CountSum) * MPU9250G_2000dps * RadtoDeg );
				axisPID.Roll =  PID_Update(&PID_Roll,  						Att.Roll ,  rollLPF,  -((float)GyroSum.Y/(float)GyroSum.CountSum) * MPU9250G_2000dps * RadtoDeg );
				axisPID.Yaw =   PID_Update(&PID_Yaw, constrainf(Att.Yaw, -10.0, 10.0),   yawLPF,   -((float)GyroSum.Z/(float)GyroSum.CountSum) * MPU9250G_2000dps * RadtoDeg );
			}
			GyroSum.X = 0;
			GyroSum.Y = 0;
			GyroSum.Z = 0;
			GyroSum.CountSum = 0;

			// table mix
			//							   Roll, Pitch,   Yaw
            Motor[0] = (uint32_t) PIDMIX( -1.0f,  1.0f, -1.0f );      // Front Right CCW
            Motor[1] = (uint32_t) PIDMIX(  1.0f,  1.0f,  1.0f );      // Front Left  CW
            Motor[2] = (uint32_t) PIDMIX(  1.0f, -1.0f, -1.0f );      // Rear Left   CCW
            Motor[3] = (uint32_t) PIDMIX( -1.0f, -1.0f,  1.0f );      // Rear Right  CW
            if (Motor[0] > MOTOR_LIMITATION) Motor[0] = MOTOR_LIMITATION;
            if (Motor[1] > MOTOR_LIMITATION) Motor[1] = MOTOR_LIMITATION;
            if (Motor[2] > MOTOR_LIMITATION) Motor[2] = MOTOR_LIMITATION;
            if (Motor[3] > MOTOR_LIMITATION) Motor[3] = MOTOR_LIMITATION;

            if ( !armed )
            {
                Motor[0] = 0;
                Motor[1] = 0;
                Motor[2] = 0;
                Motor[3] = 0;
            }

            if ( rxChannel_RAW[2] > 950 && rxChannel_RAW[2] < 1050 )
            {
                Motor[0] = 0;
                Motor[1] = 0;
                Motor[2] = 0;
                Motor[3] = 0;
            }

			// motor update
            PPMEncoder_Update(Motor[0], Motor[1], Motor[2], Motor[3]);

			executionTime100Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime100Hz;
		}

		if (Update_50Hz && !Update_1000Hz  && !Update_100Hz)
		{
			Update_50Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime50Hz    = ClockCounter_tomicro(previousTime50Hz - currentTime);
			previousTime50Hz = currentTime;

			OpticalFlow_GetMotion();
		    Debugger_SendSeriesData(9,OpticalFlow.compensatedx);
		    Debugger_SendSeriesData(10,OpticalFlow.compensatedy);

		    if (PositionHold)
		    {
		    	setRoll = PID_PositionXUpdate(&PID_PosX, setPosX);
		    	setPitch = PID_PositionYUpdate(&PID_PosY, setPosY);
		    }

			SonarUpdateCount --;
			if (!SonarUpdateCount)	// Sonar data fusion every n*20ms
			{
				SonarUpdateCount = 3;
				if (rxChannel_RAW[4]>10 && rxChannel_RAW[4]<300) Altitude = Altitude_RAW; else	AltitudeNotAvailableCount ++;
				Altitude = Sonar_MedianFilter(Altitude);
				Altitude = Sonar_LimitationFilter(Altitude);
				Sonar_Trig();
//				if (AltitudeNotAvailableCount > 10) BEEP_On(); else BEEP_Off();
				PID_AltitudeUpdateSonarFusion((float)Altitude, (float)deltaTime50Hz*0.000001);
			}

			if (rxControl) AltitudeSet = (float)rxCommand.Throttle/5.0;

			// auto-altitude throttle pid update
			if (abs(Att.Pitch)<30.0 && abs(Att.Roll)<30.0)
				AltitudeThrottle = PID_AltitudeUpdate(&PID_Altitude, AltitudeSet, (float)deltaTime50Hz*0.000001);
			else
			{
				PID_AltitudeUpdate(&PID_Altitude, AltitudeSet, (float)deltaTime50Hz*0.000001);
				AltitudeThrottle -= 10;
			}
			AltitudeThrottle = 350 + constrain(AltitudeThrottle, 0, 450);

			executionTime50Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime50Hz;
		}

		if (Update_10Hz && !Update_1000Hz && !Update_100Hz && !Update_50Hz)
		{
			Update_10Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime10Hz    = ClockCounter_tomicro(previousTime10Hz - currentTime);
			previousTime10Hz = currentTime;

			if (Mode == 1)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				if (launching && Altitude > 0.6*AltitudeSet) launching = false;
				if (!launching)
				{
					setPosX = ready_setPosX;
					setPosY = ready_setPosY;
				}
				if (OpticalFlow_Distance(ready_setPosX, ready_setPosY) <= 0.3*PointDis ) landing = true;
				if (flightTime > (20+0.1*PointDis)*1000000) landing = true;
			}

			if (Mode == 2)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				if (flightTime > 10*1000000 && OpticalFlow_Distance(0,0)<40.0) landing = true;
				if (flightTime > 20*1000000) landing = true;
			}

//			if (Mode == 3)
//			{
//				ModeCount ++;
//				if (ModeCount <= 60)
//				{
//					setPosY = - PointDis * ModeCount / 60;
//				}
//				if (ModeCount > 60 && ModeCount <= 160)
//				{
//					ready_setPosX = PointDis * sinf(((float)ModeCount-60.0)*360/100/180*3.1415926);
//					ready_setPosY = - PointDis * cosf(((float)ModeCount-60.0)*360/100/180*3.1415926);
//				}
//				if (ModeCount > 160 && ModeCount <= 220)
//				{
//					setPosY = PointDis - PointDis * ModeCount / 60;
//				}
//				if (ModeCount > 220) landing = true;
//			}

			// fly back
			if (Mode == 3)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				if (launching && Altitude > 0.8*AltitudeSet) launching = false;
				if (!launching)
				{
					setPosX = ready_setPosX;
					setPosY = ready_setPosY;
				}
				if (OpticalFlow_Distance(0, 0) >= 0.7*PointDis ) landing = true;
				if (flightTime > (20+0.1*PointDis)*1000000) landing = true;
			}

			if (Mode == 4)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				if (launching && Altitude > 0.8*AltitudeSet) launching = false;
				if (!launching)
				{
					setPosX = ready_setPosX;
					setPosY = ready_setPosY;
				}
				if (OpticalFlow_Distance(0,0) <= 0.3*PointDis ) landing = true;
				if (flightTime > (20+0.1*PointDis)*1000000) landing = true;
			}

			//square
			if (Mode == 5)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				if (launching && Altitude > 0.8*AltitudeSet) launching = false;
				if (!launching)
				{
					setPosX = 0;
					setPosY = -PointDis;
				}
				if (OpticalFlow_Distance(0,-PointDis) <= 0.3*PointDis ) Mode ++;
				if (flightTime > (20+0.1*PointDis)*1000000) Mode ++;
			}
			if (Mode == 6)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				setPosX = PointDis;
				setPosY = -PointDis;
				if (OpticalFlow_Distance(PointDis,-PointDis) <= 0.3*PointDis ) Mode ++;
				if (flightTime > (20+0.1*PointDis)*1000000) Mode ++;
			}
			if (Mode == 7)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				setPosX = PointDis;
				setPosY = 0;
				if (OpticalFlow_Distance(PointDis,0) <= 0.3*PointDis ) Mode ++;
				if (flightTime > (20+0.1*PointDis)*1000000) Mode ++;
			}
			if (Mode == 8)
			{
				flightTime = ClockCounter_tomicro(beginTime - ClockCounter_Get());
				setPosX = 0;
				setPosY = 0;
				if (OpticalFlow_Distance(0,0) <= 0.3*PointDis ) landing = true;
				if (flightTime > (20+0.1*PointDis)*1000000) landing = true;
			}

			if (landing) AltitudeSet = max(AltitudeSet-0.8, 0.0);
			if (landing && Altitude < 23)
			{
				armed = false;
				landing = false;
				if (Mode == 3)
				{
					ModeFourActivator = 1;
				}
				Mode = 0;
			}
			if (ModeFourActivator > 0) ModeFourActivator ++;
			if (ModeFourActivator > 20)
			{
				ModeFourActivator = 0;
				Mode = 4;
				setPosX = ready_setPosX;
				setPosY = ready_setPosY;
				ready_setPosX = 0.0;
				ready_setPosY = 0.0;
				armed = true;
				AltitudeHold = true;
				PositionHold = true;
				AltitudeSet = AltitudeSetRaw;
				launching = true;
				beginTime = ClockCounter_Get();
			}

			if (debug || !armed)
			{
				Debugger_SendStatusData();
				Debugger_SendMotorData();
			}

			executionTime10Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime10Hz;
		}

		if (Update_5Hz && !Update_1000Hz  && !Update_100Hz && !Update_50Hz)
		{
			Update_5Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime5Hz    = ClockCounter_tomicro(previousTime5Hz - currentTime);
			previousTime5Hz = currentTime;

			if (Debugger_SendPIDDataFlag)
			{
				Debugger_SendPIDDataFlag = false;
				Debugger_SendPIDData();
			}

			if (debug || !armed)
			{
				Debugger_SendRCData();
				//Debugger_SendVoltageData();	//not avalible ?
				Debugger_SendSensorData();
			}

			if (Att_CalibrationMode)
				LED_Green_Off();
			else
			{
				LED_Green_Toggle();
				if (armed)
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, (~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3)>>2)&GPIO_PIN_1);
				else
					LED_Red_Off();
			}

			executionTime5Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime50Hz;
		}

		if (Update_1Hz)
		{
			Update_1Hz = false;
			currentTime     = ClockCounter_Get();
			deltaTime1Hz    = ClockCounter_tomicro(previousTime1Hz - currentTime);
			previousTime1Hz = currentTime;

			if (rxCommand.Throttle < 50 && rxCommand.Pitch < -450 && rxCommand.Yaw < -450 && rxCommand.Roll > 450)
			{
				armed_unlocker ++;
				if (armed_unlocker == 2)				// armed or unarmed
				{
					armed = !armed;
					if (armed)							// reset yaw to zero
					{
						BEEP_On();
						Update_5Hz = false;
						while (!Update_5Hz);
						BEEP_Off();
						Update_10Hz = false;
						while (!Update_10Hz);
						BEEP_On();
						Update_5Hz = false;
						while (!Update_5Hz);
						BEEP_Off();

						PID_Clear();
//						MPU9250_Dmp_Init();
						MadgwickAHRSInit();
						AltitudeNotAvailableCount = 0;
						OpticalFlow.x = 0.0;
						OpticalFlow.y = 0.0;
						OpticalFlow.compensatedx = 0.0;
						OpticalFlow.compensatedy = 0.0;
					}
					else
					{
						BEEP_On();
						Update_5Hz = false;
						while (!Update_5Hz);
						BEEP_Off();
						Update_10Hz = false;
						while (!Update_10Hz);
						BEEP_On();
						Update_5Hz = false;
						while (!Update_5Hz);
						BEEP_Off();
						Update_10Hz = false;
						while (!Update_10Hz);
						BEEP_On();
						Update_5Hz = false;
						while (!Update_5Hz);
						BEEP_Off();
					}
				}
			}
			else
				armed_unlocker = 0;

			executionTime1Hz = ClockCounter_tomicro(currentTime - ClockCounter_Get());
			sumTime += executionTime1Hz;
			CPUUsage = sumTime / 1000;
			sumTime = 0;
		}

		if ((debug || !armed) && Debugger_ParserFlag)
		{
			Debugger_ParserFlag = false;
			Debugger_Parser( Debugger_RxFIFOCount );
			for(i=0 ; i<40 ; i++) Debugger_RxFIFO[i] = 0;
		}

		// send UART data while it's free
		//if (!UARTBusy(UART_BASE))
		if ( !(HWREG(UART1_BASE + 0x00000018) & 0x00000008) && UARTTxFIFO_Tail != UARTTxFIFO_Head)
		{
			HWREG(UART1_BASE + 0x00000000) = UARTTxFIFO[ UARTTxFIFO_Tail++ ];
			if (UARTTxFIFO_Tail > TXFIFO_LENGTH) UARTTxFIFO_Tail = 0;
		}
	}
}
