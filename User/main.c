#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

int16_t AX, AY, AZ, GX, GY, GZ;
uint8_t TimerErrorFlag;
uint16_t TimerCount;

float AngleAcc;
float AngleGyro;
float Angle;

uint8_t KeyNum, RunFlag;

int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;

float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

PID_t AnglePID = {
	.Kp = 5,
	.Ki = 0.1,
	.Kd = 5,
	
	.OutMax = 100,
	.OutMin = -100,
	
	.OutOffset = 3,
	
	.ErrorIntMax = 600,
	.ErrorIntMin = -600,
};

PID_t SpeedPID = {
	.Kp = 2,
	.Ki = 0.05,
	.Kd = 0,
	
	.OutMax = 20,
	.OutMin = -20,
	
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
};

PID_t TurnPID = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 50,
	.OutMin = -50,
	
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

int main(void)
{
	OLED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	
	Timer_Init();
	
	while (1)
	{
		if (RunFlag) {LED_ON();} else {LED_OFF();}
		
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			if (RunFlag == 0)
			{
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				RunFlag = 1;
			}
			else
			{
				RunFlag = 0;
			}
		}
		
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);
		OLED_Printf(56, 56, OLED_6X8, "Offset:%02.0f", AnglePID.OutOffset);
		OLED_Printf(50, 0, OLED_6X8, "Speed");
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);
		OLED_Printf(50, 48, OLED_6X8, "%+05.0f", SpeedPID.Out);
		OLED_Printf(88, 0, OLED_6X8, "Turn");
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);
		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);
		OLED_Printf(88, 48, OLED_6X8, "%+05.0f", TurnPID.Out);
		OLED_Update();
		
		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
			}
			else if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "AngleKp") == 0)
				{
					AnglePID.Kp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					AnglePID.Ki = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					AnglePID.Kd = atof(Value);
				}
				else if (strcmp(Name, "SpeedKp") == 0)
				{
					SpeedPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					SpeedPID.Ki = atof(Value);
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{
					SpeedPID.Kd = atof(Value);
				}
				else if (strcmp(Name, "TurnKp") == 0)
				{
					TurnPID.Kp = atof(Value);
				}
				else if (strcmp(Name, "TurnKi") == 0)
				{
					TurnPID.Ki = atof(Value);
				}
				else if (strcmp(Name, "TurnKd") == 0)
				{
					TurnPID.Kd = atof(Value);
				}
				else if (strcmp(Name, "Offset") == 0)
				{
					AnglePID.OutOffset = atof(Value);
				}
			}
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				SpeedPID.Target = LV / 25.0;
				TurnPID.Target = RH / 25.0;
			}
			
			BlueSerial_RxFlag = 0;
		}
		
//		BlueSerial_Printf("[plot,%f,%f,%f]", AnglePID.ErrorInt, SpeedPID.ErrorInt, TurnPID.ErrorInt);
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		Key_Tick();
		
		Count0 ++;
		if (Count0 >= 10)
		{
			Count0 = 0;
			
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			GY -= 16;

			AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
			
			AngleAcc += 0.5;
			
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			float Alpha = 0.01;
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro ;
			
			if (Angle > 50 || Angle < -50)
			{
				RunFlag = 0;
			}
			
			if (RunFlag)
			{
				AnglePID.Actual = Angle;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;
				
				LeftPWM = AvePWM + DifPWM / 2;
				RightPWM = AvePWM - DifPWM / 2;
				
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
				
				Motor_SetPWM(1, LeftPWM);
				Motor_SetPWM(2, RightPWM);
			}
			else
			{
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}
		
		Count1 ++;
		if (Count1 >= 50)
		{
			Count1 = 0;
			
			LeftSpeed = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;
			
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
			DifSpeed = LeftSpeed - RightSpeed;
			
			if (RunFlag)
			{
				SpeedPID.Actual = AveSpeed;
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;
				
				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
			}
		}
		
		
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}
