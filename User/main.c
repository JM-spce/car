#include "stm32f10x.h" // Device header
#include "BlueSerial.h"
#include "Delay.h"
#include "Encoder.h"
#include "Key.h"
#include "LED.h"
#include "MPU6050.h"
#include "Motor.h"
#include "OLED.h"
#include "PID.h"
#include "Serial.h"
#include "Timer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

uint16_t KeyNum, RunFlag;

uint8_t updata_count;				//while(1)每进入10次，OLED和serial更新一次数据

int16_t AX, AY, AZ, GX, GY, GZ;

int8_t LeftPWM, RightPWM;
int16_t LeftEncoder, RightEncoder;
float LeftSpeed, RightSpeed;
float AveSpeed;


float Angle;                      // 最终偏航角（当前角度-180~180度）
float AngleIntegral = 0;          // 角度积分值
float BaseAngle = 0;              // 基准角度
static float gyro_z_filtered = 0; // 滤波后的GZ陀螺仪数据

PID_t SpeedPID = {
    .Kp = 4.0,
    .Ki = 0.02,
    .Kd = 0.01,

    .OutMax = 100,
    .OutMin = -100,

    .ErrorIntMax = 150,
    .ErrorIntMin = -150,

    .DeadZone = 2.0,
    .DeadZoneEnable = 1
};

PID_t TurnPID = {
    .Kp = 2.0,
    .Ki = 0.01,
    .Kd = 0.1,

    .OutMax = 100,
    .OutMin = -100,

    .ErrorIntMax = 150,
    .ErrorIntMin = -150,

    .DeadZone = 1.0,
    .DeadZoneEnable = 1
};

void Angle_Normalize(void)
{
    // 将角度归一化到[-180, 180]范围
    while (AngleIntegral > 180.0)
        AngleIntegral -= 360.0;
    while (AngleIntegral < -180.0)
        AngleIntegral += 360.0;
    Angle = AngleIntegral;
}

void Angle_Reset(void)
{
    AngleIntegral = 0;
    BaseAngle = 0;
    Angle = 0;
    gyro_z_filtered = 0;
    PID_Init(&TurnPID);
}

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

    // 初始化PID控制器
    PID_Init(&SpeedPID);
    PID_Init(&TurnPID);

    while (1)
    {
        if (RunFlag)
        {
            LED_ON();
        }
        else
        {
            LED_OFF();
        }

        // 处理串口数据
        if (Serial_RxFlag == 1)
        {
            char *Tag = strtok(Serial_RxPacket, ",");
            if (strcmp(Tag, "key") == 0)
            {
                char *KeyName = strtok(NULL, ",");
                char *Action = strtok(NULL, ",");

                if (strcmp(KeyName, "1") == 0 && strcmp(Action, "down") == 0)
                {
                    KeyNum = 1;
                }
                else if (strcmp(KeyName, "2") == 0 && strcmp(Action, "down") == 0)
                {
                    KeyNum = 2;
                }
                else if (strcmp(KeyName, "3") == 0 && strcmp(Action, "down") == 0)
                {
                    KeyNum = 3;
                }
                else if (strcmp(KeyName, "4") == 0 && strcmp(Action, "down") == 0)
                {
                    KeyNum = 4;
                }
                else if (strcmp(KeyName, "5") == 0 && strcmp(Action, "down") == 0)
                {
                    KeyNum = 5;
                }
                else if (strcmp(KeyName, "6") == 0 && strcmp(Action, "down") == 0)
                {
                    KeyNum = 6;
                }
            }
            else if (strcmp(Tag, "slider") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");

                if (strcmp(Name, "SpeedKp") == 0)
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
                else if (strcmp(Name, "SpeedDeadZone") == 0)
                {
                    SpeedPID.DeadZone = atof(Value);
                }
                else if (strcmp(Name, "TurnDeadZone") == 0)
                {
                    TurnPID.DeadZone = atof(Value);
                }
            }
            else if (strcmp(Tag, "joystick") == 0)
            {
                int8_t LH = atoi(strtok(NULL, ","));
                int8_t LV = atoi(strtok(NULL, ","));
                int8_t RH = atoi(strtok(NULL, ","));
                int8_t RV = atoi(strtok(NULL, ","));
                SpeedPID.Target = LV / 5.0;
                TurnPID.Target = RH; // 直接使用RH作为角度目标
            }
            else if (strcmp(Tag, "angle") == 0)
            {
                // 直接角度控制命令：angle,90 或 angle,-45
                char *AngleStr = strtok(NULL, ",");
                if (AngleStr != NULL)
                {
                    float target_angle = atof(AngleStr);
                    // 限制角度范围
                    if (target_angle > 180)
                        target_angle = 180;
                    if (target_angle < -180)
                        target_angle = -180;
                    TurnPID.Target = target_angle;
                }
            }

            Serial_RxFlag = 0;
        }

        // 获取物理按键状态
        uint8_t PhysicalKey = Key_GetNum();
        if (PhysicalKey != 0)	KeyNum = PhysicalKey;

        // 按键处理
        switch (KeyNum)
        {
        case 1:
            if (RunFlag == 0)
            {
                RunFlag = 1;
                PID_Init(&SpeedPID);
                PID_Init(&TurnPID);
            }
            else
            {
                RunFlag = 0;
            }
            KeyNum = 0;
            break;
        case 2:
            TurnPID.Target += 30;
            if (TurnPID.Target > 180)
                TurnPID.Target -= 360;
            KeyNum = 0;
            break;
        case 3:
            TurnPID.Target -= 30;
            if (TurnPID.Target < -180)
                TurnPID.Target += 360;
            KeyNum = 0;
            break;
        case 4:
            SpeedPID.Target += 4;
            KeyNum = 0;
            break;
        case 5:
            SpeedPID.Target -= 4;
            KeyNum = 0;
            break;
        case 6:
            Angle_Reset();
            KeyNum = 0;
            break;
        default:
            break;
        }

		//更新数据显示
        updata_count++;
        if (updata_count >= 10)
        {
            updata_count = 0;
            // OLED显示
            OLED_Clear();
            OLED_Printf(0, 0, OLED_6X8, "Angle:%+06.1f", Angle);
            OLED_Printf(0, 16, OLED_6X8, "Target:%+06.1f", TurnPID.Target);
            OLED_Printf(0, 32, OLED_6X8, "%+04.1f %+04.1f", LeftSpeed, RightSpeed);
            // OLED_Printf(0, 32, OLED_6X8, "Speed:%+05.1f", AveSpeed);
            OLED_Printf(0, 48, OLED_6X8, "PWM:%d,%d", LeftPWM, RightPWM);
            // OLED_Printf(0, 32, OLED_6X8, "Encoder:%d,%d", LeftEncoder, RightEncoder);
            OLED_Update();

            Serial_Printf("%d,%d,%.2f,%.2f,%d,%d\n",
                          LeftEncoder, RightEncoder, 
						  LeftSpeed, RightSpeed, 
						  LeftPWM, RightPWM);
        }
    }
}

void TIM1_UP_IRQHandler(void)
{
    static uint16_t Count1;
    const float alpha = 0.1; // 低通滤波系数
    const float dt = 0.01;   // 采样周期(100Hz)

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        Key_Tick();

        // 高频任务：角度积分（100Hz）
        Count1++;
        if (Count1 >= 10)
        {
            Count1 = 0;
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

            // 陀螺仪数据低通滤波
            gyro_z_filtered = alpha * (GZ / 16.4 - 0.6) + (1 - alpha) * gyro_z_filtered;

            // 角度积分（度为单位）
            AngleIntegral += gyro_z_filtered * dt;

            // 角度归一化
            Angle_Normalize();
            // }

            // // 低频任务：控制算法（20Hz）
            // Count2++;
            // if (Count2 >= 50)
            // {
            //	 Count2 = 0;

            // 读取编码器数据
            LeftEncoder = Encoder_Get(1);
            RightEncoder = Encoder_Get(2);

            // 计算左右轮速度（转/秒）
            LeftSpeed = LeftEncoder / 44.0 / 0.05; // 44个脉冲每转，0.05秒采样周期
            RightSpeed = RightEncoder / 44.0 / 0.05;

            // 计算平均速度
            AveSpeed = (LeftSpeed + RightSpeed) / 2.0;

            if (RunFlag)
            {
                // 速度环PID控制
                SpeedPID.Actual = AveSpeed;
                PID_Update(&SpeedPID);

                // 转向环PID控制 - 使用角度误差
                TurnPID.Target = TurnPID.Target;
                TurnPID.Actual = Angle;
                PID_Update(&TurnPID);

                // 结合速度环和转向环输出
                LeftPWM = SpeedPID.Out - TurnPID.Out;  // 左轮 = 速度输出 - 转向输出
                RightPWM = SpeedPID.Out + TurnPID.Out; // 右轮 = 速度输出 + 转向输出

                // 限制PWM范围
                if (LeftPWM > 100)
                    LeftPWM = 100;
                if (LeftPWM < -100)
                    LeftPWM = -100;
                if (RightPWM > 100)
                    RightPWM = 100;
                if (RightPWM < -100)
                    RightPWM = -100;

                Motor_SetPWM(1, LeftPWM);
                Motor_SetPWM(2, RightPWM);
            }
            else
            {
                // 停止状态
                Motor_SetPWM(1, 0);
                Motor_SetPWM(2, 0);
                // 停止时不重置角度积分，保持当前位置记忆
                TurnPID.ErrorInt = 0; // 只重置PID积分项
                SpeedPID.ErrorInt = 0;
            }
        }
    }
}
