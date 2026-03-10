#include "Handfile.h"

int main(void)
{
    Init();
    while (1)
    {
        switch (TaskMode)
        {
        case MODE_IDLE:
            Task_Idle();
            break;
        case MODE_TASK1_AB:
            Task_1AB();
            break;
        case MODE_TASK2_ABCDA:
            Task_2ABCDA();
            break;
        case MODE_TASK3_ACBDA:
            Task_3ACBDA();
            break;
        case MODE_TASK4_4_ACBDA:
            Task_4Fig8_4Cycles();
            break;
        case MODE_Test:
            turn_task();
            // black_line_task();
            break;
        }

        LED_BUZZER();
        BlueSerial();
        // 获取物理按键状态
        PhysicalKey = Key_GetNum();
        if (PhysicalKey != 0)
            KeyNum = PhysicalKey;
        Key_action();
        Data_Update();
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

        Count1++;
        if (Count1 >= 10)
        {
            Count1 = 0;

            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            HC4051_ReadBinaryChannels(adc_values, binary_values, THRESHOLD);
            Calculate_Line_Position(&line_pos, &valid_sensor_count);

            // 陀螺仪数据低通滤波 0.57
            gyro_z_filtered = alpha * (GZ / 16.4 - 0.58) + (1 - alpha) * gyro_z_filtered;

            // 角度积分（度为单位）
            AngleIntegral += gyro_z_filtered * dt;

            // 角度归一化
            Angle_Normalize();

            // 读取编码器数据
            LeftEncoder = Encoder_Get(1);
            RightEncoder = Encoder_Get(2);

            // 计算左右轮速度（转/秒）
            LeftSpeed = LeftEncoder / 52.0 / dt; // 4(倍频)*13(线)个脉冲每转，0.05秒采样周期
            RightSpeed = RightEncoder / 52.0 / dt;

            // 计算平均速度
            AveSpeed = (LeftSpeed + RightSpeed) / 2.0;

            // 更新行驶距离
            Distance_Update();

            if (TaskMode != MODE_IDLE)
            {
                // 灰度循迹
                PID_Update(&CarPID, line_pos);
                // 速度环PID控制
                PID_Update(&SpeedPID, AveSpeed);

                // 转向环PID控制 - 使用角度误差
                PID_UpdateAngle(&TurnPID, Angle);

                // 结合速度环和转向环输出

                LeftPWM = SpeedPID.Out - TurnPID.Out + CarPID.Out;  // 左轮 = 速度输出 - 转向输出 - 循迹输出
                RightPWM = SpeedPID.Out + TurnPID.Out - CarPID.Out; // 右轮 = 速度输出 + 转向输出 + 循迹输出

                // 限制PWM范围
                // LeftPWM = fmin(fmax(LeftPWM, PWM_MIN), PWM_MAX);
                // RightPWM = fmin(fmax(RightPWM, PWM_MIN), PWM_MAX);

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
                CarPID.ErrorInt = 0;
            }
        }
    }
}
