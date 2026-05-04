#include "Handfile.h"

int main(void)
{
    Init();
    while (1)
    {
        LED_BUZZER();
        Serial();
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
    static uint16_t Count2;
    const float dt_1 = 0.01;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

        Key_Tick();
        Count1++;
        Count2++;

        // 读取灰度传感器
        HC4051_ReadBinaryChannels(adc_values, binary_values, THRESHOLD);

        if (TaskMode != MODE_IDLE)
        {
            Calculate_Line_Position(&line_pos, &valid_sensor_count);
            // PID 控制
            if (CarPID.Enabled)
                PID_Update(&CarPID, line_pos);
            if (SpeedPID.Enabled)
                PID_Update(&SpeedPID, AveSpeed);
            if (TurnPID.Enabled)
                PID_UpdateAngle(&TurnPID, JY901P.yaw);

            // 输出计算
            LeftPWM = SpeedPID.Out - TurnPID.Out + CarPID.Out;
            RightPWM = SpeedPID.Out + TurnPID.Out - CarPID.Out;

            // LeftPWM = fmin(fmax(LeftPWM, PWM_MIN), PWM_MAX);
            // RightPWM = fmin(fmax(RightPWM, PWM_MIN), PWM_MAX);

            Motor_SetPWM(1, LeftPWM);
            Motor_SetPWM(2, RightPWM);
        }
        else
        {
            Motor_SetPWM(1, 0);
            Motor_SetPWM(2, 0);
            TurnPID.ErrorInt = 0;
            SpeedPID.ErrorInt = 0;
            CarPID.ErrorInt = 0;
        }

        if (Count1 >= 5)
        {
            Count1 = 0;
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
                // turn_task();
                // black_line_task();
                break;
            }
        }

        if (Count2 >= 10)
        {
            Count2 = 0;

            Update_Sensor_Status();

            // 读取编码器
            LeftEncoder = Encoder_Get(1);
            RightEncoder = Encoder_Get(2);

            // 更新行驶距离
            Distance_Update();

            // 计算速度
            LeftSpeed = LeftEncoder / 52.0 / dt_1;
            RightSpeed = RightEncoder / 52.0 / dt_1;
            AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
        }
    }
}
