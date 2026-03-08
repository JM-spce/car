#include "Handfile.h"

// 传感器权重（0-7 路对应位置 -50~+50）
const int8_t sensor_weight[8] = {-50, -30, -15, -5, 5, 15, 30, 50};

CarTaskMode TaskMode = MODE_IDLE;

Point current_point = POINT_A;

// ====================== 全局变量（显式初始化） ======================
uint8_t KeyNum = 0;                // 按键编号（初始 0）
uint8_t PhysicalKey = 0;           // 物理按键
uint8_t RunFlag = 0;               // 运行标志（初始停止）
uint8_t updata_count = 0;          // 显示更新计数器
uint16_t adc_values[8] = {0};      // ADC 值（初始 0）
uint8_t binary_values[8] = {0};    // 二值化值（初始 0）
int16_t LeftPWM = 0, RightPWM = 0; // 电机 PWM（初始 0）
int16_t line_pos = 0.0f;
const uint8_t LED_BUZZER_Count = 100;
uint8_t LED_BUZZER_falg = 0; // LED 灯和蜂鸣器  0关，1开，自增到100跳为0

int16_t AX, AY, AZ, GX, GY, GZ;

uint16_t temp;

int16_t LeftEncoder, RightEncoder;
float LeftSpeed, RightSpeed;
float AveSpeed;

float Angle;               // 最终偏航角（当前角度 -180~180 度）
float AngleIntegral = 0;   // 角度积分值
float BaseAngle = 0;       // 基准角度
float gyro_z_filtered = 0; // 滤波后的 GZ 陀螺仪数据

float TotalDistance = 0.0f;   // 总行驶距离（厘米）
float CurrentDistance = 0.0f; // 当前段行驶距离（厘米）
float WheelRadius = 2.4f;     // 轮子半径（厘米），根据实际情况调整

uint16_t BASE_PWM = 300;

PID_t SpeedPID = {
    .Target = 40,
    .Kp = 20,
    .Ki = 0.5,
    .Kd = 0,
    .OutMax = 1000,
    .OutMin = -1000,
    .IntegralMax = 1500,
    .IntegralMin = -1500,
    .DeadZone = 2.0,

    .Mode = PID_INCREMENTAL, // 增量式
    .Enabled = 1};

PID_t TurnPID = {
    .Kp = 18,
    .Ki = 0.2f,
    .Kd = 0.2f,
    .OutMax = 1000,
    .OutMin = -1000,
    .IntegralMax = 1000,
    .IntegralMin = -1000,
    .DeadZone = 0,
    .DeadZoneEn = 0,
    .Mode = PID_DERIVATIVE_ON_MEASUREMENT, // 微分先行（抗干扰）
    .Enabled = 1};

PID_t CarPID = {
    .Kp = 30.0f,    // 比例系数（实测稳定值）
    .Ki = 0.0f,     // 关闭积分（避免饱和）
    .Kd = 2.0f,     // 微分系数（抑制超调）
    .Target = 0.0f, // 目标位置（传感器中心）
    .OutMax = 1500,
    .OutMin = -1500,
    .IntegralMax = 0.0f, // 积分禁用
    .IntegralMin = 0.0f,
    .DeadZone = DEAD_ZONE, // 死区（避免震荡）
    .DeadZoneEn = 1,
    .Mode = PID_DERIVATIVE_ON_MEASUREMENT, // 微分先行（抗干扰）
    .Enabled = 1};

void LED_BUZZER(void)
{
    if (LED_BUZZER_falg != 0)
    {
        LED_ON();
        BUZZER_ON();
        LED_BUZZER_falg++;
        if (LED_BUZZER_falg == LED_BUZZER_Count)
        {
            LED_BUZZER_falg = 0;
        }
    }
    else if (LED_BUZZER_falg == 0)
    {
        LED_OFF();
        BUZZER_OFF();
    }
}
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
    PID_SetTarget(&TurnPID, 0);
    PID_Update(&TurnPID, 0);
}
void Init(void)
{
    OLED_Init();
    BUZZER_Init();
    MPU6050_Init();
    BlueSerial_Init();
    LED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();
    HC4051_Init();
}

// 处理蓝牙数据
void BlueSerial(void)
{
    if (BlueSerial_RxFlag == 1)
    {
        char *Tag = strtok(BlueSerial_RxPacket, ",");
        char *KeyName = strtok(NULL, ",");
        char *Action = strtok(NULL, ",");

        if (Tag != NULL && strcmp(Tag, "key") == 0 && strcmp(Action, "down") == 0)
        {
            switch (KeyName[0])
            {
            case '0':
                KeyNum = 0;
                break;
            case '1':
                KeyNum = 1;
                break;
            case '2':
                KeyNum = 2;
                break;
            case '3':
                KeyNum = 3;
                break;
            case '4':
                KeyNum = 4;
                break;
            case '5':
                KeyNum = 5;
                break;
            case '6':
                KeyNum = 6;
                break;
            case '7':
                KeyNum = 7;
                break;
            case '8':
                KeyNum = 8;
                break;
            case '9':
                KeyNum = 9;
                break;
            default:
                break;
            }
        }
        // else if (strcmp(Tag, "slider") == 0)
        // {
        //     char *Name = strtok(NULL, ",");
        //     char *Value = strtok(NULL, ",");

        //     if (strcmp(Name, "SpeedKp") == 0)
        //     {
        //         SpeedPID.Kp = atof(Value);
        //     }
        //     else if (strcmp(Name, "SpeedKi") == 0)
        //     {
        //         SpeedPID.Ki = atof(Value);
        //     }
        //     else if (strcmp(Name, "SpeedKd") == 0)
        //     {
        //         SpeedPID.Kd = atof(Value);
        //     }
        //     else if (strcmp(Name, "TurnKp") == 0)
        //     {
        //         TurnPID.Kp = atof(Value);
        //     }
        //     else if (strcmp(Name, "TurnKi") == 0)
        //     {
        //         TurnPID.Ki = atof(Value);
        //     }
        //     else if (strcmp(Name, "TurnKd") == 0)
        //     {
        //         TurnPID.Kd = atof(Value);
        //     }
        //     else if (strcmp(Name, "SpeedDeadZone") == 0)
        //     {
        //         SpeedPID.DeadZone = atof(Value);
        //     }
        //     else if (strcmp(Name, "TurnDeadZone") == 0)
        //     {
        //         TurnPID.DeadZone = atof(Value);
        //     }
        // }
        // else if (strcmp(Tag, "joystick") == 0)
        // {
        //     int8_t LH = atoi(strtok(NULL, ","));
        //     int8_t LV = atoi(strtok(NULL, ","));
        //     int8_t RH = atoi(strtok(NULL, ","));
        //     int8_t RV = atoi(strtok(NULL, ","));
        //     SpeedPID.Target = LV / 5.0;
        //     TurnPID.Target = RH; // 直接使用RH作为角度目标
        // }
        // else if (strcmp(Tag, "angle") == 0)
        // {
        //     // 直接角度控制命令：angle,90 或 angle,-45
        //     char *AngleStr = strtok(NULL, ",");
        //     if (AngleStr != NULL)
        //     {
        //         float target_angle = atof(AngleStr);
        //         // 限制角度范围
        //         if (target_angle > 180)
        //             target_angle = 180;
        //         if (target_angle < -180)
        //             target_angle = -180;
        //         TurnPID.Target = target_angle;
        //     }
        // }

        BlueSerial_RxFlag = 0;
    }
}

void Key_action(void)
{
    //状态机
    // switch (KeyNum)
    // {
    // case 1:
    //     TaskMode = MODE_TASK1_AB;
    //     KeyNum = 0;
    //     break;
    // case 2:
    //     TaskMode = MODE_TASK2_ABCDA;
    //     KeyNum = 0;
    //     break;
    // case 3:
    //     TaskMode = MODE_TASK3_ACBDA;
    //     KeyNum = 0;
    //     break;
    // case 4:
    //     TaskMode = MODE_TASK4_4_ACBDA;
    //     KeyNum = 0;
    //     break;
    // case 5:
    //     TaskMode = MODE_IDLE;
    //     KeyNum = 0;
    //     break;
    // default:
    //     break;
    // }

    //TurnPID
    switch (KeyNum)
    {
    case 1:
        TurnPID.Kp += 0.2f;
        KeyNum = 0;
        break;
    case 2:
        TurnPID.Kp -= 0.1f;
        KeyNum = 0;
        break;
    case 3:
        TurnPID.Ki += 0.2f;
        KeyNum = 0;
        break;
    case 4:
        TurnPID.Ki -= 0.1f;
        KeyNum = 0;
        break;
    case 5:
        TurnPID.Kd += 0.2f;
        KeyNum = 0;
        break;
    case 6:
        TurnPID.Kd -= 1.0f;
        KeyNum = 0;
        break;
    case 7:
        temp = TurnPID.Target + 15;
        PID_SetTarget(&TurnPID, temp);
        KeyNum = 0;
        break;
    case 8:
        TaskMode = MODE_Test;
        // temp = TurnPID.Actual - 10;
        // PID_SetTarget(&TurnPID, temp);
        KeyNum = 0;
        break;
    case 9:
        TaskMode = MODE_IDLE;
        KeyNum = 0;

    default:
        break;
    }
}

void Data_Update(void)
{
    updata_count++;
    if (updata_count >= 10)
    {
        updata_count = 0;

        OLED_Clear();
        // for (uint8_t i = 0; i < 8; i++)
        //     OLED_ShowBinNum(i * 6, 0, binary_values[i], 1, OLED_6X8);

        OLED_Printf(0, 0, OLED_6X8, "  Speed");
        OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", SpeedPID.Kp);
        OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", SpeedPID.Ki);
        OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", SpeedPID.Kd);
        OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", SpeedPID.Target);
        OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", AveSpeed);
        OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", SpeedPID.Out);
        OLED_Printf(0, 56, OLED_6X8, "Ds:       Ang:%5.2f", Angle);
        OLED_ShowFloatNum(3 * 6, 56, CurrentDistance, 3, 1, OLED_6X8);

        OLED_Printf(50, 0, OLED_6X8, "Turn");
        OLED_Printf(50, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
        OLED_Printf(50, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
        OLED_Printf(50, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
        OLED_Printf(50, 32, OLED_6X8, "%+05.1f", TurnPID.Target);
        OLED_Printf(50, 40, OLED_6X8, "%+05.1f", Angle);
        OLED_Printf(50, 48, OLED_6X8, "%+05.0f", TurnPID.Out);
        OLED_Printf(88, 0, OLED_6X8, "car");
        OLED_Printf(88, 8, OLED_6X8, "%05.2f", CarPID.Kp);
        OLED_Printf(88, 16, OLED_6X8, "%05.2f", CarPID.Ki);
        OLED_Printf(88, 24, OLED_6X8, "%05.2f", CarPID.Kd);
        OLED_Printf(88, 32, OLED_6X8, "%+05.1f", CarPID.Target);
        OLED_ShowFloatNum(88, 40, line_pos, 3, 1, OLED_6X8);
        OLED_Printf(88, 48, OLED_6X8, "%+05.0f", CarPID.Out);

        
        // for (uint8_t i = 0; i < 8; i++)
        //     OLED_ShowNum(i * 16, 0, binary_values[i], 1, OLED_6X8);
        // OLED_ShowSignedNum(0, 8, line_pos, 5, OLED_6X8);
        // // OLED_Printf(0, 8, OLED_6X8, "Angle:%+06.1f", Angle);
        // OLED_Printf(0, 16, OLED_6X8, "Target:%+06.1f", TurnPID.Target);
        // OLED_Printf(0, 24, OLED_6X8, "%+04.1f %+04.1f", LeftSpeed, RightSpeed);
        // // OLED_Printf(0, 24, OLED_6X8, "Speed:%+05.1f", AveSpeed);
        // OLED_Printf(0, 32, OLED_6X8, "PWM:%d,%d", LeftPWM, RightPWM);
        // // OLED_Printf(0, 24, OLED_6X8, "Encoder:%d,%d", LeftEncoder, RightEncoder);
        // OLED_Update();

        // OLED_ShowString(0, 0, "Kp:", OLED_6X8);
        // OLED_ShowString(0, 0, "Kp:", OLED_6X8);
        // OLED_ShowFloatNum(6 * 3, 0, CarPID.Kp, 2, 2, OLED_6X8);
        // OLED_ShowString(0, 8, "Kd:", OLED_6X8);
        // OLED_ShowFloatNum(6 * 3, 8, CarPID.Kd, 2, 2, OLED_6X8);
        // OLED_ShowString(0, 16, "Speed:", OLED_6X8);
        // OLED_ShowFloatNum(10 * 4, 16, SpeedPID.Out, 3, 1, OLED_6X8);
        // OLED_ShowString(0, 24, "Pos:", OLED_6X8);
        // OLED_ShowSignedNum(4 * 6, 24, line_pos, 2, OLED_6X8);
        // OLED_ShowString(8 * 6, 24, "Ang:", OLED_6X8);
        // OLED_ShowFloatNum(12 * 6, 24, Angle, 3, 1, OLED_6X8);
        // OLED_ShowString(0, 32, "PWM:", OLED_6X8);
        // OLED_ShowNum(4 * 6, 32, LeftPWM, 3, OLED_6X8);
        // OLED_ShowNum(8 * 6, 32, RightPWM, 3, OLED_6X8);
        // OLED_ShowString(0, 40, "Mod:", OLED_6X8);
        // OLED_ShowNum(4 * 6, 40, TaskMode, 1, OLED_6X8);
        // OLED_ShowString(0, 48, "Dist:", OLED_6X8);
        // OLED_ShowFloatNum(5 * 6, 48, CurrentDistance, 3, 1, OLED_6X8);
        // OLED_ShowString(9 * 6, 48, "cm", OLED_6X8);

        OLED_Update();

        BlueSerial_Printf("%.2f,%.2f\n", Angle, TurnPID.Target);

        // BlueSerial_Printf("%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f\n",
        //                   SpeedPID.Target, TurnPID.Target,
        //                   LeftSpeed, RightSpeed,
        //                   LeftPWM, RightPWM,
        //                   LeftEncoder, RightEncoder,
        //                   Angle);
    }
}

// 计算加权线位置
int16_t Calculate_Line_Position(void)
{
    int32_t weight_sum = 0;
    uint8_t valid_sensor = 0;

    // 统计有效传感器和总权重
    for (uint8_t i = 0; i < 8; i++)
    {
        if (binary_values[i] == 1)
        {
            weight_sum += (int32_t)sensor_weight[i];
            valid_sensor++;
        }
    }

    // 计算加权平均位置
    if (valid_sensor == 0)
        return 0; // 无有效传感器，返回中心
    float avg_pos = (float)weight_sum / valid_sensor;
    return (int16_t)avg_pos;
}

// 计算当前行驶距离
void Distance_Update(void)
{
    // 计算平均移动距离（弧长公式：s = r * θ，θ单位为弧度）
    // 编码器每转 52 个脉冲，每个脉冲对应的角度 = 2π/52
    int16_t left_delta = LeftEncoder;
    int16_t right_delta = RightEncoder;

    float avg_delta = (left_delta + right_delta) / 2.0f;
    float distance_cm = avg_delta * (2.0f * 3.14159f / (52.0f * 20)) * WheelRadius;

    // 累加总距离
    TotalDistance += distance_cm;
    CurrentDistance += distance_cm;
}

// 重置距离
void Distance_Reset(void)
{
    TotalDistance = 0.0f;
    CurrentDistance = 0.0f;
}

void Task_Idle(void)
{
    // 停止所有电机
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);

    // Angle_Reset();
    // 停止所有距离计数
    // Distance_Reset();
}
// 任务1：A → B 直线（无轨迹，陀螺仪直走）
void Task_1AB(void)
{
    if (current_point == POINT_A)
    {
        SpeedPID.Enabled = 1;
        TurnPID.Enabled = 1;
        CarPID.Enabled = 0;

        if (CurrentDistance >= DIST_AB && line_pos != 0)
        {
            CurrentDistance = 0;
            current_point = POINT_B;
            LED_BUZZER_falg = 1;
            TaskMode = MODE_IDLE;
        }
    }
}

// 任务2：A → B → C → D → A 一圈
// AB(直) → BC(循迹半圆) → CD(直) → DA(循迹半圆)
void Task_2ABCDA(void)
{
    // A → B：直线无轨迹，陀螺仪
    if (current_point == POINT_A)
    {
        CarPID.Enabled = 0;

        if (CurrentDistance >= DIST_AB && line_pos != 0)
        {
            CurrentDistance = 0;
            current_point = POINT_B;
            LED_BUZZER_falg = 1;
        }
    }
    // B → C：灰度半圆循迹
    else if (current_point == POINT_B)
    {
        SpeedPID.Enabled = 1;
        CarPID.Enabled = 1;
        TurnPID.Enabled = 0;
        if (CurrentDistance >= DIST_BC && line_pos == 0)
        {
            SpeedPID.Enabled = 1;
            TurnPID.Enabled = 1;
            CarPID.Enabled = 0;

            CurrentDistance = 0;
            current_point = POINT_C;
            LED_BUZZER_falg = 1;
        }
    }

    // C → D：直线无轨迹，陀螺仪
    else if (current_point == POINT_C)
    {
        if (CurrentDistance >= DIST_AB)
        {
            CurrentDistance = 0;
            current_point = POINT_D;
            LED_BUZZER_falg = 1;
        }
    }

    // D → A：灰度半圆循迹，回到起点
    else if (current_point == POINT_D)
    {
        SpeedPID.Enabled = 1;
        CarPID.Enabled = 1;
        TurnPID.Enabled = 0;
        if (CurrentDistance >= DIST_BC && line_pos != 0)
        {
            CurrentDistance = 0;
            current_point = POINT_C;
            LED_BUZZER_falg = 1;
        }
    }
}

// 任务3：8字 A→C→B→D→A
// A→C(直) → C→B(循迹半圆) → B→D(直) → D→A(循迹半圆)
void Task_3ACBDA(void)
{
    // A → C：直线
    if (current_point == POINT_A)
    {
        CarPID.Enabled = 0;
        TurnPID.Target = Angle_AC;
        TurnPID.Enabled = 1;

        if (CurrentDistance >= DIST_AC)
        {
            CurrentDistance = 0;
            current_point = POINT_C;
        }
    }

    // C → B：循迹半圆
    else if (current_point == POINT_C)
    {
        CarPID.Enabled = 1;
        TurnPID.Enabled = 0;

        if (CurrentDistance >= DIST_BC)
        {
            CurrentDistance = 0;
            Angle_Reset();
            current_point = POINT_B;
        }
    }

    // B → D：直线无轨迹
    else if (current_point == POINT_B)
    {
        CarPID.Enabled = 0;
        TurnPID.Target = -Angle_AC;
        TurnPID.Enabled = 1;

        if (CurrentDistance >= DIST_AC)
        {
            CurrentDistance = 0;
            current_point = POINT_D;
        }
    }

    // D → A：循迹半圆，回到A
    else if (current_point == POINT_D)
    {
        CarPID.Enabled = 1;
        TurnPID.Enabled = 0;

        if (CurrentDistance >= DIST_BC)
        {
            CurrentDistance = 0;
            Angle_Reset();
            current_point = POINT_A;
        }
    }
}

// 任务4：8字连续4圈
void Task_4Fig8_4Cycles(void)
{
    // static uint8_t lap = 0;

    // Task_3ACBDA();

    // if (Car_State == STATE_TASK_COMPLETE)
    // {
    //     lap++;
    //     if (lap >= 4)
    //     {
    //         lap = 0;
    //         Car_State = STATE_TASK_COMPLETE;
    //     }
    //     else
    //     {
    //         Car_State = STATE_ANGLE_CRUISE;
    //         current_point = POINT_A;
    //         CurrentDistance = 0;
    //         Angle_Reset();
    //     }
    // }
}

void black_line_task(void)
{
    SpeedPID.Enabled = 1;
    TurnPID.Enabled = 0;
    CarPID.Enabled = 1;
}

void turn_task(void)
{
    SpeedPID.Enabled = 0;
    TurnPID.Enabled = 1;
    CarPID.Enabled = 0;
}
