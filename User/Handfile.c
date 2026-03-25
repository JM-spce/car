#include "Handfile.h"

// 传感器权重（0-7 路对应位置 -50~+50）
// const int8_t sensor_weight[8] = {-40, -30, -15, -5, 5, 15, 30, 40};
const int8_t sensor_weight[8] = {-50, -25, -15, -5, 5, 15, 25, 50};

CarTaskMode TaskMode = MODE_IDLE;

Point current_point = POINT_A;

// ====================== 全局变量（显式初始化） ======================
uint8_t cycle_flag = 0;
uint8_t cycle_count = 0;

uint8_t KeyNum = 0;                // 按键编号（初始 0）
uint8_t PhysicalKey = 0;           // 物理按键
uint8_t RunFlag = 0;               // 运行标志（初始停止）
uint8_t updata_count = 0;          // 显示更新计数器
uint16_t adc_values[8] = {0};      // ADC 值（初始 0）
uint8_t binary_values[8] = {0};    // 二值化值（初始 0）
int16_t LeftPWM = 0, RightPWM = 0; // 电机 PWM（初始 0）

int16_t line_pos = 0.0f;
uint8_t valid_sensor_count = 0;
uint8_t count_8_consecutive = 0; // 连续检测到 8 的计数器
uint8_t is_valid_8_detected = 0; // 连续三次为 8 的标志位
int8_t line_flag = 0;            // 0:正常循迹,1:关闭右四路,-1关闭左四路

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
float WheelRadius = 2.4f;     // 轮子半径（厘米）

// 速度配置
const float NORMAL_SPEED = 50.0f; // 正常行驶速度（入弯前的初始速度）     50
const float TURN_SPEED = 40.0f;   // 弯道行驶速度                       50
const float ACCEL_SPEED = 60.0f;  // 加速速度（出弯后）                  70

float Smooth_Speed = 00.0f;

PID_t SpeedPID = {
    .Target = NORMAL_SPEED,
    .Kp = 20,
    .Ki = 0.6,
    .Kd = 0.15, // 0.05
    .OutMax = 1000,
    .OutMin = -1000,
    .IntegralMax = 1500,
    .IntegralMin = -1500,
    .DeadZone = 0.5,
    .Mode = PID_INCREMENTAL, // 增量式
    .Enabled = 1};

PID_t TurnPID = {
    .Kp = 3.2f, // 5
    .Ki = 0.0f,
    .Kd = 0.5f, // 0.15
    .OutMax = 500,
    .OutMin = -500,
    .IntegralMax = 1000,
    .IntegralMin = -1000,
    .DeadZone = 0,
    .DeadZoneEn = 0,
    .Mode = PID_DERIVATIVE_ON_MEASUREMENT, // 微分先行（抗干扰）
    .Enabled = 1};

PID_t CarPID = {    // 40,45,50
    .Kp = 16.0f,    // 比例系数 50
    .Ki = 0.0f,     // 关闭积分（避免饱和）
    .Kd = 0.7f,     // 微分系数（抑制超调）0.4
    .Target = 0.0f, // 目标位置（传感器中心）
    .OutMax = 500,
    .OutMin = -500,
    .IntegralMax = 0.0f, // 积分禁用
    .IntegralMin = 0.0f,
    .DeadZone = DEAD_ZONE, // 死区（避免震荡）
    .DeadZoneEn = 1,
    .Mode = PID_DERIVATIVE_ON_MEASUREMENT, // 微分先行（抗干扰）
    .Enabled = 1};

void LED_BUZZER(void)
{
    if (LED_BUZZER_falg == 1)
    {
        LED_ON();
        BUZZER_ON();
    }
    if (LED_BUZZER_falg != 0)
    {
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
float Angle_Normalize(float Angle)
{
    // 将角度归一化到[-180, 180]范围
    while (Angle > 180.0)
        Angle -= 360.0;
    while (Angle < -180.0)
        Angle += 360.0;
    return Angle;
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
        BlueSerial_RxFlag = 0;
    }
}

void Key_action(void)
{
    // 状态机
    switch (KeyNum)
    {
    case 1:
        TaskMode = MODE_TASK1_AB;
        KeyNum = 0;
        break;
    case 2:
        TaskMode = MODE_TASK2_ABCDA;
        KeyNum = 0;
        break;
    case 3:
        TaskMode = MODE_TASK3_ACBDA;
        KeyNum = 0;
        break;
    case 4:
        TaskMode = MODE_TASK4_4_ACBDA;
        KeyNum = 0;
        break;
    case 5:
        TaskMode = MODE_IDLE;
        KeyNum = 0;
        break;
    case 6:
        Angle_Reset();
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
        // 灰度
         for (uint8_t i = 0; i < 8; i++)
             OLED_ShowBinNum(i * 6, 0, binary_values[i], 1, OLED_6X8);
         OLED_ShowNum(0, 8, adc_values[0], 5, OLED_6X8);
         OLED_ShowNum(0, 16, adc_values[1], 5, OLED_6X8);
         OLED_ShowNum(0, 24, adc_values[2], 5, OLED_6X8);
         OLED_ShowNum(0, 32, adc_values[3], 5, OLED_6X8);
         OLED_ShowNum(6*6, 8, adc_values[4], 5, OLED_6X8);
         OLED_ShowNum(6*6, 16, adc_values[5], 5, OLED_6X8);
         OLED_ShowNum(6*6, 24, adc_values[6], 5, OLED_6X8);
         OLED_ShowNum(6*6, 32, adc_values[7], 5, OLED_6X8);

        // OLED_Printf(0, 0, OLED_6X8, "  Speed");
        // OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", SpeedPID.Kp);
        // OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", SpeedPID.Ki);
        // OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", SpeedPID.Kd);
        // OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", SpeedPID.Target);
        // OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", AveSpeed);
        // OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", SpeedPID.Out);
        // OLED_Printf(0, 56, OLED_6X8, "Ds:       A:%3.2f,%d", Angle, valid_sensor_count);
        // OLED_ShowFloatNum(3 * 6, 56, CurrentDistance, 3, 1, OLED_6X8);
        // OLED_Printf(50, 0, OLED_6X8, "Turn");
        // OLED_Printf(50, 8, OLED_6X8, "%05.2f", TurnPID.Kp);
        // OLED_Printf(50, 16, OLED_6X8, "%05.2f", TurnPID.Ki);
        // OLED_Printf(50, 24, OLED_6X8, "%05.2f", TurnPID.Kd);
        // OLED_Printf(50, 32, OLED_6X8, "%+05.1f", TurnPID.Target);
        // OLED_Printf(50, 40, OLED_6X8, "%+05.1f", Angle);
        // OLED_Printf(50, 48, OLED_6X8, "%+05.0f", TurnPID.Out);
        // OLED_Printf(88, 0, OLED_6X8, "car");
        // OLED_Printf(88, 8, OLED_6X8, "%05.2f", CarPID.Kp);
        // OLED_Printf(88, 16, OLED_6X8, "%05.2f", CarPID.Ki);
        // OLED_Printf(88, 24, OLED_6X8, "%05.2f", CarPID.Kd);
        // OLED_Printf(88, 32, OLED_6X8, "%+05.1f", CarPID.Target);
        // OLED_ShowFloatNum(88, 40, line_pos, 3, 1, OLED_6X8);
        // OLED_Printf(88, 48, OLED_6X8, "%+05.0f", CarPID.Out);

        OLED_Update();

        BlueSerial_Printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", valid_sensor_count, SpeedPID.Actual, SpeedPID.Target, Angle, TurnPID.Target, CurrentDistance);
    }
}

// 计算加权线位置
void Calculate_Line_Position(int16_t *line_pos, uint8_t *valid_sensor_count)
{
    int32_t weight_sum = 0;  // 总权重
    uint8_t valid_count = 0; // 白线数量

    // sensor_weight[8] = {-50, -30, -15, -5, 5, 15, 30, 50}
    // 0位置黑线，sum=50,pos=50/7=7.14
    // 01位置黑线,sum=50+30=80,pos=80/6=13.33
    // 将sum/count改为sum/(8-count)
    // int16_t pos = (int16_t)((float)weight_sum / (8-valid_count));
    // 0位置黑线，sum=50,pos=50/(8-7)=50
    // 01位置黑线,sum=50+30=80,pos=80/(8-6)=40

    for (uint8_t i = 0; i < 8; i++)
    {
        if (binary_values[i] == 1)
        {
            weight_sum += sensor_weight[i];
            valid_count++;
        }
    }

    if (valid_count == 0)
    {
        *line_pos = 0;
        *valid_sensor_count = 0;
        return;
    }

    int16_t pos = (int16_t)((float)weight_sum / (8 - valid_count));

    if (binary_values[3] == 0 || binary_values[4] == 0)
        line_flag = 0;

    // 输出滤波：根据 line_flag 限制方向
    if (line_flag == -1 && pos > 0)
        pos = -pos; // 关闭左四路，只允许负值
    else if (line_flag == 1 && pos < 0)
        pos = -pos; // 关闭右四路，只允许正值

    *line_pos = pos;
    *valid_sensor_count = valid_count;
}

void Update_Sensor_Status(void)
{
    if (valid_sensor_count == 8)
    {
        if (count_8_consecutive < 3)
        {
            count_8_consecutive++;
        }
        if (count_8_consecutive >= 3)
        {
            is_valid_8_detected = 1;
        }
    }
    else
    {
        count_8_consecutive = 0;
        is_valid_8_detected = 0;
    }
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

void Smooth_Speed_Control(float distance_to_turn, uint8_t in_turn)
{
    float target_speed;

    if (in_turn)
    {
        // 弯道中：固定低速
        target_speed = TURN_SPEED;
    }
    else if (distance_to_turn > 0 && distance_to_turn <= DECEL_DIST)
    {
        // 入弯前：平滑减速到 TURN_SPEED
        float decel_ratio = distance_to_turn / DECEL_DIST;
        float smooth_ratio = (1.0f + cosf(decel_ratio * 3.14159f)) / 2.0f;
        target_speed = TURN_SPEED + (NORMAL_SPEED - TURN_SPEED) * smooth_ratio;
    }
    else if (distance_to_turn < 0 && fabs(distance_to_turn) <= ACCEL_DIST)
    {
        // 出弯后：平滑加速到 ACCEL_SPEED
        float accel_ratio = fabs(distance_to_turn) / ACCEL_DIST;
        float smooth_ratio = sinf(accel_ratio * 3.14159f / 2.0f);
        target_speed = TURN_SPEED + (ACCEL_SPEED - TURN_SPEED) * smooth_ratio;
        if (target_speed > ACCEL_SPEED)
            target_speed = ACCEL_SPEED;
    }
    // else if (distance_to_turn > DECEL_DIST)
    // {
    //     // 远离弯道：正常速度
    //     target_speed = NORMAL_SPEED;
    // }
    else // distance_to_turn < -ACCEL_DIST（出弯后很远）
    {
        target_speed = NORMAL_SPEED;
    }

    // 一阶低通滤波（保持平滑）
    float alpha = 0.1f; // 更小的alpha，更平滑
    Smooth_Speed = Smooth_Speed * (1.0f - alpha) + target_speed * alpha;

    PID_SetTarget(&SpeedPID, Smooth_Speed);
}

void Task_Idle(void)
{
    // 停止所有电机
    Motor_SetPWM(1, 0);
    Motor_SetPWM(2, 0);
    current_point = POINT_A;
    CurrentDistance = 0;

    // Angle_Reset();
    // 停止所有距离计数
    // Distance_Reset();
}
// 任务1：A → B 直线
void Task_1AB(void)
{
    if (current_point == POINT_A)
    {
        if (CurrentDistance < 1.0f) // 仅在起点执行一次
        {
            PID_Clear(&TurnPID);
            Angle_Reset();
            PID_SetTarget(&TurnPID, 0.0f); // 明确设置目标角度为 0 度
        }

        LED_BUZZER_falg = 0;
        line_flag = 0;

        SpeedPID.Enabled = 1;
        TurnPID.Enabled = 1;
        CarPID.Enabled = 0;

        if (CurrentDistance >= DIST_AB && is_valid_8_detected == 0)
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
        if (CurrentDistance < 1.0f) // 仅在起点执行一次
        {
            PID_Clear(&TurnPID);
            Angle_Reset();
            PID_SetTarget(&TurnPID, 0.0f); // 明确设置目标角度为 0 度
        }

        SpeedPID.Enabled = 1;
        TurnPID.Enabled = 1;
        CarPID.Enabled = 0;

        // 入弯前丝滑减速控制
        float distance_to_turn = DIST_AB + 10 - CurrentDistance;
        Smooth_Speed_Control(distance_to_turn, 0);

        if (CurrentDistance >= DIST_AB && is_valid_8_detected == 0)
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

        // 弯道中保持低速
        Smooth_Speed_Control(0, 1);

        if (CurrentDistance >= DIST_BC && is_valid_8_detected == 1)
        {
            PID_Clear(&TurnPID);
            PID_SetTarget(&TurnPID, -178);
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
        // 入弯前丝滑减速控制
        float distance_to_turn = DIST_CD + 10 - CurrentDistance;
        Smooth_Speed_Control(distance_to_turn, 0);

        if (CurrentDistance >= DIST_AB && is_valid_8_detected == 0)
        {
            CurrentDistance = 0;
            // 弯道中保持低速
            Smooth_Speed_Control(0, 1);
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

        if (CurrentDistance >= DIST_BC && is_valid_8_detected == 1)
        {
            CurrentDistance = 0;
            current_point = POINT_A;
            LED_BUZZER_falg = 1;
            TaskMode = MODE_IDLE;
        }
    }
}

// 任务3：8字 A→C→B→D→A
// A→C(直) → C→B(循迹半圆) → B→D(直) → D→A(循迹半圆)
void Task_3ACBDA(void)
{
    // A → C：直线斜线入弯（关闭右四路）
    if (current_point == POINT_A)
    {
        line_flag = 1;
        SpeedPID.Enabled = 1;
        TurnPID.Enabled = 1;
        CarPID.Enabled = 0;
        // PID_SetTarget(&TurnPID, -41.66f); //-38.66,43.66
        if (CurrentDistance < 1.0f) // 仅在起点执行一次
        {
            PID_Clear(&TurnPID);
            Angle_Reset();
            PID_SetTarget(&TurnPID, -39.66f); 
        }

        // 入弯前丝滑减速控制
        float distance_to_turn = DIST_AC + 8 - CurrentDistance;
        Smooth_Speed_Control(distance_to_turn, 0);

        if (CurrentDistance >= DIST_AC && is_valid_8_detected == 0)
        {
            SpeedPID.Enabled = 1;
            CarPID.Enabled = 1;
            TurnPID.Enabled = 0;
            // PID_SetTarget(&TurnPID, 0);
            CurrentDistance = 0;
            // 弯道中保持低速
            Smooth_Speed_Control(0, 1);
            current_point = POINT_C;
            LED_BUZZER_falg = 1;
        }
    }
    // C->B：灰度半圆循迹
    else if (current_point == POINT_C)
    {
        // if (binary_values[3] == 0 || binary_values[4] == 0)
        //     line_flag = 0;

        if (CurrentDistance >= DIST_BC && is_valid_8_detected == 1)
        {
            // // ========== 重置基准角，防绕大圈 ==========
            // 1. 重置角度积分，以当前角度为基准 0°
            BaseAngle = Angle; // 保存出弯后的当前角度作为新基准
            AngleIntegral = 0; // 清空角度积分
            Angle = 0;         // 新基准下当前角度为 0°
            // 2. 清空 TurnPID 的历史偏差、积分、微分状态（关键）
            PID_Clear(&TurnPID);
            // 3. 计算归一化后的目标角度（基于新基准）
            float target_angle = -145.34f - BaseAngle; // 141.34  149.34
            // 角度归一化到 [-180, 180]，避免绕大圈
            while (target_angle > 180.0f)
                target_angle -= 360.0f;
            while (target_angle < -180.0f)
                target_angle += 360.0f;
            // 4. 设置归一化后的目标角度
            PID_SetTarget(&TurnPID, target_angle);

            // ========== 原有逻辑保留 ==========
            SpeedPID.Enabled = 1;
            TurnPID.Enabled = 1;
            CarPID.Enabled = 0;

            // 出弯后丝滑加速
            Smooth_Speed_Control(ACCEL_DIST, 0);

            CurrentDistance = 0;
            current_point = POINT_B;
            LED_BUZZER_falg = 1;
        }
    }

    // B → D：直线无轨迹
    else if (current_point == POINT_B)
    {
        line_flag = -1; // 关闭左四路

        // 入弯前丝滑减速控制
        float distance_to_turn = DIST_BD + 8 - CurrentDistance;
        Smooth_Speed_Control(distance_to_turn, 0);

        if (CurrentDistance >= DIST_BD && is_valid_8_detected == 0)
        {
            // PID_SetTarget(&TurnPID, BaseAngle);
            CurrentDistance = 0;
            // 弯道中保持低速
            Smooth_Speed_Control(0, 1);
            current_point = POINT_D;
            LED_BUZZER_falg = 1;
        }
    }

    // D → A：灰度半圆循迹（正常循迹）
    else if (current_point == POINT_D)
    {
        SpeedPID.Enabled = 1;
        CarPID.Enabled = 1;
        TurnPID.Enabled = 0;

        if (CurrentDistance >= DIST_AC && is_valid_8_detected == 1)
        {
            CurrentDistance = 0;
            current_point = POINT_A;
            if (TaskMode == MODE_TASK4_4_ACBDA)
                cycle_flag = 1;
            LED_BUZZER_falg = 1;
            TaskMode = MODE_IDLE;
            // line_flag = 0; // 任务结束重置
        }
    }
}

// 任务4：8字连续4圈
void Task_4Fig8_4Cycles(void)
{
    Task_3ACBDA();
    if (cycle_flag)
    {
        cycle_count++;
        cycle_flag = 0;
        if (cycle_count >= 3)
        {
            cycle_count = 0;
            // LED_BUZZER_falg = 0;
            TaskMode = MODE_IDLE;
        }
        else
        {
            current_point = POINT_A;
            CurrentDistance = 0;

            Angle_Reset();
            TaskMode = MODE_TASK4_4_ACBDA;
        }
    }
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
