#ifndef __HANDFILE_H
#define __HANDFILE_H

#include "stm32f10x.h"
#include "BlueSerial.h"
#include "Delay.h"
#include "Encoder.h"
#include "Key.h"
#include "LED.h"
#include "BUZZER.h"
#include "MPU6050.h"
#include "Motor.h"
#include "OLED.h"
#include "PID.h"
#include "Serial.h"
#include "hc4051.h"
#include "Timer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    MODE_IDLE          = 0, // 待机模式（未选择）
    MODE_TASK1_AB      = 1, // 模式 1：A→B 单次行驶
    MODE_TASK2_ABCDA   = 2, // 模式 2：ABCDA 一圈循环
    MODE_TASK3_ACBDA   = 3, // 模式 3：8 字路径（A→C→B→D→A）
    MODE_TASK4_4_ACBDA = 4, // 模式 4：8 字路径连续 4 圈
    MODE_Test          = 5  // 测试模式
} CarTaskMode;

typedef enum {
    STATE_SYSTEM_INIT,    // 系统初始化（传感器、电机、PID）
    STATE_TASK_SELECT,    // 任务模式选择（按键交互）
    STATE_LINE_TRACKING,  // 黑线循迹（灰度传感器主导）
    STATE_ANGLE_CRUISE,   // 角度巡航（陀螺仪定角行驶）
    STATE_POINT_STOPPING, // 定点停车（A/B/C/D 点声光提示）
    STATE_TASK_COMPLETE,  // 任务完成（全局停车 + 结果显示）
    STATE_FAULT_RECOVERY  // 故障恢复（脱线/漂移后的自救）
} CarCoreState;

#define PWM_MAX        800  // 电机最大 PWM
#define PWM_MIN        200  // 电机最小 PWM（避免停转）
#define THRESHOLD      620  // 灰度传感器阈值
#define CONTROL_PERIOD 1    // 控制周期（10ms）
#define DEAD_ZONE      1.0f // PID 死区（避免小幅震荡）

#define DIST_AB        90.0f  // A→B 直线距离
#define DIST_BC        120.0f // B→C 圆弧距离
#define DIST_CD        90.0f // C→D 直线距离
#define DIST_DA        120.0f // D→A 圆弧距离
#define DIST_AC        128.0f // A→C 直线

// 减速区距离（入弯前提前减速的距离）
#define DECEL_DIST     20.0f  // 提前 20cm 减速
#define ACCEL_DIST     10.0f  // 出弯后 10cm 加速

typedef enum {
    POINT_A = 0,
    POINT_B,
    POINT_C,
    POINT_D,
    POINT_UNKNOWN
} Point;

// 传感器权重（0-7 路对应位置 10-80）
extern const int8_t sensor_weight[8];

// ====================== 全局变量声明 ======================
extern uint8_t KeyNum;
extern uint8_t PhysicalKey;
extern uint8_t RunFlag;
extern uint8_t updata_count;
extern uint16_t adc_values[8];
extern uint8_t binary_values[8];
extern int16_t LeftPWM, RightPWM;
extern int16_t line_pos;
extern uint8_t valid_sensor_count;
extern int8_t line_flag;
extern int16_t AX, AY, AZ, GX, GY, GZ;
extern int16_t LeftEncoder, RightEncoder;
extern float LeftSpeed, RightSpeed;
extern float AveSpeed;
extern float Angle;
extern float AngleIntegral;
extern float BaseAngle;
extern float gyro_z_filtered;
extern PID_t SpeedPID;
extern PID_t TurnPID;
extern PID_t CarPID;
extern CarTaskMode TaskMode;

// ====================== 函数声明 ======================

void Init(void);
void BlueSerial(void);
void Key_action(void);
void LED_BUZZER(void);
void Data_Update(void);

void Angle_Normalize(void);
void Angle_Reset(void);
void Distance_Update(void);
void Calculate_Line_Position(int16_t *line_pos, uint8_t *valid_sensor);
void Update_Sensor_Status(void);

void Task_Idle(void);
void Task_1AB(void);
void Task_2ABCDA(void);
void Task_3ACBDA(void);
void Task_4Fig8_4Cycles(void);

void black_line_task(void);
void turn_task(void);

#endif
