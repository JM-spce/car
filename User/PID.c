/**
 ******************************************************************************
 * @file    PID.c
 * @author  平衡小车项目组
 * @version V1.0
 * @date    2026-02-27
 * @brief   PID控制器实现文件 - 包含完整的PID算法实现
 ******************************************************************************
 * @details
 * 
 * 本文件实现了三种不同的PID控制算法：
 * 
 * 1. 位置式PID (PID_POSITION):
 *    - 输出绝对值
 *    - 适用于执行机构接受绝对位置指令的场合
 *    - 公式: Out = Kp*e + Ki*∫e*dt + Kd*de/dt
 * 
 * 2. 增量式PID (PID_INCREMENTAL):
 *    - 输出相对变化量
 *    - 适用于步进电机等增量控制设备
 *    - 公式: ΔOut = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 * 
 * 3. 微分先行PID (PID_DERIVATIVE_ON_MEASUREMENT):
 *    - 微分作用于测量值而非误差
 *    - 抗干扰能力强，推荐用于寻迹控制
 *    - 公式: Out = Kp*e + Ki*∫e*dt - Kd*dActual/dt
 * 
 * 特殊功能说明：
 * - 死区控制：当误差小于设定值时，输出为0，避免小幅震荡
 * - 积分分离：大误差时不进行积分，防止积分饱和和超调
 * - 输出限幅：确保输出在安全范围内
 ******************************************************************************
 */

#include "PID.h"
#include <stddef.h>

/**
 * @brief PID控制器初始化函数
 * @param pid 指向PID结构体的指针
 * @note 初始化所有参数为安全的默认值
 */
void PID_Init(PID_t *pid)
{
    // 空指针检查，防止程序崩溃
    if (pid == NULL) return;
    // ==================== 初始化PID核心参数 ====================
    pid->Kp = 0.0f;                   // 比例系数初始为0
    pid->Ki = 0.0f;                   // 积分系数初始为0
    pid->Kd = 0.0f;                   // 微分系数初始为0
    // ==================== 初始化控制目标 ====================
    pid->Target = 0.0f;               // 目标值设为0
    pid->Actual = 0.0f;               // 实际值设为0
    // ==================== 初始化误差历史 ====================
    pid->Error[0] = 0.0f;             // 当前误差
    pid->Error[1] = 0.0f;             // 上次误差
    pid->Error[2] = 0.0f;             // 上上次误差
    pid->ErrorInt = 0.0f;             // 积分累积值清零
    pid->ErrorDer = 0.0f;             // 微分值清零
    // ==================== 设置默认限制范围 ====================
    pid->OutMax = 100.0f;             // 输出最大值100%
    pid->OutMin = -100.0f;            // 输出最小值-100%
    pid->IntegralMax = 50.0f;         // 积分上限50%
    pid->IntegralMin = -50.0f;        // 积分下限-50%
    // ==================== 默认关闭高级功能 ====================
    pid->DeadZone = 0.0f;             // 死区范围为0
    pid->DeadZoneEn = 0;              // 关闭死区功能
    pid->IntegralSepThres = 10.0f;    // 积分分离阈值10
    pid->IntegralSepEn = 1;           // 开启积分分离功能
	// ==================== 默认PID使能 ====================
	pid->Enabled = 1;                 // 默认使能 PID
    // ==================== 初始化输出 ====================
    pid->Out = 0.0f;                  // 输出初值为0
    // ==================== 设置默认工作模式 ====================
    pid->Mode = PID_DERIVATIVE_ON_MEASUREMENT; // 默认使用微分先行模式（最适合寻迹）
}

/**
 * @brief 设置PID核心参数
 * @param pid 指向PID结构体的指针
 * @param Kp 比例系数 - 影响系统响应速度和稳定性
 * @param Ki 积分系数 - 影响稳态误差消除能力
 * @param Kd 微分系数 - 影响系统稳定性和抗干扰能力
 * @note 参数设置后立即生效
 */
void PID_SetParam(PID_t *pid, float Kp, float Ki, float Kd)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 更新PID参数
    pid->Kp = Kp;                     // 设置比例系数
    pid->Ki = Ki;                     // 设置积分系数
    pid->Kd = Kd;                     // 设置微分系数
}

/**
 * @brief 设置输出和积分限制范围
 * @param pid 指向PID结构体的指针
 * @param OutMax 输出最大值 - 防止执行机构过载
 * @param OutMin 输出最小值 - 防止执行机构反向过载
 * @param IntegralMax 积分最大值 - 防止积分饱和
 * @param IntegralMin 积分最小值 - 防止积分反向饱和
 * @note 通常根据具体应用场景调整这些参数
 */
void PID_SetLimits(PID_t *pid, float OutMax, float OutMin, float IntegralMax, float IntegralMin)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 设置输出限制
    pid->OutMax = OutMax;             // 设置输出上限
    pid->OutMin = OutMin;             // 设置输出下限
    
    // 设置积分限制
    pid->IntegralMax = IntegralMax;   // 设置积分上限
    pid->IntegralMin = IntegralMin;   // 设置积分下限
}

/**
 * @brief 设置死区控制参数
 * @param pid 指向PID结构体的指针
 * @param DeadZone 死区范围 - 当误差绝对值小于此值时视为无误差
 * @param Enable 使能开关 - 0:关闭死区 1:开启死区
 * @note 死区可以减少小幅震荡，提高系统稳定性
 */
void PID_SetDeadZone(PID_t *pid, float DeadZone, uint8_t Enable)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 设置死区参数
    pid->DeadZone = DeadZone;         // 设置死区范围
    pid->DeadZoneEn = Enable;         // 设置使能状态
}

/**
 * @brief 设置积分分离参数
 * @param pid 指向PID结构体的指针
 * @param Threshold 积分分离阈值 - 当误差绝对值大于此值时停止积分
 * @param Enable 使能开关 - 0:关闭积分分离 1:开启积分分离
 * @note 积分分离可以防止大误差时积分饱和，提高系统响应速度
 */
void PID_SetIntegralSep(PID_t *pid, float Threshold, uint8_t Enable)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 设置积分分离参数
    pid->IntegralSepThres = Threshold; // 设置分离阈值
    pid->IntegralSepEn = Enable;       // 设置使能状态
}

/**
 * @brief 设置PID工作模式
 * @param pid 指向PID结构体的指针
 * @param Mode PID工作模式枚举值
 * @note 切换模式时会自动清除PID的历史状态
 */
void PID_SetMode(PID_t *pid, PID_Mode_t Mode)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 设置新的工作模式
    pid->Mode = Mode;
    
    // 切换模式时清空状态，避免模式切换引起的问题
    PID_Clear(pid);
}

/**
 * @brief 设置PID目标值
 * @param pid 指向PID结构体的指针
 * @param Target 目标值 - 期望达到的位置或数值
 * @note 目标值可以根据实际控制需求动态调整
 */
void PID_SetTarget(PID_t *pid, float Target)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 设置新的目标值
    pid->Target = Target;
}

/**
 * @brief 清空PID控制器状态
 * @param pid 指向PID结构体的指针
 * @note 清除所有历史误差和积分累积值，相当于重新开始控制
 */
void PID_Clear(PID_t *pid)
{
    // 安全检查
    if (pid == NULL) return;
    
    // 清空误差历史
    pid->Error[0] = 0.0f;             // 当前误差清零
    pid->Error[1] = 0.0f;             // 上次误差清零
    pid->Error[2] = 0.0f;             // 上上次误差清零
    
    // 清空积分和微分
    pid->ErrorInt = 0.0f;             // 积分累积值清零
    pid->ErrorDer = 0.0f;             // 微分值清零
    
    // 输出清零
    pid->Out = 0.0f;                  // 最终输出清零
}

/**
 * @brief PID核心计算函数 - 实现三种PID算法
 * @param pid 指向PID结构体的指针
 * @param Actual 当前实际测量值
 * @note 这是PID控制的核心函数，每次调用完成一次完整的PID计算过程
 */
void PID_Update(PID_t *pid, float Actual)
{
    // 安全检查
    if (pid == NULL) return;
    
	// ==================== 使能判断 ====================
	//如果失能则OUT输出恒为0
	 if (!pid->Enabled)
    {
        pid->Out = 0.0f;
        return;
    }

    // ==================== 更新当前实际值 ====================
    pid->Actual = Actual;
    
    // ==================== 计算当前误差 ====================
    pid->Error[0] = pid->Target - pid->Actual; // 误差 = 目标值 - 实际值
    
    // ==================== 死区处理 ====================
    // 如果开启了死区且误差在死区范围内，则输出为0
    if (pid->DeadZoneEn && fabs(pid->Error[0]) <= pid->DeadZone)
    {
        pid->Out = 0.0f;              // 输出置零
        pid->ErrorInt = 0.0f;         // 清空积分（避免死区积累）
        
        // 更新误差历史记录
        pid->Error[2] = pid->Error[1];
        pid->Error[1] = pid->Error[0];
        return;                       // 直接返回，不进行PID计算
    }
    
    // ==================== 根据不同模式计算PID输出 ====================
    switch (pid->Mode)
    {
        // ==================== 位置式PID ====================
        case PID_POSITION:
        {
            // 积分计算（带积分分离功能）
            if (pid->IntegralSepEn && fabs(pid->Error[0]) > pid->IntegralSepThres)
            {
                // 误差过大时，暂停积分以防止积分饱和
                pid->ErrorInt += 0.0f; // 不进行积分
            }
            else
            {
                // 正常积分计算
                pid->ErrorInt += pid->Error[0] * pid->Ki; // 积分项累加
                
                // 积分限幅，防止积分饱和
                pid->ErrorInt = fmin(fmax(pid->ErrorInt, pid->IntegralMin), pid->IntegralMax);
            }
            
            // 微分计算（基于误差变化率）
            pid->ErrorDer = (pid->Error[0] - pid->Error[1]) * pid->Kd;
            
            // 位置式PID完整计算公式
            pid->Out = pid->Kp * pid->Error[0] + pid->ErrorInt + pid->ErrorDer;
            break;
        }
        
        // ==================== 增量式PID ====================
        case PID_INCREMENTAL:
        {
            // 增量式PID计算（只计算输出的变化量）
            float delta = pid->Kp * (pid->Error[0] - pid->Error[1]) +      // 比例增量
                          pid->Ki * pid->Error[0] +                        // 积分增量
                          pid->Kd * (pid->Error[0] - 2 * pid->Error[1] + pid->Error[2]); // 微分增量
            
            // 增量叠加到原输出上
            pid->Out += delta;
            break;
        }
        
        // ==================== 微分先行PID（推荐）====================
        case PID_DERIVATIVE_ON_MEASUREMENT:
        {
            // 比例项计算
            float p_term = pid->Kp * pid->Error[0];
            
            // 积分项计算（带积分分离）
            float i_term = 0.0f;
            if (pid->IntegralSepEn && fabs(pid->Error[0]) <= pid->IntegralSepThres)
            {
                // 误差较小时才进行积分
                pid->ErrorInt += pid->Error[0] * pid->Ki; // 积分累加
                pid->ErrorInt = fmin(fmax(pid->ErrorInt, pid->IntegralMin), pid->IntegralMax); // 积分限幅
                i_term = pid->ErrorInt; // 积分项输出
            }
            // 误差较大时，积分项为0（积分分离效果）
            
            // 微分项计算（关键改进：微分作用于实际测量值而不是误差）
            // 这样可以有效抑制测量噪声的影响
            float d_term = -pid->Kd * (pid->Actual - pid->Error[1]); // 注意：Error[1]存储的是上次的实际值
            
            // PID总输出计算
            pid->Out = p_term + i_term + d_term;
            
            // 保存当前实际值，供下次微分计算使用
            pid->Error[1] = pid->Actual;
            break;
        }
    }
    
    // ==================== 输出限幅 ====================
    // 确保最终输出在设定的安全范围内
    pid->Out = fmin(fmax(pid->Out, pid->OutMin), pid->OutMax);
    
    // ==================== 更新误差历史 ====================
    // 为下次计算准备误差数据
    pid->Error[2] = pid->Error[1];    // 上上次误差更新
    pid->Error[1] = pid->Error[0];    // 上次误差更新
}
