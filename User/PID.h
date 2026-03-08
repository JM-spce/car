/**
 ******************************************************************************
 * @file    PID.h
 * @author  平衡小车项目组
 * @version V1.0
 * @date    2026-02-27
 * @brief   PID控制器头文件 - 包含完整的PID控制算法实现
 ******************************************************************************
 * @attention
 *
 * 本文件实现了三种PID控制模式：
 * 1. 位置式PID - 经典PID算法
 * 2. 增量式PID - 只输出增量值
 * 3. 微分先行PID - 推荐用于寻迹控制，抗干扰能力强
 *
 * 特色功能：
 * - 死区控制：避免小误差时的频繁调节
 * - 积分分离：大误差时不积分，防止积分饱和
 * - 输出限幅：防止输出超出合理范围
 * - 参数动态调整：运行时可修改PID参数
 ******************************************************************************
 */

#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"
#include <math.h>

/**
 * @brief PID控制模式枚举定义
 */
typedef enum {
    PID_POSITION = 0,                 /*!< 位置式PID - 输出绝对值 */
    PID_INCREMENTAL = 1,              /*!< 增量式PID - 输出相对变化量 */
    PID_DERIVATIVE_ON_MEASUREMENT = 2 /*!< 微分先行PID - 微分作用于测量值，抗干扰性强（推荐） */
} PID_Mode_t;

/**
 * @brief PID控制器结构体定义
 * 
 * 该结构体包含了PID控制所需的所有参数和状态变量
 */
typedef struct {
    // ==================== 核心PID参数 ====================
    float Kp;                         /*!< 比例系数 - 决定响应速度和稳态精度 */
    float Ki;                         /*!< 积分系数 - 消除稳态误差 */
    float Kd;                         /*!< 微分系数 - 预测趋势，抑制超调 */
    
    // ==================== 控制目标与反馈 ====================
    float Target;                     /*!< 目标值 - 期望达到的目标位置/数值 */
    float Actual;                     /*!< 实际值 - 当前测量到的实际位置/数值 */
    
    // ==================== 误差计算相关 ====================
    float Error[3];                   /*!< 误差历史数组 [0]:当前误差 [1]:上次误差 [2]:上上次误差 */
    float ErrorInt;                   /*!< 积分累积值 - 所有历史误差的累加 */
    float ErrorDer;                   /*!< 微分值 - 误差变化率 */
    
    // ==================== 输出限制 ====================
    float OutMax;                     /*!< 输出上限 - PID输出的最大值 */
    float OutMin;                     /*!< 输出下限 - PID输出的最小值 */
    
    // ==================== 积分限制 ====================
    float IntegralMax;                /*!< 积分上限 - 防止积分饱和 */
    float IntegralMin;                /*!< 积分下限 - 防止积分反向饱和 */
    
    // ==================== 死区控制 ====================
    float DeadZone;                   /*!< 死区范围 - 误差在此范围内时认为无偏差 */
    uint8_t DeadZoneEn;               /*!< 死区使能 - 0:关闭 1:开启 */
    
    // ==================== 积分分离 ====================
    float IntegralSepThres;           /*!< 积分分离阈值 - 误差大于此值时停止积分 */
    uint8_t IntegralSepEn;            /*!< 积分分离使能 - 0:关闭 1:开启 */
        
    // ==================== PID使能 ====================
	uint8_t Enabled;                  /*!< 使能标志 - 1:使能，0:失能（失能时输出恒为 0） */
	
    // ==================== 输出结果 ====================
    float Out;                        /*!< PID最终输出值 - 用于控制执行机构 */
    
    // ==================== 工作模式 ====================
    PID_Mode_t Mode;                  /*!< PID工作模式选择 */
} PID_t;

// ==================== 函数声明 ====================

/**
 * @brief PID控制器初始化
 * @param pid PID结构体指针
 * @note 初始化所有参数为默认值，设置为微分先行模式
 */
void PID_Init(PID_t *pid);

/**
 * @brief 设置PID核心参数
 * @param pid PID结构体指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 */
void PID_SetParam(PID_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief 设置输出和积分限制范围
 * @param pid PID结构体指针
 * @param OutMax 输出最大值
 * @param OutMin 输出最小值
 * @param IntegralMax 积分最大值
 * @param IntegralMin 积分最小值
 */
void PID_SetLimits(PID_t *pid, float OutMax, float OutMin, float IntegralMax, float IntegralMin);

/**
 * @brief 设置死区参数
 * @param pid PID结构体指针
 * @param DeadZone 死区范围
 * @param Enable 使能开关 0:关闭 1:开启
 */
void PID_SetDeadZone(PID_t *pid, float DeadZone, uint8_t Enable);

/**
 * @brief 设置积分分离参数
 * @param pid PID结构体指针
 * @param Threshold 积分分离阈值
 * @param Enable 使能开关 0:关闭 1:开启
 */
void PID_SetIntegralSep(PID_t *pid, float Threshold, uint8_t Enable);

/**
 * @brief 设置PID工作模式
 * @param pid PID结构体指针
 * @param Mode 工作模式枚举值
 * @note 切换模式时会自动清空PID状态
 */
void PID_SetMode(PID_t *pid, PID_Mode_t Mode);

/**
 * @brief 设置PID目标值
 * @param pid PID结构体指针
 * @param Target 目标值
 */
void PID_SetTarget(PID_t *pid, float Target);

/**
 * @brief PID核心计算函数
 * @param pid PID结构体指针
 * @param Actual 当前实际测量值
 * @note 这是PID控制的核心函数，每次调用完成一次PID计算
 */
void PID_Update(PID_t *pid, float Actual);

/**
 * @brief 清空PID状态
 * @param pid PID结构体指针
 * @note 清除所有误差历史和积分累积值
 */
void PID_Clear(PID_t *pid);

#endif
