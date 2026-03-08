#ifndef __HC4051_H
#define __HC4051_H

#include "stm32f10x.h"

// ==================== 硬件连接配置 ====================
// 地址线连接配置
#define HC4051_A0_PORT    GPIOB
#define HC4051_A0_PIN     GPIO_Pin_3
#define HC4051_A1_PORT    GPIOB  
#define HC4051_A1_PIN     GPIO_Pin_4
#define HC4051_A2_PORT    GPIOB
#define HC4051_A2_PIN     GPIO_Pin_5

// ADC输入配置
#define HC4051_ADC_PORT   GPIOA
#define HC4051_ADC_PIN    GPIO_Pin_4
#define HC4051_ADC_CHANNEL ADC_Channel_4

// ==================== 通道定义 ====================
#define HC4051_CH1    0    // 000       // CH1: 0(000) -> A2=0, A1=0, A0=0 
#define HC4051_CH2    1    // 001       // CH2: 1(001) -> A2=0, A1=0, A0=1  
#define HC4051_CH3    2    // 010       // CH3: 2(010) -> A2=0, A1=1, A0=0
#define HC4051_CH4    3    // 011       // CH4: 3(011) -> A2=0, A1=1, A0=1
#define HC4051_CH5    4    // 100       // CH5: 4(100) -> A2=1, A1=0, A0=0
#define HC4051_CH6    5    // 101       // CH6: 5(101) -> A2=1, A1=0, A0=1
#define HC4051_CH7    6    // 110       // CH7: 6(110) -> A2=1, A1=1, A0=0
#define HC4051_CH8    7    // 111       // CH8: 7(111) -> A2=1, A1=1, A0=1

// ==================== 函数声明 ====================
void HC4051_Init(void);
void HC4051_SelectChannel(uint8_t ch);
uint16_t HC4051_ReadADC(uint8_t ch);
void HC4051_ReadAllChannels(uint16_t* values);
uint8_t HC4051_GetCurrentChannel(void);
void HC4051_ReadBinaryChannels(uint16_t* adc_values, uint8_t* binary_values, uint16_t threshold);

#endif
