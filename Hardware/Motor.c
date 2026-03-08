#include "stm32f10x.h"                  
#include "PWM.h"

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    PWM_Init();
}

void Motor_SetPWM(uint8_t n, int16_t PWM)
{
    // 修复2：限幅PWM值到1000步范围（0~999），避免超界
    uint16_t pwm_val = (PWM >= 0) ? PWM : -PWM;
    pwm_val = (pwm_val > 999) ? 999 : pwm_val; // 限制最大为999
    
    if (n == 1) // 左电机
    {
        if (PWM >= 0) // 前进
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_12);
            GPIO_ResetBits(GPIOB, GPIO_Pin_13);
            PWM_SetCompare1(pwm_val);
        }
        else // 后退
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            GPIO_SetBits(GPIOB, GPIO_Pin_13);
            PWM_SetCompare1(pwm_val);
        }
    }
    else if (n == 2) // 右电机
    {
        if (PWM >= 0) // 前进
        {
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            PWM_SetCompare2(pwm_val);
        }
        else // 后退
        {
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            PWM_SetCompare2(pwm_val);
        }
    }
}
