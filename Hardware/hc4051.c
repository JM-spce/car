#include "hc4051.h"
#include "Delay.h"

// 添加状态变量
static uint8_t current_channel = 0xFF;  // 记录当前选中的通道

// 初始化GPIO和ADC1
void HC4051_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // 使能ADC1时钟
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);                     // ADC时钟分频（72/6=12MHz，符合ADC要求）
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  // 禁用JTAG，释放PB3/PB4/PB5用于GPIO

    // 2. 配置PB3/PB4/PB5为推挽输出（A0/A1/A2）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;         // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 3. 配置PA4为模拟输入（ADC通道）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;            // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 4. 初始化ADC1
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;       // 独立模式
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;            // 关闭扫描模式
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;      // 单次转换模式
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;   // 数据右对齐
    ADC_InitStruct.ADC_NbrOfChannel = 1;                  // 单次转换通道数
    ADC_Init(ADC1, &ADC_InitStruct);
    
    // 5. 使能ADC1
    ADC_Cmd(ADC1, ENABLE);
    
    // 6. ADC校准
    ADC_ResetCalibration(ADC1);                          // 重置校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));          // 等待校准重置完成
    ADC_StartCalibration(ADC1);                          // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));               // 等待校准完成
    
    // 7. 初始化完成后将所有控制引脚置低（选择CH1）
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    current_channel = HC4051_CH1;
}

// 选择4051通道（0-7）
void HC4051_SelectChannel(uint8_t ch)
{
    // 参数验证
    if(ch > 7) return;
    
    // 如果已经是目标通道，则不重复设置
    if(current_channel == ch) return;

    // 通道对应关系：
    // CH1: 0(000) -> A2=0, A1=0, A0=0
    // CH2: 1(001) -> A2=0, A1=0, A0=1  
    // CH3: 2(010) -> A2=0, A1=1, A0=0
    // CH4: 3(011) -> A2=0, A1=1, A0=1
    // CH5: 4(100) -> A2=1, A1=0, A0=0
    // CH6: 5(101) -> A2=1, A1=0, A0=1
    // CH7: 6(110) -> A2=1, A1=1, A0=0
    // CH8: 7(111) -> A2=1, A1=1, A0=1
    
    if(ch & 0x01) GPIO_SetBits(GPIOB, GPIO_Pin_3);
    else GPIO_ResetBits(GPIOB, GPIO_Pin_3);
    
    if(ch & 0x02) GPIO_SetBits(GPIOB, GPIO_Pin_4);
    else GPIO_ResetBits(GPIOB, GPIO_Pin_4);
    
    if(ch & 0x04) GPIO_SetBits(GPIOB, GPIO_Pin_5);
    else GPIO_ResetBits(GPIOB, GPIO_Pin_5);

    
    current_channel = ch;
    Delay_us(1);
}

uint16_t HC4051_ReadADC(uint8_t ch)
{
    if(ch > 7) return 0;
    
    HC4051_SelectChannel(ch);  //选择通道
    
    // 配置ADC通道4，采样时间55.5周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);         // 软件触发ADC转换
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));  // 等待转换完成
    
    return ADC_GetConversionValue(ADC1);  // 返回ADC结果
}

// 批量读取所有通道
void HC4051_ReadAllChannels(uint16_t* values)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        values[i] = HC4051_ReadADC(i);
    }
}

// 获取当前选中通道
uint8_t HC4051_GetCurrentChannel(void)
{
    return current_channel;
}

void HC4051_ReadBinaryChannels(uint16_t* adc_values, uint8_t* binary_values, uint16_t threshold)
{
    HC4051_ReadAllChannels(adc_values);
    
    for(uint8_t i = 0; i < 8; i++)
    {
        if(adc_values[i] > threshold)
        {
            binary_values[i] = 1;
        }
        else
        {
            binary_values[i] = 0;
        }
    }
}
