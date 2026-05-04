#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "JY901PSerial.h"
#include "JY901P.h"

char JY901PSerial_RxPacket[100];
uint8_t JY901PSerial_RxFlag;

JY901PConnectState JY901PConnectionState = JY901P_DISCONNECTED;

void JY901PSerial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2, ENABLE);

	JY901PConnectionState = JY901P_DISCONNECTED;
}

void JY901PSerial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void JY901PSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		JY901PSerial_SendByte(Array[i]);
	}
}

void JY901PSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		JY901PSerial_SendByte(String[i]);
	}
}

uint32_t JY901PSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void JY901PSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		JY901PSerial_SendByte(Number / JY901PSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void JY901PSerial_Printf(char *format, ...)
{
	if (JY901PConnectionState == JY901P_CONNECTED)
	{
		char String[100];
		va_list arg;
		va_start(arg, format);
		vsprintf(String, format, arg);
		va_end(arg);
		JY901PSerial_SendString(String);
	}
}

JY901PConnectState JY901PSerial_GetConnectionState(void)
{
	return JY901PConnectionState;
}

void JY901PSerial_SetConnected(void)
{
	JY901PConnectionState = JY901P_CONNECTED;
}

void JY901PSerial_SetDisconnected(void)
{
	JY901PConnectionState = JY901P_DISCONNECTED;
}

void JY901P_Calibrate_ZAxis(void)
{
    uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t calibrate_cmd[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
    uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    
    JY901PSerial_SendArray(unlock_cmd, 5);
    
    for (volatile uint32_t i = 0; i < 100000; i++);
    
    JY901PSerial_SendArray(calibrate_cmd, 5);
    
    for (volatile uint32_t i = 0; i < 100000; i++);
    
    JY901PSerial_SendArray(save_cmd, 5);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART2);
		JY901PConnectionState = JY901P_CONNECTED;
        IMU_ParseData(data);
    }
}
