#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include <stdio.h>

extern char BlueSerial_RxPacket[];
extern uint8_t BlueSerial_RxFlag;
typedef enum {
    BLUE_DISCONNECTED = 0,
    BLUE_CONNECTED = 1
} BlueConnectState;
extern BlueConnectState BlueConnectionState;

void BlueSerial_Init(void);

void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

BlueConnectState BlueSerial_GetConnectionState(void);
void BlueSerial_SetConnected(void);
void BlueSerial_SetDisconnected(void);
#endif
