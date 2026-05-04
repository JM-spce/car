#ifndef __JY901P_SERIAL_H
#define __JY901P_SERIAL_H

#include <stdio.h>

extern char JY901PSerial_RxPacket[];
extern uint8_t JY901PSerial_RxFlag;
typedef enum {
    JY901P_DISCONNECTED = 0,
    JY901P_CONNECTED = 1
} JY901PConnectState;
extern JY901PConnectState JY901PConnectionState;

void JY901PSerial_Init(void);

void JY901PSerial_SendByte(uint8_t Byte);
void JY901PSerial_SendArray(uint8_t *Array, uint16_t Length);
void JY901PSerial_SendString(char *String);
void JY901PSerial_SendNumber(uint32_t Number, uint8_t Length);
void JY901PSerial_Printf(char *format, ...);

JY901PConnectState JY901PSerial_GetConnectionState(void);
void JY901PSerial_SetConnected(void);
void JY901PSerial_SetDisconnected(void);
void JY901P_Calibrate_ZAxis(void);
#endif
