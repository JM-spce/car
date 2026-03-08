#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float ErrorIntMax;
	float ErrorIntMin;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;
	
	// 死区配置
	float DeadZone;         // 死区范围（对称）
	uint8_t DeadZoneEnable; // 死区使能标志
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif
