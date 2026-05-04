#ifndef __JY901P_H
#define __JY901P_H

#include "stm32f10x.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float q0;
    float q1;
    float q2;
    float q3;
} EulerAngle_Struct;

void IMU_ParseData(uint8_t data);
void IMU_YawCalibrate(void);
EulerAngle_Struct IMU_GetEulerAngle(void);

extern volatile EulerAngle_Struct JY901P;
extern float ang_offset;

#endif
