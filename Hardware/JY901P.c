#include "JY901P.h"
#include "delay.h"

volatile EulerAngle_Struct JY901P;
float ang_offset = 0.0f;

static uint8_t rx_buffer[11];
static volatile uint8_t rx_state = 0;
static uint8_t rx_index = 0;

void IMU_ParseData(uint8_t data)
{
    uint8_t i, sum = 0;
    
    switch (rx_state)
    {
        case 0:
            if (data == 0x55)
            {
                rx_buffer[0] = data;
                rx_state = 1;
                rx_index = 1;
            }
            break;
            
        case 1:
            if (data == 0x51 || data == 0x52 || data == 0x53 || data == 0x59)
            {
                rx_buffer[1] = data;
                rx_state = 2;
                rx_index = 2;
            }
            else
            {
                rx_state = 0;
                rx_index = 0;
            }
            break;
            
        case 2:
            rx_buffer[rx_index++] = data;
            if (rx_index == 11)
            {
                for (i = 0; i < 10; i++)
                {
                    sum += rx_buffer[i];
                }
                
                if (sum == rx_buffer[10])
                {
                    switch (rx_buffer[1])
                    {
                        case 0x53:
                            JY901P.roll = ((int16_t)((rx_buffer[3] << 8) | rx_buffer[2])) / 32768.0f * 180.0f;
                            JY901P.pitch = ((int16_t)((rx_buffer[5] << 8) | rx_buffer[4])) / 32768.0f * 180.0f;
                            JY901P.yaw = ((int16_t)((rx_buffer[7] << 8) | rx_buffer[6])) / 32768.0f * 180.0f;
                            break;
                            
                        case 0x51:
                            JY901P.acc_x = ((int16_t)((rx_buffer[3] << 8) | rx_buffer[2]));
                            JY901P.acc_y = ((int16_t)((rx_buffer[5] << 8) | rx_buffer[4]));
                            JY901P.acc_z = ((int16_t)((rx_buffer[7] << 8) | rx_buffer[6]));
                            break;
                            
                        case 0x52:
                            JY901P.gyro_x = ((int16_t)((rx_buffer[3] << 8) | rx_buffer[2])) / 32768.0f * 2000.0f;
                            JY901P.gyro_y = ((int16_t)((rx_buffer[5] << 8) | rx_buffer[4])) / 32768.0f * 2000.0f;
                            JY901P.gyro_z = ((int16_t)((rx_buffer[7] << 8) | rx_buffer[6])) / 32768.0f * 2000.0f;
                            break;
                            
                        case 0x59:
                            JY901P.q0 = ((int16_t)((rx_buffer[3] << 8) | rx_buffer[2])) / 32768.0f;
                            JY901P.q1 = ((int16_t)((rx_buffer[5] << 8) | rx_buffer[4])) / 32768.0f;
                            JY901P.q2 = ((int16_t)((rx_buffer[7] << 8) | rx_buffer[6])) / 32768.0f;
                            JY901P.q3 = ((int16_t)((rx_buffer[9] << 8) | rx_buffer[8])) / 32768.0f;
                            break;
                    }
                }
                
                rx_state = 0;
                rx_index = 0;
                sum = 0;
            }
            break;
            
        default:
            rx_state = 0;
            rx_index = 0;
            break;
    }
}

EulerAngle_Struct IMU_GetEulerAngle(void)
{
    return JY901P;
}

void IMU_YawCalibrate(void)
{
    const uint16_t sample_count = 1000;
    float sum = 0.0f;
    uint16_t i;
    
    for (i = 0; i < sample_count; i++)
    {
        sum += JY901P.yaw;
    }
    
    ang_offset = sum / (float)sample_count;
}

