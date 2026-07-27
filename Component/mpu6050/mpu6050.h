#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <math.h>

// 陀螺仪量程系数 (对应 ±2000°/s)
#define GYRO_SCALE       16.384f   // LSB/(°/s)  = 32768 / 2000
// 加速度计量程系数 (对应 ±8g)
#define ACCEL_SCALE      4096.0f   // LSB/g      = 32768 / 8

void  MPU6050_Init(void);
void  MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz);
void  MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az);
void  MPU6050_GetRaw(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);

// 从加速度计计算姿态角 (pitch/roll, 单位 °)
void  MPU6050_GetAngleAccel(float *pitch, float *roll);

// 互补滤波融合角度 (需周期调用, dt 单位为秒)
void  MPU6050_ComplFilter(float dt,
                          int16_t gx, int16_t gy, int16_t gz,
                          int16_t ax, int16_t ay, int16_t az,
                          float *pitch, float *roll);

// 一阶低通滤波 (用于原始数据平滑)
void  MPU6050_LPF(float input, float *output, float alpha);

#endif
