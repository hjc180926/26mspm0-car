// MPU6050 驱动 — MSPM0 移植版
// 支持：初始化、原始数据读取、加速度计角度、互补滤波融合
#include "mpu6050.h"
#include "IIC.h"

/* MPU6050 寄存器地址 */
#define MPU_ADDR        0xD0        // 7 位地址 0x68 左移 1 位
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43

/* 内部函数：写一个寄存器 */
static void write_reg(uint8_t reg, uint8_t val)
{
    IIC_Start();
    IIC_Send_Byte(MPU_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(val);
    IIC_Wait_Ack();
    IIC_Stop();
}

/* 内部函数：读一个寄存器 */
static uint8_t read_reg(uint8_t reg)
{
    uint8_t val;
    IIC_Start();
    IIC_Send_Byte(MPU_ADDR);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();                        // 重复起始
    IIC_Send_Byte(MPU_ADDR | 0x01);     // 读地址
    IIC_Wait_Ack();
    val = IIC_Read_Byte(0);             // 非应答
    IIC_Stop();
    return val;
}

/* 初始化 MPU6050 */
void MPU6050_Init(void)
{
    IIC_Init();

    // 等待 MPU6050 上电稳定 (~100ms)
    volatile uint32_t i;
    for (i = 0; i < 800000; i++);

    // 解除休眠模式 (PWR_MGMT_1 写 0)
    write_reg(PWR_MGMT_1, 0x00);

    // 采样率分频: 1kHz / (1 + 7) = 125Hz
    write_reg(SMPLRT_DIV, 0x07);

    // 数字低通滤波器: 带宽 5Hz (最大程度降噪)
    write_reg(CONFIG, 0x06);

    // 陀螺仪 ±2000°/s: FS_SEL = 3 (即 0x18)
    write_reg(GYRO_CONFIG, 0x18);

    // 加速度计 ±8g: AFS_SEL = 2 (即 0x10)
    write_reg(ACCEL_CONFIG, 0x10);
}

/* 读陀螺仪原始数据 */
void MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++)
        buf[i] = read_reg(GYRO_XOUT_H + i);
    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

/* 读加速度计原始数据 */
void MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++)
        buf[i] = read_reg(ACCEL_XOUT_H + i);
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

/* 同时读取全部 6 轴数据 (效率高于分别调用) */
void MPU6050_GetRaw(int16_t *ax, int16_t *ay, int16_t *az,
                    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    for (uint8_t i = 0; i < 14; i++)
        buf[i] = read_reg(ACCEL_XOUT_H + i);
    *ax = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay = (int16_t)((buf[2]  << 8) | buf[3]);
    *az = (int16_t)((buf[4]  << 8) | buf[5]);
    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

/* 仅用加速度计计算俯仰 / 横滚角 (单位 °) */
void MPU6050_GetAngleAccel(float *pitch, float *roll)
{
    int16_t ax, ay, az;
    MPU6050_GetAccel(&ax, &ay, &az);

    float ax_g = (float)ax / ACCEL_SCALE;
    float ay_g = (float)ay / ACCEL_SCALE;
    float az_g = (float)az / ACCEL_SCALE;

    *pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;
    *roll  = atan2f( ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 57.29578f;
}

/* 互补滤波：融合陀螺仪积分 + 加速度计姿态 */
// dt 是两次调用之间的时间间隔 (秒)
// pitch/roll 为输入输出参数：传入上次角度，传出更新后的角度
void MPU6050_ComplFilter(float dt,
                         int16_t gx, int16_t gy, int16_t gz,
                         int16_t ax, int16_t ay, int16_t az,
                         float *pitch, float *roll)
{
    // 陀螺仪积分角度增量 (°)
    float gx_dps = (float)gx / GYRO_SCALE;   // °/s
    float gy_dps = (float)gy / GYRO_SCALE;

    float gyro_pitch = *pitch + gy_dps * dt;  // 注意：gy 绕 Y 轴对应 pitch
    float gyro_roll  = *roll  + gx_dps * dt;  // gx 绕 X 轴对应 roll

    // 加速度计计算角度
    float ax_g = (float)ax / ACCEL_SCALE;
    float ay_g = (float)ay / ACCEL_SCALE;
    float az_g = (float)az / ACCEL_SCALE;

    float accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;
    float accel_roll  = atan2f( ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 57.29578f;

    // 互补滤波: 角度 = 0.98 × 陀螺仪积分 + 0.02 × 加速度计
    // 系数可调：增大 alpha 更依赖陀螺仪（响应快但漂移），减小更依赖加速度计（稳定但滞后）
    const float alpha = 0.98f;
    *pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    *roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
}

/* 一阶低通滤波 */
// alpha 越小滤波越强 (反应越慢)
void MPU6050_LPF(float input, float *output, float alpha)
{
    *output = alpha * input + (1.0f - alpha) * (*output);
}
