// MPU6050 驱动 — MSPM0 SDK 2.04 硬件 I2C0
// 所有 I2C 传输用 STOP-START 方式 (不用 Repeated Start)

#include "mpu6050.h"
#include "ti_msp_dl_config.h"

#define MPU_ADDR  0x68

#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

/* ── 等待 I2C 传输完成 ── */
static void i2c_wait_idle(void)
{
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}
}

/* ── 写寄存器 ── */
static bool i2c_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    i2c_wait_idle();
    return !(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR);
}

/* ── 读多字节: 先写寄存器地址, STOP, 再读 ── */
static bool i2c_read_multi(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 步骤1: 写寄存器地址 + STOP */
    uint8_t reg_buf[1] = {reg};
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, reg_buf, 1);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    i2c_wait_idle();
    if (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return false;

    /* 步骤2: 读 len 字节 + STOP */
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (uint8_t i = 0; i < len; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            if (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return false;
            if (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) {
                /* 传输结束但 RX FIFO 还空 = 没收到数据 */
                return false;
            }
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    i2c_wait_idle();
    return !(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR);
}

/* ═══════════════════════════════════════════
   公开接口
   ═══════════════════════════════════════════ */

void MPU6050_Init(void)
{
    volatile uint32_t i;
    for (i = 0; i < 800000; i++);  /* ~100ms */
    i2c_write(PWR_MGMT_1,   0x00);
    i2c_write(SMPLRT_DIV,   0x07);
    i2c_write(CONFIG,       0x06);
    i2c_write(GYRO_CONFIG,  0x18);
    i2c_write(ACCEL_CONFIG, 0x10);
}

void MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    i2c_read_multi(GYRO_XOUT_H, buf, 6);
    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    i2c_read_multi(ACCEL_XOUT_H, buf, 6);
    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

void MPU6050_GetRaw(int16_t *ax, int16_t *ay, int16_t *az,
                    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    i2c_read_multi(ACCEL_XOUT_H, buf, 14);
    *ax = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay = (int16_t)((buf[2]  << 8) | buf[3]);
    *az = (int16_t)((buf[4]  << 8) | buf[5]);
    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
}

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

void MPU6050_ComplFilter(float dt,
                         int16_t gx, int16_t gy, int16_t gz,
                         int16_t ax, int16_t ay, int16_t az,
                         float *pitch, float *roll)
{
    float gx_dps = (float)gx / GYRO_SCALE;
    float gy_dps = (float)gy / GYRO_SCALE;
    float gyro_pitch = *pitch + gy_dps * dt;
    float gyro_roll  = *roll  + gx_dps * dt;
    float ax_g = (float)ax / ACCEL_SCALE;
    float ay_g = (float)ay / ACCEL_SCALE;
    float az_g = (float)az / ACCEL_SCALE;
    float accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;
    float accel_roll  = atan2f( ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 57.29578f;
    const float alpha = 0.98f;
    *pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    *roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
}

void MPU6050_LPF(float input, float *output, float alpha)
{
    *output = alpha * input + (1.0f - alpha) * (*output);
}
