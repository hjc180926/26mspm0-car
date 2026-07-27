/**
 * @file    main.c
 * @brief   M0 - 速度环 + MPU6050 三轴角度显示 (硬件I2C0)
 */

#include "ti_msp_dl_config.h"
#include "led/led.h"
#include "led/key.h"
#include "UART/uart0.h"
#include "tb6612/tb6612.h"
#include "encoder/encoder.h"
#include "oled/oled.h"
#include "service/speed_service.h"
#include "common/util.h"
#include "mpu6050/mpu6050.h"
#include <stdio.h>
#include <stdint.h>

static volatile uint32_t g_tick_10ms = 0;

static const int32_t g_gear[] = {0, 30, 200, 400, -20, -150, -400};
#define GEAR_COUNT (sizeof(g_gear) / sizeof(g_gear[0]))
static uint32_t g_speed_gear_idx = 0;

static bool  g_mpu_ok = false;
static float g_pitch  = 0.0f;
static float g_roll   = 0.0f;
static float g_yaw    = 0.0f;

static void fmt_angle_deg1(float deg, char *sign, int32_t *int_part, int32_t *dec_part)
{
    int32_t deg10 = (int32_t)(deg * 10.0f);
    *sign = (deg10 >= 0) ? '+' : '-';
    if (deg10 < 0) deg10 = -deg10;
    *int_part = deg10 / 10;
    *dec_part = deg10 % 10;
}

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();     /* 含 I2C_0 (PA0/PA1) 硬件初始化 */
    led_init();
    led_on();
    key_init();
    uart0_init();
    tb6612_init();
    encoder_init();
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMER_IIDX_ZERO:
        g_tick_10ms++;
        break;
    default:
        break;
    }
}

void GROUP1_IRQHandler(void)
{
    if (encoder_right_int_is_pending()) {
        encoder_right_irq_handler();
    }
}

int main(void)
{
    prvSetupHardware();

    /* ── MPU6050 初始化: 硬件I2C0 ── */
    uart0_sendStr("\r\n--- MPU6050 HW I2C0 ---\r\n");

    /* SysConfig 已配好 I2C_0, 直接读写 */
    {
        uint8_t whoami_buf[1] = {0x75};
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, whoami_buf, 1);
        DL_I2C_startControllerTransfer(I2C_0_INST, 0x68, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
        while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

        DL_I2C_startControllerTransfer(I2C_0_INST, 0x68, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {}
        uint8_t whoami = DL_I2C_receiveControllerData(I2C_0_INST);
        while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}

        char buf[32];
        snprintf(buf, sizeof(buf), "WHO_AM_I=0x%02X\r\n", whoami);
        uart0_sendStr(buf);

        if (whoami == 0x68) {
            uart0_sendStr("MPU6050 OK\r\n");
            g_mpu_ok = true;
            MPU6050_Init();
        } else {
            uart0_sendStr("MPU6050 FAIL\r\n");
        }
    }

    if (!speed_service_init()) { while (1) {} }

    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    uart0_sendStr("M0 Ready\r\n");

    OLED_Init();
    OLED_Clear();
    OLED_vsprint(0, 0, 16, g_mpu_ok ? "Speed+MPU OK" : "MPU FAIL!");
    OLED_Refresh();

    uint32_t last_tick_200 = 0, last_tick_mpu = 0;
    bool     key_was       = false;
    uint16_t clear_count   = 99;

    while (1) {
        uint32_t now = g_tick_10ms;
        speed_service_step_10ms();

        if (g_mpu_ok && now != last_tick_mpu) {
            last_tick_mpu = now;
            int16_t ax, ay, az, gx, gy, gz;
            MPU6050_GetRaw(&ax, &ay, &az, &gx, &gy, &gz);
            float ax_g = (float)ax / ACCEL_SCALE;
            float ay_g = (float)ay / ACCEL_SCALE;
            float az_g = (float)az / ACCEL_SCALE;
            g_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;
            g_roll  = atan2f( ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 57.29578f;
            g_yaw += ((float)gz / GYRO_SCALE) * 0.01f;
            while (g_yaw >  180.0f) g_yaw -= 360.0f;
            while (g_yaw < -180.0f) g_yaw += 360.0f;
        }

        bool key_now = key_read_user();
        if (key_now && !key_was) {
            g_speed_gear_idx++;
            if (g_speed_gear_idx >= GEAR_COUNT) g_speed_gear_idx = 0;
            speed_service_set_target(g_gear[g_speed_gear_idx], g_gear[g_speed_gear_idx]);
        }
        key_was = key_now;

        if (now - last_tick_200 >= 20) {
            last_tick_200 = now;
            speed_service_state_t ws;
            bool valid = speed_service_get_state(&ws);
            if (++clear_count >= 100) { OLED_Clear(); clear_count = 0; }

            if (!valid) {
                OLED_vsprint(0, 0, 16, "wait speed init");
                OLED_vsprint(0, 16, 16, "spd loop init..");
            } else {
                int32_t la = util_abs_i32(ws.left_rpm);
                int32_t ra = util_abs_i32(ws.right_rpm);
                char ls = (ws.left_rpm < 0) ? '-' : ' ';
                char rs = (ws.right_rpm < 0) ? '-' : ' ';
                OLED_vsprint(0, 0, 16, "L:%c%3ld R:%c%3ld", ls, (long)la, rs, (long)ra);
                OLED_vsprint(0, 16, 16, "T:%4ld G:%3ld %s",
                    (long)ws.left_target_rpm, (long)g_gear[g_speed_gear_idx], ws.stopped ? "STP" : "RUN");
            }

            if (g_mpu_ok) {
                char ps, rs, ys;
                int32_t pi, ri, yi, pd, rd, yd;
                fmt_angle_deg1(g_pitch, &ps, &pi, &pd);
                fmt_angle_deg1(g_roll,  &rs, &ri, &rd);
                fmt_angle_deg1(g_yaw,   &ys, &yi, &yd);
                OLED_vsprint(0, 32, 16, "P:%c%2ld.%1ld R:%c%2ld.%1ld", ps, (long)pi, (long)pd, rs, (long)ri, (long)rd);
                OLED_vsprint(0, 48, 16, "Y:%c%3ld.%1ld", ys, (long)yi, (long)yd);
            } else {
                OLED_vsprint(0, 32, 16, "MPU6050 FAIL");
                OLED_vsprint(0, 48, 16, "check PA0/PA1");
            }
            OLED_Refresh();
        }
    }
}
