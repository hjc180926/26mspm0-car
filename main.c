/**
 * @file    main.c
 * @brief   M0 - 固定 15% PWM 正反转 5s 切换 + 编码器 + MPU6050
 *
 * 硬件: MG513X, 13PPR×4×28=1456脉冲/圈, 轮径65mm
 *       PULSE_PER_CM ≈ 71.3
 */

#include "ti_msp_dl_config.h"
#include "led/led.h"
#include "UART/uart0.h"
#include "tb6612/tb6612.h"
#include "encoder/encoder.h"
#include "oled/oled.h"
#include "mpu6050/mpu6050.h"
#include "common/util.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

static volatile uint32_t g_tick_1ms = 0;

/* 软件任务节拍 */
#define TICK_10MS  10U
#define TICK_100MS 100U

/* ═══════════════════════════════════════════════════════════════════════
 *  物理换算
 * ═══════════════════════════════════════════════════════════════════════ */
#define PULSE_PER_CM_X10  713UL

/* delta/10ms → cm/s × 10 */
#define SPEED_CMS_X10(d)  ((int32_t)((int64_t)(d) * 10000 / PULSE_PER_CM_X10))

/* 累计脉冲(有符号) → cm × 10 */
#define DIST_CM_X10(c)    ((int64_t)(c) * 100 / PULSE_PER_CM_X10)

/* ═══════════════════════════════════════════════════════════════════════
 *  全局状态
 * ═══════════════════════════════════════════════════════════════════════ */
static int32_t  g_left_cms_x10  = 0;
static int32_t  g_right_cms_x10 = 0;
static int32_t  g_left_dist     = 0;   /* 有符号累计脉冲 */
static int32_t  g_right_dist    = 0;
static bool     g_mpu_ok        = false;
static float    g_pitch         = 0.0f;
static float    g_roll          = 0.0f;

/* ═══════════════════════════════════════════════════════════════════════ */

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init();
    led_on();
    uart0_init();
    tb6612_init();
    encoder_init();
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
    case DL_TIMER_IIDX_ZERO:
        g_tick_1ms++;
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

static void apply_pwm(int16_t lp, int16_t rp)
{
    if (lp == 0 && rp == 0) {
        tb6612_brake();
    } else {
        tb6612_set_speed(rp, lp);
    }
}

static void fmt_cms(char *buf, int32_t val_x10)
{
    int32_t v = util_abs_i32(val_x10);
    char s = (val_x10 < 0) ? '-' : ' ';
    snprintf(buf, 8, "%c%ld.%01ld", s, (long)(v / 10), (long)(v % 10));
}

/* 有符号距离 */
static void fmt_dist(char *buf, int32_t raw)
{
    int64_t d = DIST_CM_X10(raw);
    int64_t v = (d >= 0) ? d : -d;
    char s = (d < 0) ? '-' : ' ';
    snprintf(buf, 8, "%c%3lld.%01lld", s, (long long)(v / 10), (long long)(v % 10));
}

static void fmt_angle(char *buf, float deg)
{
    int32_t d10 = (int32_t)(deg * 10.0f);
    char sign = (d10 >= 0) ? '+' : '-';
    if (d10 < 0) d10 = -d10;
    snprintf(buf, 7, "%c%2ld.%01ld", sign, (long)(d10 / 10), (long)(d10 % 10));
}

static void oled_draw(int32_t l_cms_x10, int32_t r_cms_x10,
                      int32_t l_dist, int32_t r_dist,
                      uint32_t elapsed_ms,
                      bool mpu_ok, float pitch, float roll)
{
    char bl[8], br[8], dl[8], dr[8], ps[7], rs[7];
    fmt_cms(bl, l_cms_x10);
    fmt_cms(br, r_cms_x10);
    fmt_dist(dl, l_dist);
    fmt_dist(dr, r_dist);
    fmt_angle(ps, pitch);
    fmt_angle(rs, roll);

    OLED_Clear();
    OLED_vsprint(0, 0, 16, "L:%s R:%s cm/s", bl, br);
    OLED_vsprint(0, 16, 16, "DL:%s DR:%s cm", dl, dr);
    OLED_vsprint(0, 32, 16, "T:%lu.%lus",
        (unsigned long)(elapsed_ms / 1000),
        (unsigned long)((elapsed_ms % 1000) / 100));
    if (mpu_ok) {
        OLED_vsprint(0, 48, 16, "P:%s R:%s", ps, rs);
    } else {
        OLED_vsprint(0, 48, 16, "MPU: no signal");
    }
    OLED_Refresh();
}

int main(void)
{
    prvSetupHardware();

    DL_TimerG_stopCounter(TIMER_0_INST);
    DL_TimerG_setLoadValue(TIMER_0_INST, 79999U);

    uart0_sendStr("\r\n--- FWD/REV 15%% 5s ---\r\n");
    {
        uint8_t wbuf[1] = {0x75};
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, wbuf, 1);
        DL_I2C_startControllerTransfer(I2C_0_INST, 0x68, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
        while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}
        DL_I2C_startControllerTransfer(I2C_0_INST, 0x68, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {}
        uint8_t whoami = DL_I2C_receiveControllerData(I2C_0_INST);
        while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {}
        if (whoami == 0x68) {
            g_mpu_ok = true;
            MPU6050_Init();
            uart0_sendStr("MPU6050 OK\r\n");
        } else {
            uart0_sendStr("MPU6050 FAIL\r\n");
        }
    }

    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    uart0_sendStr("Test Ready\r\n");
    OLED_Init();

    /* ── 正反转状态：0=正转 1=反转，每 5000ms 切换 ── */
    bool     fwd           = true;
    uint32_t dir_deadline  = 5000;
    bool     dir_changed   = true;

    while (1) {
        uint32_t now = g_tick_1ms;

        /* 方向切换 */
        if (now >= dir_deadline) {
            fwd = !fwd;
            dir_deadline = now + 5000;
            dir_changed  = true;
        }

        /* ── 10ms：编码器 + MPU6050 + PWM 输出 ── */
        if ((now % TICK_10MS) == 0) {
            if (dir_changed) {
                dir_changed = false;
                int16_t pwm = fwd ? 15 : -15;
                apply_pwm(pwm, pwm);
            }

            encoder_data_t enc;
            encoder_get_data(&enc);
            g_left_cms_x10  = SPEED_CMS_X10(enc.left_delta);
            g_right_cms_x10 = SPEED_CMS_X10(enc.right_delta);
            g_left_dist  += enc.left_delta;
            g_right_dist += enc.right_delta;

            if (g_mpu_ok) {
                int16_t ax, ay, az;
                MPU6050_GetAccel(&ax, &ay, &az);
                float ax_g = (float)ax / ACCEL_SCALE;
                float ay_g = (float)ay / ACCEL_SCALE;
                float az_g = (float)az / ACCEL_SCALE;
                g_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.29578f;
                g_roll  = atan2f( ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 57.29578f;
            }
        }

        /* ── 100ms：OLED ── */
        if ((now % TICK_100MS) == 0) {
            oled_draw(g_left_cms_x10, g_right_cms_x10,
                      g_left_dist, g_right_dist,
                      now,
                      g_mpu_ok, g_pitch, g_roll);
        }
    }
}
