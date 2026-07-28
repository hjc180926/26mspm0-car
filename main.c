/**
 * @file    main.c
 * @brief   MPU6050 yaw — 用已验证的驱动函数，不加自定义 I2C
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

#define PULSE_PER_CM_X10  713UL
#define SPEED_CMS_X10(d)  ((int32_t)((int64_t)(d) * 10000 / PULSE_PER_CM_X10))
#define DIST_CM_X10(c)    ((int64_t)(c) * 100 / PULSE_PER_CM_X10)

#define GYRO_DEADBAND_DPS   0.25f
#define YAW_DT_S            0.01f
#define BIAS_CALIB_SAMPLES  300

static int32_t  g_l_cms_x10 = 0, g_r_cms_x10 = 0;
static int32_t  g_l_dist = 0, g_r_dist = 0;
static bool     g_mpu_ok = false;
static float    g_pitch = 0.0f, g_roll = 0.0f;

static float    g_yaw           = 0.0f;
static float    g_bias          = 0.0f;
static float    g_prev_gz       = 0.0f;
static bool     g_first         = true;
static uint32_t g_next_yaw      = 0;
static int      g_still_cnt     = 0;

/* ═══════════════════════════════════════════════════════════════════════ */

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init(); led_on();
    uart0_init();
    tb6612_init();
    encoder_init();
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
        g_tick_1ms++;
}

void GROUP1_IRQHandler(void)
{
    if (encoder_right_int_is_pending())
        encoder_right_irq_handler();
}

static void fmt_cms(char *b, int32_t v) {
    int32_t a = util_abs_i32(v);
    snprintf(b, 8, "%c%ld.%01ld", (v<0)?'-':' ', (long)(a/10), (long)(a%10));
}
static void fmt_dist(char *b, int32_t r) {
    int64_t d = DIST_CM_X10(r);
    int64_t a = (d>=0)?d:-d;
    snprintf(b, 8, "%c%3lld.%01lld", (d<0)?'-':' ', (long long)(a/10), (long long)(a%10));
}
static void fmt_ang(char *b, float d) {
    int32_t v = (int32_t)(d*10.0f);
    char s = (v>=0)?'+':'-'; if(v<0)v=-v;
    snprintf(b, 7, "%c%2ld.%01ld", s, (long)(v/10), (long)(v%10));
}

static float calib(void)
{
    float s = 0; int16_t x,y,z;
    uart0_sendStr("Calib...\r\n");
    for (int i=0; i<BIAS_CALIB_SAMPLES; i++) {
        MPU6050_GetGyro(&x,&y,&z);
        s += (float)z / GYRO_SCALE;
        volatile uint32_t d=8000; while(d--){}
    }
    float b = s / BIAS_CALIB_SAMPLES;
    char buf[40]; snprintf(buf,sizeof(buf),"bias=%.4f\r\n",(double)b);
    uart0_sendStr(buf);
    return b;
}

static void yaw_step(void)
{
    int16_t gx,gy,gz;
    MPU6050_GetGyro(&gx,&gy,&gz);

    float gz_raw = (float)gz / GYRO_SCALE;
    float gz_dps = gz_raw - g_bias;
    if (fabsf(gz_dps) < GYRO_DEADBAND_DPS) gz_dps = 0.0f;

    if (g_first) { g_prev_gz = gz_dps; g_first = false; return; }

    g_yaw += (g_prev_gz + gz_dps) * 0.5f * YAW_DT_S;
    g_prev_gz = gz_dps;

    while (g_yaw >  180.0f) g_yaw -= 360.0f;
    while (g_yaw < -180.0f) g_yaw += 360.0f;

    float g_mag = fabsf((float)gx/GYRO_SCALE) + fabsf((float)gy/GYRO_SCALE) + fabsf(gz_raw);

    int16_t ax,ay,az;
    MPU6050_GetAccel(&ax,&ay,&az);
    float ax_g=(float)ax/ACCEL_SCALE, ay_g=(float)ay/ACCEL_SCALE, az_g=(float)az/ACCEL_SCALE;
    float a_mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

    if (g_mag < 2.0f && fabsf(a_mag - 1.0f) < 0.08f) {
        if (++g_still_cnt > 50) {
            g_still_cnt = 50;
            g_bias += (gz_raw - g_bias) * 0.0005f;
        }
    } else {
        g_still_cnt = 0;
    }

    g_pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 57.29578f;
    g_roll  = atan2f( ay_g, sqrtf(ax_g*ax_g + az_g*az_g)) * 57.29578f;
}

/* ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    prvSetupHardware();
    DL_TimerG_stopCounter(TIMER_0_INST);
    DL_TimerG_setLoadValue(TIMER_0_INST, 79999U);

    uart0_sendStr("\r\n--- Yaw v5 ---\r\n");

    { uint8_t w[1]={0x75}; DL_I2C_fillControllerTXFIFO(I2C_0_INST,w,1);
      DL_I2C_startControllerTransfer(I2C_0_INST,0x68,DL_I2C_CONTROLLER_DIRECTION_TX,1);
      while(!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_IDLE)){}
      DL_I2C_startControllerTransfer(I2C_0_INST,0x68,DL_I2C_CONTROLLER_DIRECTION_RX,1);
      while(DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)){}
      if(DL_I2C_receiveControllerData(I2C_0_INST)==0x68){g_mpu_ok=true;MPU6050_Init();uart0_sendStr("OK\r\n");}
      else uart0_sendStr("FAIL\r\n");
      while(!(DL_I2C_getControllerStatus(I2C_0_INST)&DL_I2C_CONTROLLER_STATUS_IDLE)){} }

    NVIC_SetPriority(TIMER_0_INST_INT_IRQN,3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    if(g_mpu_ok){ g_bias=calib(); g_next_yaw=g_tick_1ms+10; }

    uart0_sendStr("Ready\r\n"); OLED_Init();

    uint32_t le=g_tick_1ms, lo=g_tick_1ms, lp=0;
    while(1){
        uint32_t n=g_tick_1ms;

        if(g_mpu_ok) while((int32_t)(n-g_next_yaw)>=0){ g_next_yaw+=10; yaw_step(); }

        while((int32_t)(n-le)>=10){ le+=10;
            encoder_data_t e; encoder_get_data(&e);
            g_l_cms_x10=SPEED_CMS_X10(e.left_delta); g_r_cms_x10=SPEED_CMS_X10(e.right_delta);
            g_l_dist+=e.left_delta; g_r_dist+=e.right_delta; }

        if((int32_t)(n-lo)>=100){ lo+=100;
            char bl[8],br[8],dl[8],dr[8],ps[7],rs[7];
            fmt_cms(bl,g_l_cms_x10); fmt_cms(br,g_r_cms_x10);
            fmt_dist(dl,g_l_dist); fmt_dist(dr,g_r_dist);
            fmt_ang(ps,g_pitch); fmt_ang(rs,g_roll);
            OLED_Clear();
            OLED_vsprint(0,0,16,"L:%s R:%s cm/s",bl,br);
            OLED_vsprint(0,16,16,"DL:%s DR:%s cm",dl,dr);
            OLED_vsprint(0,32,16,"T:%lu.%lus",
                (unsigned long)(n/1000),
                (unsigned long)((n%1000)/100));
            if(g_mpu_ok) OLED_vsprint(0,48,16,"P:%s R:%s",ps,rs);
            else OLED_vsprint(0,48,16,"MPU: no signal");
            OLED_Refresh(); }

        if((int32_t)(n-lp)>=5000){ lp+=5000;
            char b[50]; snprintf(b,sizeof(b),"B=%.4f Y=%.1f S=%d\r\n",
                (double)g_bias,(double)g_yaw,g_still_cnt);
            uart0_sendStr(b); }
    }
}
