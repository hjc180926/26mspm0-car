/**
 * @file    main.c
 * @brief   8路循迹 + 三种循迹模式 + 按键计时 + 编码器里程自动停车
 */

#include "ti_msp_dl_config.h"
#include "led/led.h"
#include "tb6612/tb6612.h"
#include "encoder/encoder.h"
#include "oled/oled.h"
#include "tracking/tracking.h"
#include "buzzer/buzzer.h"
#include <stdint.h>
#include <stdbool.h>

volatile uint32_t g_tick_1ms = 0;

/* -- 长距离循迹参数 -- */
#define TRK_STRAIGHT   35
#define TRK_SMALL      27
#define TRK_MID        22
#define TRK_BIG         15

/* -- 缓起缓停参数 (模式2/3共用) -- */
#define RAMP_MAX_SPEED        30
#define RAMP_DOWN_CM     80     /* 最后80cm减速 */
#define RAMP_MIN_SPEED        18

/* -- 模式2 短距离 -- */
#define MODE2_TARGET_CM_X10   1750   /* 175.0cm */

/* -- 模式3 减速循迹 -- */
#define MODE3_OFFSET          10
#define MODE3_OFFSET           5
/* -- 模式1 里程 -- */
#define TARGET_CM_X10  6253  /* 614.16cm */

#define BTN_U11 (DL_GPIO_readPins(GPIO_BTN_U11_PORT, GPIO_BTN_U11_PIN_BTN_U11_PIN)==0)
#define BTN_U10 (DL_GPIO_readPins(GPIO_BTN_U10_PORT, GPIO_BTN_U10_PIN_BTN_U10_PIN)==0)

static uint32_t g_tmr_start   = 0;
static bool     g_tmr_on      = false;
static bool     g_run         = false;
static uint32_t g_tmr_frozen  = 0;
static int32_t  g_enc_start_l = 0;
static int32_t  g_enc_start_r = 0;
static uint32_t g_dist_cm_x10 = 0;

static uint8_t  g_mode        = 1;
static int16_t  g_ramp_spd    = 0;

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init(); led1_on(); led2_on();
    buzzer_init();
    tracking_init();
    tb6612_init();
    encoder_init();
}

void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMER_0_INST)==DL_TIMER_IIDX_ZERO) {
        DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMER_IIDX_ZERO);
        g_tick_1ms++;
    }
}

void GROUP1_IRQHandler(void)
{
    if(encoder_right_int_is_pending()) encoder_right_irq_handler();
}

int main(void)
{
    prvSetupHardware();
    DL_TimerG_stopCounter(TIMER_0_INST);
    DL_TimerG_setLoadValue(TIMER_0_INST, 79999U);

    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    OLED_Init();
    uint32_t le = g_tick_1ms, lo = g_tick_1ms;
    uint32_t last_btn = 0;
    bool     pb9_was  = false;
    bool     pb20_was = false;

    while(1) {
        uint32_t n = g_tick_1ms;

        /* 10ms: 按键消抖 */
        if((int32_t)(n - last_btn) >= 10) { last_btn += 10;
            bool pb9  = BTN_U11;
            bool pb20 = BTN_U10;

            if(pb9 && !pb9_was) {
                g_run = !g_run;
                if(g_run) {
                    encoder_data_t enc; encoder_get_data(&enc);
                    g_enc_start_l = enc.left_count;
                    g_enc_start_r = enc.right_count;
                    g_dist_cm_x10 = 0;
                    g_tmr_on = true; g_tmr_start = n;
                    g_ramp_spd = 0;
                } else {
                    g_tmr_frozen = n - g_tmr_start; g_tmr_on = false;
                    tb6612_set_speed(0, 0);
                }
            }
            pb9_was = pb9;

            if(pb20 && !pb20_was && !g_run) {
                g_mode = (g_mode >= 3) ? 1 : (uint8_t)(g_mode + 1);
            }
            pb20_was = pb20;
        }

        /* 10ms: 里程 + 循迹控制 */
        while((int32_t)(n - le) >= 10) { le += 10;
            static int16_t lx = TRK_STRAIGHT, rx = TRK_STRAIGHT;

            if(g_run) {
                /* 编码器里程 */
                encoder_data_t enc; encoder_get_data(&enc);
                int32_t a = enc.left_count  - g_enc_start_l;
                int32_t b = enc.right_count - g_enc_start_r;
                if(a < 0) a = -a; if(b < 0) b = -b;
                uint32_t avg = ((uint32_t)a + (uint32_t)b) / 2U;
                g_dist_cm_x10 = (uint32_t)((avg * 10U) / ENCODER_PULSES_PER_CM);

                /* 里程停车 */
                uint32_t target = (g_mode == 2) ? MODE2_TARGET_CM_X10 : TARGET_CM_X10;
                if(g_dist_cm_x10 >= target) {
                    g_run = false; g_tmr_on = false;
                    g_tmr_frozen = n - g_tmr_start;
                    tb6612_set_speed(0, 0);
                }
            }

            if(g_run) {
                /* 缓起 (模式2/3) + 缓停 (仅模式2) */
                if(g_mode == 2 || g_mode == 3) {
                    static uint8_t rc = 0;
                    if(++rc >= 7) { rc = 0;
                        if(g_ramp_spd < RAMP_MAX_SPEED) g_ramp_spd++;
                    }
                    if(g_mode == 2) {
                        uint32_t rem = MODE2_TARGET_CM_X10 - g_dist_cm_x10;
                        if(rem < (uint32_t)(RAMP_DOWN_CM * 10)) {
                            int16_t limit = (int16_t)((rem * RAMP_MAX_SPEED)
                                            / (RAMP_DOWN_CM * 10));
                            if(limit < RAMP_MIN_SPEED) limit = RAMP_MIN_SPEED;
                            if(g_ramp_spd > limit) g_ramp_spd = limit;
                        }
                    }
                }

                /* 8路循迹 (三模式共用) */
                uint8_t m  = tracking_read_all();
                uint8_t c0 = (m >> 0) & 1, c1 = (m >> 1) & 1;
                uint8_t c2 = (m >> 2) & 1, c3 = (m >> 3) & 1;
                uint8_t c4 = (m >> 4) & 1, c5 = (m >> 5) & 1;
                uint8_t c6 = (m >> 6) & 1, c7 = (m >> 7) & 1;
                uint8_t any = c0 | c1 | c2 | c3 | c4 | c5 | c6 | c7;

                if(!any) {
                    tb6612_set_speed(lx, rx);
                } else if(c0) {
                    lx = TRK_STRAIGHT; rx = TRK_BIG;
                    tb6612_set_speed(lx, rx);
                } else if(c7) {
                    lx = TRK_BIG; rx = TRK_STRAIGHT;
                    tb6612_set_speed(lx, rx);
                } else if(c1) {
                    lx = TRK_STRAIGHT; rx = TRK_MID;
                    tb6612_set_speed(lx, rx);
                } else if(c6) {
                    lx = TRK_MID; rx = TRK_STRAIGHT;
                    tb6612_set_speed(lx, rx);
                } else if(c2) {
                    lx = TRK_STRAIGHT; rx = TRK_SMALL;
                    tb6612_set_speed(lx, rx);
                } else if(c5) {
                    lx = TRK_SMALL; rx = TRK_STRAIGHT;
                    tb6612_set_speed(lx, rx);
                } else {
                    lx = TRK_STRAIGHT; rx = TRK_STRAIGHT;
                    tb6612_set_speed(lx, rx);
                }

                /* 缓起缓停: 缩放速度 (模式2/3) */
                if(g_mode == 2 || g_mode == 3) {
                    lx = (int16_t)((int32_t)lx * g_ramp_spd / TRK_STRAIGHT);
                    rx = (int16_t)((int32_t)rx * g_ramp_spd / TRK_STRAIGHT);
                    if(g_mode == 3) {
                        lx -= MODE3_OFFSET; if(lx < 8) lx = 8;
                        rx -= MODE3_OFFSET; if(rx < 8) rx = 8;
                    }
                    tb6612_set_speed(lx, rx);
                }
            }
        }

        /* 100ms: OLED */
        if((int32_t)(n - lo) >= 100) { lo += 100;
            uint8_t  tk = tracking_read_all();
            uint32_t t  = g_tmr_on ? (n - g_tmr_start) : g_tmr_frozen;
            uint32_t sec = t / 1000, ts = (t % 1000) / 100;
            uint32_t cm  = g_dist_cm_x10 / 10, mm = g_dist_cm_x10 % 10;
            int i;

            OLED_Clear();
            OLED_ShowChar(0, 0, 'T', 16, 1);
            OLED_ShowChar(8, 0, ':', 16, 1);
            for(i = 0; i < 8; i++) {
                OLED_ShowChar((uint8_t)((2 + i) * 8), 0,
                    ((tk >> i) & 1) ? '1' : '0', 16, 1);
            }
            OLED_ShowString(0, 16, "Time:", 16, 1);
            OLED_ShowNum(5*8, 16, sec, 3, 16, 1);
            OLED_ShowChar(8*8, 16, '.', 16, 1);
            OLED_ShowNum(9*8, 16, ts, 1, 16, 1);
            OLED_ShowChar(10*8, 16, 's', 16, 1);
            OLED_ShowString(12*8, 16, g_tmr_on ? " ON" : "OFF", 16, 1);

            OLED_ShowString(0, 32, "Dist:", 16, 1);
            OLED_ShowNum(5*8, 32, cm, 4, 16, 1);
            OLED_ShowChar(9*8, 32, '.', 16, 1);
            OLED_ShowNum(10*8, 32, mm, 1, 16, 1);
            OLED_ShowString(11*8, 32, "cm", 16, 1);

            OLED_ShowString(0, 48, "Mode:", 16, 1);
            OLED_ShowString(5*8, 48,
                (g_mode == 1) ? "TRK" : (g_mode == 2) ? "1.5" : "3", 16, 1);
            if((g_mode == 2 || g_mode == 3) && g_run) {
                OLED_ShowNum(9*8, 48, g_ramp_spd, 2, 16, 1);
            }
            OLED_Refresh();
        }
    }
}
