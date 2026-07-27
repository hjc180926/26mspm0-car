/**
 * @file    main.c
 * @brief   M0_Templant_FreeRTOS - speed-only 速度环独立测试工程（裸机版）
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
#include <stdio.h>
#include <stdint.h>

#define SPEED_DEBUG_PRINT_ENABLE  1

static volatile uint32_t g_tick_10ms = 0;

static const int32_t g_gear[] = {0, 30, 200, 400, -20, -150, -400};
#define GEAR_COUNT (sizeof(g_gear) / sizeof(g_gear[0]))
static uint32_t g_speed_gear_idx = 0;

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init();
    led_on();            /* PB22 上电常亮 */
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

    if (!speed_service_init()) {
        while (1) {}
    }

    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    uart0_sendStr("M0 Speed-Only Closed Loop Ready | speed key | 80MHz\r\n");

    OLED_Init();
    OLED_Clear();
    OLED_vsprint(0, 0, 16, "Speed-Only Init");
    OLED_Refresh();

    uint32_t last_tick_50  = 0;
    uint32_t last_tick_200 = 0;
    bool     key_was       = false;
    uint16_t clear_count   = 99;

    while (1) {
        uint32_t now = g_tick_10ms;

        speed_service_step_10ms();

        bool key_now = key_read_user();
        if (key_now && !key_was) {
            g_speed_gear_idx++;
            if (g_speed_gear_idx >= GEAR_COUNT) {
                g_speed_gear_idx = 0;
            }
            (void)speed_service_set_target(g_gear[g_speed_gear_idx],
                                           g_gear[g_speed_gear_idx]);
        }
        key_was = key_now;

#if SPEED_DEBUG_PRINT_ENABLE
        if (now - last_tick_50 >= 5) {
            last_tick_50 = now;
            speed_service_state_t dbg;
            if (speed_service_get_state(&dbg)) {
                char buf[64];
                snprintf(buf, sizeof(buf), "%ld,%ld\r\n",
                         (long)dbg.left_rpm, (long)dbg.right_rpm);
                uart0_sendStr(buf);
            }
        }
#endif

        if (now - last_tick_200 >= 20) {
            last_tick_200 = now;
            speed_service_state_t ws;
            bool valid = speed_service_get_state(&ws);
            clear_count++;
            if (clear_count >= 100) {
                OLED_Clear();
                clear_count = 0;
            }
            if (!valid) {
                OLED_vsprint(0, 0, 16, "wait speed init");
                OLED_vsprint(0, 16, 16, "spd loop init..");
            } else {
                int32_t la = util_abs_i32(ws.left_rpm);
                int32_t ra = util_abs_i32(ws.right_rpm);
                char ls = (ws.left_rpm  < 0) ? '-' : ' ';
                char rs = (ws.right_rpm < 0) ? '-' : ' ';
                OLED_vsprint(0, 0, 16, "L:%c%3ld R:%c%3ld",
                             ls, (long)la, rs, (long)ra);
                OLED_vsprint(0, 16, 16, "LT:%4ld RT:%4ld",
                             (long)ws.left_target_rpm, (long)ws.right_target_rpm);
                OLED_vsprint(0, 32, 16, "Gear:%ldRPM [%ld]",
                             (long)g_gear[g_speed_gear_idx], (long)g_speed_gear_idx);
                OLED_vsprint(0, 48, 16, "%s",
                             ws.stopped ? "STOPPED" : "RUNNING ");
            }
            OLED_Refresh();
        }
    }
}
