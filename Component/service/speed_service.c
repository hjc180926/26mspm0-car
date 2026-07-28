/**
 * @file    speed_service.c
 * @brief   速度环适配层实现：裸机版（PID已旁路，直接开环输出）
 */
#include "service/speed_service.h"
#include "control/speed_loop_core.h"
#include "encoder/encoder.h"
#include "tb6612/tb6612.h"
#include <stddef.h>

typedef struct {
    int32_t left_rpm;
    int32_t right_rpm;
    bool low_speed_ff_enable;
} speed_target_msg_t;

static speed_target_msg_t    g_speed_target;
static volatile bool         g_target_pending;
static speed_service_state_t g_speed_state;
static bool                  g_low_speed_ff_enable;

/* PID 旁路开关：改为 true 则恢复 PID 闭环 */
static bool g_pid_bypass = true;

bool speed_service_init(void)
{
    g_target_pending      = false;
    g_low_speed_ff_enable = false;
    g_speed_state = (speed_service_state_t){0};
    return true;
}

bool speed_service_set_target_with_ff(int32_t left_rpm, int32_t right_rpm, bool low_speed_ff_enable)
{
    g_speed_target.left_rpm            = left_rpm;
    g_speed_target.right_rpm           = right_rpm;
    g_speed_target.low_speed_ff_enable = low_speed_ff_enable;
    g_target_pending                   = true;
    return true;
}

bool speed_service_set_target(int32_t left_rpm, int32_t right_rpm)
{
    return speed_service_set_target_with_ff(left_rpm, right_rpm, false);
}

void speed_service_step_10ms(void)
{
    if (g_target_pending) {
        g_target_pending = false;
        g_low_speed_ff_enable = g_speed_target.low_speed_ff_enable;

        if (g_pid_bypass) {
            /* ── PID 旁路：目标 RPM 直接映射 PWM 开环输出 ──
             * 映射: pwm = target_rpm / 4  (400 RPM → 100% PWM)
             * 目标为 0/0 时制动 */
            if (g_speed_target.left_rpm == 0 && g_speed_target.right_rpm == 0) {
                tb6612_brake();
            } else {
                int32_t lp = g_speed_target.left_rpm / 4;
                int32_t rp = g_speed_target.right_rpm / 4;
                if (lp >  100) lp =  100;
                if (lp < -100) lp = -100;
                if (rp >  100) rp =  100;
                if (rp < -100) rp = -100;
                tb6612_set_speed((int16_t)rp, (int16_t)lp);
            }

            /* 快照：直接用目标值填充，实测 RPM 填 0 */
            g_speed_state.left_rpm         = 0;
            g_speed_state.right_rpm        = 0;
            g_speed_state.left_target_rpm  = g_speed_target.left_rpm;
            g_speed_state.right_target_rpm = g_speed_target.right_rpm;
            g_speed_state.stopped = (g_speed_target.left_rpm == 0 &&
                                     g_speed_target.right_rpm == 0);
        } else {
            /* ── 原始 PID 闭环路径 ── */
            speed_loop_core_t g_speed_core = {0};
            speed_loop_core_init(&g_speed_core);
            if (speed_loop_core_set_target(&g_speed_core,
                                           g_speed_target.left_rpm,
                                           g_speed_target.right_rpm)) {
                tb6612_brake();
            }
        }
    }

    if (!g_pid_bypass) {
        return;  /* PID 路径在上面的 target_pending 中已处理 */
    }
}

bool speed_service_get_state(speed_service_state_t *out)
{
    if (out == NULL) return false;
    *out = g_speed_state;
    return true;
}

bool speed_service_wheels_stopped_snapshot(void)
{
    return g_speed_state.stopped;
}
