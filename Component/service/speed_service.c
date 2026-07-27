/**
 * @file    speed_service.c
 * @brief   速度环适配层实现：裸机版
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
static speed_loop_core_t     g_speed_core;
static bool                  g_low_speed_ff_enable;

static void speed_service_publish_snapshot(void)
{
    speed_loop_core_output_t output = {0};
    speed_loop_core_get_output(&g_speed_core, &output);
    g_speed_state.left_rpm         = output.left_rpm;
    g_speed_state.right_rpm        = output.right_rpm;
    g_speed_state.left_target_rpm  = output.left_target_rpm;
    g_speed_state.right_target_rpm = output.right_target_rpm;
    g_speed_state.stopped          = output.stopped;
}

bool speed_service_init(void)
{
    g_target_pending      = false;
    g_low_speed_ff_enable = false;
    speed_loop_core_init(&g_speed_core);
    encoder_reset();
    speed_service_publish_snapshot();
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
        if (speed_loop_core_set_target(&g_speed_core,
                                       g_speed_target.left_rpm,
                                       g_speed_target.right_rpm)) {
            tb6612_brake();
            speed_service_publish_snapshot();
        }
    }

    encoder_data_t encoder;
    encoder_get_data(&encoder);

    speed_loop_core_output_t output = {0};
    if (speed_loop_core_update(&g_speed_core, encoder.left_delta, encoder.right_delta,
                               SPEED_LOOP_CORE_SAMPLE_PERIOD_MS,
                               ENCODER_COUNTS_PER_REV,
                               g_low_speed_ff_enable, &output)) {
        if (output.brake) {
            tb6612_brake();
        } else {
            tb6612_set_speed((int16_t)output.right_pwm, (int16_t)output.left_pwm);
        }
    }

    speed_service_publish_snapshot();
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
