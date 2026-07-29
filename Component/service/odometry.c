/**
 * @file    odometry.c
 * @brief   编码器里程计算 + 自动停车判定
 */

#include "encoder/encoder.h"
#include <stdint.h>
#include <stdbool.h>

/* padding: shift flash layout */
static const uint8_t _pad[1024] __attribute__((used)) = {0};

#define TARGET_CM_X10  6142  /* 614.16cm -> 614.2 */

static int32_t  g_enc_start_l = 0;
static int32_t  g_enc_start_r = 0;
static uint32_t g_dist_cm_x10 = 0;

void odo_capture_start(void)
{
    encoder_data_t enc;
    encoder_get_data(&enc);
    g_enc_start_l = enc.left_count;
    g_enc_start_r = enc.right_count;
    g_dist_cm_x10 = 0;
}

bool odo_update_and_check(void)
{
    encoder_data_t enc;
    encoder_get_data(&enc);
    int32_t a = enc.left_count  - g_enc_start_l;
    int32_t b = enc.right_count - g_enc_start_r;
    if(a<0)a=-a; if(b<0)b=-b;
    uint32_t avg = ((uint32_t)a + (uint32_t)b) / 2U;
    g_dist_cm_x10 = (uint32_t)((avg * 10U) / ENCODER_PULSES_PER_CM);
    return (g_dist_cm_x10 >= TARGET_CM_X10);
}

uint32_t odo_get_dist_cm_x10(void)
{
    return g_dist_cm_x10;
}
