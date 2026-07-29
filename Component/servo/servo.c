/**
 * @file    servo.c
 * @brief   舵机 PWM 控制（MSPM0G3507 TIMA0_C0 / PA21）
 *
 * SysConfig 配置：TIMA0, CCP0/PA21, 1MHz 时钟, 周期 20000 = 20ms = 50Hz
 * 边沿对齐 PWM 为反向比较：高电平 = PERIOD - CC_VALUE
 * 1500us 脉宽 → CC_VALUE = 20000 - 1500 = 18500
 */

#include "servo.h"
#include "ti_msp_dl_config.h"

#define SERVO_PWM_PERIOD_COUNT  20000U

static uint16_t g_servo_angle_deg = SERVO_ANGLE_CENTER_DEG;
static uint16_t g_servo_pulse_us  = SERVO_PULSE_CENTER_US;

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static uint16_t pulse_to_angle(uint16_t pulse_us)
{
    uint32_t num = (uint32_t)(pulse_us - SERVO_PULSE_MIN_US) *
                   (SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG);
    uint32_t den = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
    return (uint16_t)(SERVO_ANGLE_MIN_DEG + (num + den/2U) / den);
}

void servo_init(void)
{
    servo_set_angle(SERVO_ANGLE_CENTER_DEG);
    DL_TimerA_startCounter(PWM_SERVO_INST);
}

void servo_set_pulse_us(uint16_t pulse_us)
{
    pulse_us = clamp_u16(pulse_us, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    uint32_t cmp = SERVO_PWM_PERIOD_COUNT - (uint32_t)pulse_us;
    DL_TimerA_setCaptureCompareValue(PWM_SERVO_INST, cmp, GPIO_PWM_SERVO_C0_IDX);
    g_servo_pulse_us  = pulse_us;
    g_servo_angle_deg = pulse_to_angle(pulse_us);
}

void servo_set_angle(uint16_t angle_deg)
{
    angle_deg = clamp_u16(angle_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    uint32_t rng_p = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
    uint32_t rng_a = SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG;
    uint32_t pulse = SERVO_PULSE_MIN_US +
        ((uint32_t)(angle_deg - SERVO_ANGLE_MIN_DEG) * rng_p + rng_a/2U) / rng_a;
    servo_set_pulse_us((uint16_t)pulse);
    g_servo_angle_deg = angle_deg;
}

uint16_t servo_get_angle(void)  { return g_servo_angle_deg; }
uint16_t servo_get_pulse_us(void) { return g_servo_pulse_us; }
