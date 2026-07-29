/**
 * @file    servo.h
 * @brief   舵机 PWM 控制接口（TIMA0_C0 / PA21）
 *
 * PWM 参数：50 Hz，周期 20 ms；脉宽 500~2500 us 对应 0~180°。
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_ANGLE_MIN_DEG        0U
#define SERVO_ANGLE_MAX_DEG      180U
#define SERVO_ANGLE_CENTER_DEG    90U

#define SERVO_PULSE_MIN_US       500U
#define SERVO_PULSE_CENTER_US   1500U
#define SERVO_PULSE_MAX_US      2500U

void servo_init(void);
void servo_set_angle(uint16_t angle_deg);
void servo_set_pulse_us(uint16_t pulse_us);
uint16_t servo_get_angle(void);
uint16_t servo_get_pulse_us(void);

#endif
