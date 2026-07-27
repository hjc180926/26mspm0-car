/**
 * @file    speed_service.h
 * @brief   速度环适配层接口（裸机版）
 */
#ifndef SPEED_SERVICE_H
#define SPEED_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#define SPEED_SERVICE_PERIOD_MS 50U

typedef struct {
    int32_t left_rpm;
    int32_t right_rpm;
    int32_t left_target_rpm;
    int32_t right_target_rpm;
    bool stopped;
} speed_service_state_t;

bool speed_service_init(void);
bool speed_service_set_target(int32_t left_rpm, int32_t right_rpm);
bool speed_service_set_target_with_ff(int32_t left_rpm, int32_t right_rpm, bool low_speed_ff_enable);
void speed_service_step_10ms(void);
bool speed_service_get_state(speed_service_state_t *out);
bool speed_service_wheels_stopped_snapshot(void);

#endif /* SPEED_SERVICE_H */
