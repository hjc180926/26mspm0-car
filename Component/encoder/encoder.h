/**
 * @file    encoder.h
 * @brief   双路增量式编码器接口定义
 *
 * 编码器规格:
 *   - 电机: MG513X
 *   - 编码器: 13 PPR (4 倍频 = 52 计数/电机圈)
 *   - 减速比: 28:1 -> 输出轴每圈 1456 计数
 *   - 轮径: 65mm -> 轮周长 20.42cm -> ~71 脉冲/cm
 *
 * 硬件映射 (实测):
 *   - 物理右轮: TIMG8 硬件 QEI (PA26=A, PA27=B)
 *   - 物理左轮: GPIO 双边沿软件解码 (PA25=A, PA14=B)
 *   - encoder_get_data() 内部做了左右交换，调用者无需关心硬件差异
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/** @brief 编码器每圈计数 (13PPR x 4 倍频 = 52, 电机轴) */
#define ENCODER_COUNTS_PER_REV      (52)

/** @brief 左轮编码器每圈计数 (电机轴) */
#define ENCODER_LEFT_COUNTS_PER_REV  ENCODER_COUNTS_PER_REV

/** @brief 右轮编码器每圈计数 (电机轴) */
#define ENCODER_RIGHT_COUNTS_PER_REV ENCODER_COUNTS_PER_REV

/** @brief 输出轴每圈计数 (52 x 28 = 1456) */
#define ENCODER_COUNTS_PER_OUTPUT_REV  (ENCODER_COUNTS_PER_REV * 28)

/** @brief 每厘米脉冲数 (实测校准) */
#define ENCODER_PULSES_PER_CM     (67)

/**
 * @brief 编码器数据快照结构体
 */
typedef struct {
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta;
    int32_t right_delta;
} encoder_data_t;

void encoder_init(void);
void encoder_reset(void);
void encoder_get_data(encoder_data_t *data);
int32_t encoder_delta_to_rpm10(int32_t delta, uint32_t period_ms);
int encoder_right_int_is_pending(void);
void encoder_right_irq_handler(void);

#endif /* ENCODER_H */
