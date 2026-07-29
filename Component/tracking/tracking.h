/**
 * @file    tracking.h
 * @brief   5路循迹传感器模块接口
 *
 * 传感器: 红外对管, 黑线输出低电平, 白底输出高电平
 *         内部上拉, 黑线=按下(true), 白底=释放(false)
 *
 * 通道布局:
 *   CH0     CH1     CH2     CH3     CH4
 *   PB25   PB18   PB21    PB22    PA30
 *
 * 使用示例:
 * @code
 *   tracking_init();
 *   uint8_t mask = tracking_read_all();  // bit0=CH0 ... bit4=CH4
 *   if (mask & (1<<2)) { ... }           // 中间传感器检测到黑线
 *
 *   int8_t pos = tracking_get_position(); // -4(最左) ~ +4(最右)
 * @endcode
 */
#ifndef TRACKING_H
#define TRACKING_H

#include <stdint.h>
#include <stdbool.h>

/** @brief 循迹通道数量 */
#define TRACKING_CHANNELS  5

void tracking_init(void);
bool tracking_read(uint8_t ch);
uint8_t tracking_read_all(void);
int8_t tracking_get_position(void);

#endif /* TRACKING_H */
