/**
 * @file    tracking.h
 * @brief   8路循迹传感器模块接口
 *
 * 传感器: 红外对管, 黑线输出低电平, 白底输出高电平
 *         内部上拉, 黑线=按下(true), 白底=释放(false)
 *
 * 通道布局:
 *   CH0     CH1     CH2     CH3     CH4     CH5     CH6     CH7
 *   PB25   PB18   PB21    PB22    PA30    PB0     PB1     PB10
 */
#ifndef TRACKING_H
#define TRACKING_H

#include <stdint.h>
#include <stdbool.h>

#define TRACKING_CHANNELS  8

void tracking_init(void);
bool tracking_read(uint8_t ch);
uint8_t tracking_read_all(void);
int8_t tracking_get_position(void);

#endif /* TRACKING_H */
