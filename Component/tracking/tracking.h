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
#define TRACKING_CHANNELS  8

/**
 * @brief  初始化循迹模块
 * @note   清零消抖状态机, 应在 SYSCFG_DL_init() 之后调用
 */
void tracking_init(void);

/**
 * @brief  读取单通道状态 (含软件消抖)
 * @param  ch  通道编号 0~4
 * @return true=黑线(低电平), false=白底(高电平)
 * @note   消抖逻辑: 连续两次读到相同电平才更新, 建议调用周期 5~10ms
 */
bool tracking_read(uint8_t ch);

/**
 * @brief  读取全部5路状态 (含消抖)
 * @return 位掩码: bit0=CH0, bit1=CH1, ..., bit4=CH4
 *         对应位=1 表示该通道检测到黑线
 */
uint8_t tracking_read_all(void);

/**
 * @brief  获取黑线位置 (加权平均)
 * @return -4(最左) ~ +4(最右), 0=居中
 *         无黑线时返回 0 (保持上次)
 * @note   权重: CH0=-4, CH1=-2, CH2=0, CH3=+2, CH4=+4
 */
int8_t tracking_get_position(void);

#endif /* TRACKING_H */
