/**
 * @file    util.h
 * @brief   跨模块共享的小型内联工具函数。
 *
 * @details header-only：仅含 static inline 函数，无需编译为 .c，
 *          通过 IncludePath 中的 `../Component` 即可 `#include "common/util.h"`。
 *          用于消除各模块中重复的 abs/clamp 等小工具。
 */
#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H

#include <stdint.h>

/** @brief 系统毫秒滴答计数器，由 TIMER_0 ISR 每 1ms 递增。 */
extern volatile uint32_t g_tick_1ms;

/** @brief 返回系统启动后的 0.1s（100ms）计数。 */
static inline uint32_t sys_tick_ds(void)
{
    return g_tick_1ms / 100;
}

/** @brief 32 位有符号整数取绝对值（避免各模块重复实现）。 */
static inline int32_t util_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

#endif /* COMMON_UTIL_H */
