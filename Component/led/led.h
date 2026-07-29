/**
 * @file    led.h
 * @brief   双路信号灯接口 (PA9/PA29, 高电平点亮)
 *
 * 硬件: 引脚 → LED → GND, set=亮, clear=灭
 */
#ifndef LED_H
#define LED_H

#include <stdint.h>

void led_init(void);

/* ── 信号灯1: PA9 ── */
void led1_on(void);
void led1_off(void);
void led1_toggle(void);

/* ── 信号灯2: PA29 ── */
void led2_on(void);
void led2_off(void);
void led2_toggle(void);

#endif /* LED_H */
