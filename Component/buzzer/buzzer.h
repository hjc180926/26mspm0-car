/**
 * @file    buzzer.h
 * @brief   蜂鸣器模块接口 (PA7, 低电平触发)
 */

#ifndef BUZZER_H
#define BUZZER_H

void buzzer_init(void);

/** @brief 蜂鸣器响 (低电平) */
void buzzer_on(void);

/** @brief 蜂鸣器停 (高电平) */
void buzzer_off(void);

/** @brief 蜂鸣器翻转 */
void buzzer_toggle(void);

#endif /* BUZZER_H */
