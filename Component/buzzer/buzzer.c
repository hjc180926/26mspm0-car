/**
 * @file    buzzer.c
 * @brief   蜂鸣器控制 (PA7, 低电平触发)
 *          clear=响, set=停
 */

#include "buzzer.h"
#include "ti_msp_dl_config.h"

#ifdef BUZZER_PORT

void buzzer_init(void)
{
    buzzer_off();  /* 初始关闭 */
}

void buzzer_on(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_7_PIN);  /* 低电平=响 */
}

void buzzer_off(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_7_PIN);    /* 高电平=停 */
}

void buzzer_toggle(void)
{
    DL_GPIO_togglePins(BUZZER_PORT, BUZZER_PIN_7_PIN);
}

#else

void buzzer_init(void)   {}
void buzzer_on(void)     {}
void buzzer_off(void)    {}
void buzzer_toggle(void)  {}

#endif
