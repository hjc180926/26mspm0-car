/**
 * @file    led.c
 * @brief   双路信号灯控制 (PA9=LED_SIG_1, PA29=LED_SIG_2, 高电平点亮)
 * @note    GPIO 初始化由 SYSCFG_DL_init() 完成
 */
#include "led.h"
#include "ti_msp_dl_config.h"

/* ── 信号灯1: PA9 ── */
#ifdef LED_SIG_1_PORT

void led1_on(void)
{
    DL_GPIO_setPins(LED_SIG_1_PORT, LED_SIG_1_PIN_9_PIN);
}

void led1_off(void)
{
    DL_GPIO_clearPins(LED_SIG_1_PORT, LED_SIG_1_PIN_9_PIN);
}

void led1_toggle(void)
{
    DL_GPIO_togglePins(LED_SIG_1_PORT, LED_SIG_1_PIN_9_PIN);
}

#else

void led1_on(void)     {}
void led1_off(void)    {}
void led1_toggle(void)  {}

#endif

/* ── 信号灯2: PA29 ── */
#ifdef LED_SIG_2_PORT

void led2_on(void)
{
    DL_GPIO_setPins(LED_SIG_2_PORT, LED_SIG_2_PIN_29_PIN);
}

void led2_off(void)
{
    DL_GPIO_clearPins(LED_SIG_2_PORT, LED_SIG_2_PIN_29_PIN);
}

void led2_toggle(void)
{
    DL_GPIO_togglePins(LED_SIG_2_PORT, LED_SIG_2_PIN_29_PIN);
}

#else

void led2_on(void)     {}
void led2_off(void)    {}
void led2_toggle(void)  {}

#endif

void led_init(void)
{
    /* GPIO 由 SYSCFG_DL_init() 配置, 无需额外操作 */
}
