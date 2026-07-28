/**
 * @file    led.c
 * @brief   LED 指示灯控制模块 (PB22, 现被循迹 CH3 占用)
 * @note    若 SysConfig 中 GPIO1 未配置, 所有函数退化为空桩
 */
#include "led.h"
#include "ti_msp_dl_config.h"

#ifdef LED_PORT

void led_init(void)
{
    /* GPIO 已在 SYSCFG_DL_init() 中初始化, 无需额外配置 */
}

void led_toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN_22_PIN);
}

void led_on(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN_22_PIN);
}

void led_off(void)
{
    DL_GPIO_setPins(LED_PORT, LED_PIN_22_PIN);
}

#else

void led_init(void)   {}
void led_toggle(void)  {}
void led_on(void)     {}
void led_off(void)    {}

#endif
