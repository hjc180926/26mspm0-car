/**
 * @file    key.c
 * @brief   按键检测模块 (PB21, 现被循迹 CH2 占用)
 * @note    若 SysConfig 中 GPIO_KEY_USER 未配置, 函数退化为空桩
 */
#include "key.h"
#include "ti_msp_dl_config.h"

#ifdef GPIO_KEY_USER_PORT

#define KEY_PORT   GPIO_KEY_USER_PORT
#define KEY_PIN    GPIO_KEY_USER_PIN_KEY_USER_PIN

static bool last_raw  = false;
static bool debounced = false;

void key_init(void)
{
    last_raw  = false;
    debounced = false;
}

bool key_read_user(void)
{
    bool raw = (DL_GPIO_readPins(KEY_PORT, KEY_PIN) == 0);

    if (raw == last_raw) {
        debounced = raw;
    }
    last_raw = raw;

    return debounced;
}

#else

void key_init(void)          {}
bool key_read_user(void)     { return false; }

#endif
