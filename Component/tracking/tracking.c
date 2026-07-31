/**
 * @file    tracking.c
 * @brief   8路循迹传感器模块 (软件消抖 + 位置解算)
 *
 * 引脚分配 (均为输入 + 内部上拉, 黑线=低电平=true):
 *   CH0: PB25  (最左)   CH4: PA30  (最右)
 *   CH1: PB18            CH5: PB0
 *   CH2: PB21  (居中)    CH6: PB1
 *   CH3: PB22            CH7: PB10
 */

#include "tracking.h"
#include "ti_msp_dl_config.h"

static const struct {
    GPIO_Regs *port;
    uint32_t   pin;
} trk_pin[TRACKING_CHANNELS] = {
#ifdef GPIO_TRK_CH0_PORT
    { GPIO_TRK_CH0_PORT,  GPIO_TRK_CH0_PIN_TRK_CH0_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_25                  },
#endif
#ifdef GPIO_TRK_CH1_PORT
    { GPIO_TRK_CH1_PORT,  GPIO_TRK_CH1_PIN_TRK_CH1_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_18                  },
#endif
#ifdef GPIO_TRK_CH2_PORT
    { GPIO_TRK_CH2_PORT,  GPIO_TRK_CH2_PIN_TRK_CH2_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_21                  },
#endif
#ifdef GPIO_TRK_CH3_PORT
    { GPIO_TRK_CH3_PORT,  GPIO_TRK_CH3_PIN_TRK_CH3_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_22                  },
#endif
#ifdef GPIO_TRK_CH4_PORT
    { GPIO_TRK_CH4_PORT,  GPIO_TRK_CH4_PIN_TRK_CH4_PIN  },
#else
    { GPIOA,              DL_GPIO_PIN_30                  },
#endif
#ifdef GPIO_TRK_CH5_PORT
    { GPIO_TRK_CH5_PORT,  GPIO_TRK_CH5_PIN_TRK_CH5_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_0                   },
#endif
#ifdef GPIO_TRK_CH6_PORT
    { GPIO_TRK_CH6_PORT,  GPIO_TRK_CH6_PIN_TRK_CH6_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_1                   },
#endif
#ifdef GPIO_TRK_CH7_PORT
    { GPIO_TRK_CH7_PORT,  GPIO_TRK_CH7_PIN_TRK_CH7_PIN  },
#else
    { GPIOB,              DL_GPIO_PIN_10                  },
#endif
};

static bool last_raw[TRACKING_CHANNELS];
static bool debounced[TRACKING_CHANNELS];

void tracking_init(void)
{
    for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
        last_raw[i]  = false;
        debounced[i] = false;
    }
}

bool tracking_read(uint8_t ch)
{
    if (ch >= TRACKING_CHANNELS) return false;
    /* 原版: 低电平=黑线=true=1 */
    bool raw = (DL_GPIO_readPins(trk_pin[ch].port, trk_pin[ch].pin) == 0);
    if (raw == last_raw[ch]) { debounced[ch] = raw; }
    last_raw[ch] = raw;
    return debounced[ch];
}

uint8_t tracking_read_all(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
        if (tracking_read(i)) { mask |= (uint8_t)(1 << i); }
    }
    return mask;
}

int8_t tracking_get_position(void)
{
    uint8_t  mask = tracking_read_all();
    int16_t  sum  = 0;
    uint8_t  cnt  = 0;
    static const int8_t weight[TRACKING_CHANNELS] = {-4, -3, -1, 0, 1, 2, 3, 4};
    for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
        if (mask & (uint8_t)(1 << i)) { sum += weight[i]; cnt++; }
    }
    if (cnt == 0) return 0;
    return (int8_t)(sum / cnt);
}
