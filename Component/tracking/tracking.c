/**
 * @file    tracking.c
 * @brief   5路循迹传感器模块 (软件消抖 + 位置解算)
 *
 * 引脚分配 (均为输入 + 内部上拉, 黑线=低电平=true):
 *   CH0: PB25  (最左)
 *   CH1: PB18
 *   CH2: PB21  (居中)  注: 当前与 KEY_USER 冲突, 需在 SysConfig 重新分配
 *   CH3: PB22            注: 当前与 LED 冲突, 需在 SysConfig 重新分配
 *   CH4: PA30  (最右)
 *
 * 需要在 SysConfig 中添加 5 个 GPIO 实例 (输入 + 内部上拉),
 * 生成 ti_msp_dl_config.h 后引脚宏自动就位。
 * 若未生成宏, 可临时用下方的 direct-port 回退定义。
 *
 * 消抖逻辑: 连续两次读到相同电平才更新 (与 key.c 一致)
 * 建议调用周期: 5~10ms
 */

#include "tracking.h"
#include "ti_msp_dl_config.h"

/* ── 引脚定义 (由 SysConfig 生成后自动生效) ── */
/* 若 SysConfig 尚未生成对应宏, 启用下方回退: */
#if 0
/* 临时回退: 直接用端口+位号 */
#define TRK_CH0_PORT  GPIOB
#define TRK_CH0_PIN   DL_GPIO_PIN_25
#define TRK_CH1_PORT  GPIOB
#define TRK_CH1_PIN   DL_GPIO_PIN_18
#define TRK_CH2_PORT  GPIOB
#define TRK_CH2_PIN   DL_GPIO_PIN_21
#define TRK_CH3_PORT  GPIOB
#define TRK_CH3_PIN   DL_GPIO_PIN_22
#define TRK_CH4_PORT  GPIOA
#define TRK_CH4_PIN   DL_GPIO_PIN_30
#endif

/** @brief 每通道的端口和引脚 */
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

/* ── 消抖状态 ── */
static bool last_raw[TRACKING_CHANNELS];
static bool debounced[TRACKING_CHANNELS];

/**
 * @brief  初始化消抖状态机
 */
void tracking_init(void)
{
    for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
        last_raw[i]  = false;
        debounced[i] = false;
    }
}

/**
 * @brief  读取单通道 (含消抖)
 * @param  ch  0=最左 ~ 4=最右
 * @return true=黑线(低电平), false=白底(高电平)
 */
bool tracking_read(uint8_t ch)
{
    if (ch >= TRACKING_CHANNELS) return false;

    /* 低电平 = 检测到黑线 = true */
    bool raw = (DL_GPIO_readPins(trk_pin[ch].port, trk_pin[ch].pin) != 0);

    if (raw == last_raw[ch]) {
        debounced[ch] = raw;
    }
    last_raw[ch] = raw;

    return debounced[ch];
}

/**
 * @brief  读取全部5路 (含消抖)
 * @return 位掩码, bit0=CH0(最左) ... bit4=CH4(最右)
 */
uint8_t tracking_read_all(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < TRACKING_CHANNELS; i++) {
        if (tracking_read(i)) {
            mask |= (uint8_t)(1 << i);
        }
    }
    return mask;
}

/**
 * @brief  获取黑线位置 (加权平均)
 * @return -4(最左) ~ +4(最右), 0=居中
 *
 * 权重: CH0=-4, CH1=-2, CH2=0, CH3=+2, CH4=+4
 * 无黑线时返回 0; 多路检测时按权重取平均
 */
