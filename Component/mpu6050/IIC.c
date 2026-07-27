// MSPM0 软件模拟 I2C 驱动 (开漏SDA + 推挽SCL)
// SDA 只拉低不放高，高电平靠外部上拉电阻
// PA0-SDA, PA1-SCL, 80MHz 主频

#include "IIC.h"
#include "ti_msp_dl_config.h"

#define I2C_SDA_PORT    GPIO_I2C_0_SDA_PORT
#define I2C_SDA_PIN     GPIO_I2C_0_SDA_PIN
#define I2C_SCL_PORT    GPIO_I2C_0_SCL_PORT
#define I2C_SCL_PIN     GPIO_I2C_0_SCL_PIN
#define I2C_SDA_IOMUX   GPIO_I2C_0_IOMUX_SDA
#define I2C_SCL_IOMUX   GPIO_I2C_0_IOMUX_SCL

/* 延时 ~5us @80MHz */
static inline void i2c_delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 400; i++);
}

/* ── SDA 控制：只拉低，不放高 ── */

/* SDA 输出低电平 (开漏拉低) */
static inline void sda_low(void)
{
    DL_GPIO_clearPins(I2C_SDA_PORT, I2C_SDA_PIN);
    DL_GPIO_enableOutput(I2C_SDA_PORT, I2C_SDA_PIN);
}

/* SDA 释放为高阻 (外部上拉 → 高电平) */
static inline void sda_release(void)
{
    DL_GPIO_disableOutput(I2C_SDA_PORT, I2C_SDA_PIN);
}

/* 读 SDA 电平 */
static inline uint32_t sda_read(void)
{
    return DL_GPIO_readPins(I2C_SDA_PORT, I2C_SDA_PIN);
}

/* ── SCL 控制：推挽 ── */
static inline void scl_high(void) { DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN); }
static inline void scl_low(void)  { DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN); }

/* ═══════════════════════════════════════════════════════════════════════════
   公开接口
   ═══════════════════════════════════════════════════════════════════════════ */

void IIC_Init(void)
{
    /* SDA: GPIO 输出模式，初始写0，释放为高阻 */
    DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);
    DL_GPIO_clearPins(I2C_SDA_PORT, I2C_SDA_PIN);
    sda_release();  /* 释放 → 上拉高 */

    /* SCL: GPIO 推挽输出，初始高 */
    DL_GPIO_initDigitalOutput(I2C_SCL_IOMUX);
    DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
    DL_GPIO_enableOutput(I2C_SCL_PORT, I2C_SCL_PIN);
}

void IIC_Start(void)
{
    sda_release();       /* SDA 高 */
    scl_high();          /* SCL 高 */
    i2c_delay();
    sda_low();           /* SDA ↓ (起始条件) */
    i2c_delay();
    scl_low();           /* SCL ↓ */
    i2c_delay();
}

void IIC_Stop(void)
{
    sda_low();           /* SDA 低 */
    scl_high();          /* SCL ↑ */
    i2c_delay();
    sda_release();       /* SDA ↑ (停止条件) */
    i2c_delay();
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t ack;
    sda_release();       /* 释放SDA, 等待从机拉低 */
    scl_high();          /* SCL ↑, 从机此时应答 */
    i2c_delay();
    ack = sda_read() ? 1 : 0;
    scl_low();           /* SCL ↓ */
    i2c_delay();
    return ack;          /* 0=ACK, 1=NACK */
}

void IIC_Send_Byte(uint8_t txd)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (txd & 0x80) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        scl_high();      /* SCL ↑, 从机采样 */
        i2c_delay();
        scl_low();       /* SCL ↓ */
        i2c_delay();
        txd <<= 1;
    }
    sda_release();       /* 释放SDA, 等待ACK */
    i2c_delay();
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t rx = 0;
    sda_release();       /* 释放SDA */
    for (uint8_t i = 0; i < 8; i++) {
        scl_high();      /* SCL ↑, 从机输出数据 */
        i2c_delay();
        rx <<= 1;
        if (sda_read()) {
            rx |= 0x01;
        }
        scl_low();       /* SCL ↓ */
        i2c_delay();
    }
    /* 发送应答位 */
    if (ack) {
        sda_release();   /* NACK: 释放 */
    } else {
        sda_low();       /* ACK: 拉低 */
    }
    i2c_delay();
    scl_high();          /* SCL ↑ */
    i2c_delay();
    scl_low();           /* SCL ↓ */
    i2c_delay();
    sda_release();       /* 释放SDA */
    return rx;
}
