// MSPM0 软件模拟 I2C 驱动 — 复用硬件 I2C0 引脚 (PA0-SDA, PA1-SCL)
// 调用前需先由 SYSCFG_DL_init() 配置 PA0/PA1 为 GPIO 输出（或运行时重配 IOMUX）
// 基于 MSPM0 SDK 2.04 DL_GPIO API

#include "IIC.h"
#include "ti_msp_dl_config.h"

/* 复用 SysConfig 中 I2C_0 的引脚定义 */
#define I2C_SDA_PORT    GPIO_I2C_0_SDA_PORT
#define I2C_SDA_PIN     GPIO_I2C_0_SDA_PIN
#define I2C_SCL_PORT    GPIO_I2C_0_SCL_PORT
#define I2C_SCL_PIN     GPIO_I2C_0_SCL_PIN
#define I2C_SDA_IOMUX   GPIO_I2C_0_IOMUX_SDA
#define I2C_SCL_IOMUX   GPIO_I2C_0_IOMUX_SCL

/* 约 5us 延时（80MHz 主频） */
static inline void i2c_delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 40; i++);
}

static inline void sda_output(void)
{
    DL_GPIO_enableOutput(I2C_SDA_PORT, I2C_SDA_PIN);
}

static inline void sda_input(void)
{
    DL_GPIO_disableOutput(I2C_SDA_PORT, I2C_SDA_PIN);
}

void IIC_Init(void)
{
    // 将 PA0/PA1 从 I2C 外设功能切为 GPIO 模式
    DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(I2C_SCL_IOMUX);

    // 初始均为高电平 (推挽输出)
    DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
    DL_GPIO_setPins(I2C_SDA_PORT, I2C_SDA_PIN);
    DL_GPIO_enableOutput(I2C_SCL_PORT, I2C_SCL_PIN);
    DL_GPIO_enableOutput(I2C_SDA_PORT, I2C_SDA_PIN);
}

// SCL 高电平时 SDA 下降沿 → 起始信号
void IIC_Start(void)
{
    sda_output();
    DL_GPIO_setPins(I2C_SDA_PORT, I2C_SDA_PIN);
    DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    DL_GPIO_clearPins(I2C_SDA_PORT, I2C_SDA_PIN);
    i2c_delay();
    DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
}

// SCL 高电平时 SDA 上升沿 → 停止信号
void IIC_Stop(void)
{
    sda_output();
    DL_GPIO_clearPins(I2C_SDA_PORT, I2C_SDA_PIN);
    i2c_delay();
    DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    DL_GPIO_setPins(I2C_SDA_PORT, I2C_SDA_PIN);
    i2c_delay();
}

// 等待从机应答：返回 0 表示应答，1 表示无应答
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ack = 0;
    sda_input();
    DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    if (DL_GPIO_readPins(I2C_SDA_PORT, I2C_SDA_PIN))
        ack = 1;
    DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    return ack;
}

// 发送一个字节 (MSB first)
void IIC_Send_Byte(uint8_t txd)
{
    sda_output();
    for (uint8_t i = 0; i < 8; i++) {
        DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
        i2c_delay();
        if (txd & 0x80)
            DL_GPIO_setPins(I2C_SDA_PORT, I2C_SDA_PIN);
        else
            DL_GPIO_clearPins(I2C_SDA_PORT, I2C_SDA_PIN);
        i2c_delay();
        DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
        i2c_delay();
        txd <<= 1;
    }
    DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
}

// 读取一个字节 (ack=1 发送应答, ack=0 发送非应答)
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t rx = 0;
    sda_input();
    for (uint8_t i = 0; i < 8; i++) {
        DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
        i2c_delay();
        DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
        i2c_delay();
        rx <<= 1;
        if (DL_GPIO_readPins(I2C_SDA_PORT, I2C_SDA_PIN))
            rx |= 0x01;
    }
    // 发送应答位
    sda_output();
    DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    if (ack)
        DL_GPIO_setPins(I2C_SDA_PORT, I2C_SDA_PIN);   // 非应答
    else
        DL_GPIO_clearPins(I2C_SDA_PORT, I2C_SDA_PIN);  // 应答
    i2c_delay();
    DL_GPIO_setPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    DL_GPIO_clearPins(I2C_SCL_PORT, I2C_SCL_PIN);
    i2c_delay();
    return rx;
}
