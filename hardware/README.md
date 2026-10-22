# 硬件资料

## 主板原理图

`schematic-V1.0-2026-08-01.pdf` —— 26 电赛小车主板，嘉立创 EDA 绘制，V1.0，A4 单页。

设计信息：创建 2026-07-22，更新 2026-07-23。

### 板载资源

| 模块 | 说明 | 关键引脚 |
|---|---|---|
| 主控 | LCKFB-TMX-MSPM0G3507 核心板 | — |
| 电源输入 | DC005 座 + XT60，LM2596 降压 12V → 5V | — |
| 电机驱动 | TB6612，两路电机输出 CN3 / CN4 | PWMA-PA12, AIN1-PB4, AIN2-PB3, STBY-PB5, BIN1-PB2, BIN2-PB20 |
| 灰度传感器 | 8 路 | PB0, PB1, PA10, PA11, PB21, PB22, PA22, PB23 |
| OLED | I2C 接口 | SCL-PA31, SDA-PA28 |
| 陀螺仪 | MPU6050，I2C | SCL-PA31, SDA-PA28 |
| 无线 | NRF24L01，SPI | PB18, PB13, PB26 等 |
| 按键 | 4 个独立按键 | PB20, PB19, PB9, PB8 |
| RGB | WS2812 类 | PA2 |
| 蜂鸣器 | 5 kHz | PA7 |
| 调试串口 | UART0 / UART2 / UART3 | PA10/PA11、PA23/PA24、PB6/PB7 |

### 注意

- OLED 与 MPU6050 **共用 I2C（PA31 / PA28）**，挂同一总线，地址不能冲突
- 灰度 8 路里 PA10 / PA11 同时是 UART0 的 TX / RX，若用串口调试需二选一
- 电机供电与逻辑供电分离，TB6612 的 VM 接 12V、VCC 接 5V
