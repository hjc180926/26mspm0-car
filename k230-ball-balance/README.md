# K230 视觉钢球平衡（MG996R 舵机 PID）

小车上「钢球平衡」功能的视觉闭环实现：K230 摄像头识别钢珠位置，PID 输出舵机脉宽，控制摆杆倾角把钢珠稳定在目标位置。

与仓库根目录的 MSPM0 固件是**两套独立系统**：固件负责底盘运动，本目录负责平衡机构，两者通过小车本体组合在一起，代码互不依赖。

## 运行平台

| 项 | 值 |
|---|---|
| 主控 | 庐山派 K230（CanMV / MicroPython） |
| 舵机 | MG996R，GPIO46 → PWM2，50 Hz |
| 脉宽范围 | 500 – 2500 us，**中位 2150 us** |
| 视觉输入 | RGB888P 1080×720，AI 检测钢珠 |
| 模型配置 | `/sdcard/mp_deployment_source/deploy_config.json` |

## 关键参数

```python
SERVO_NEUTRAL_US   = 2150     # 机械中位（不是 1500）
SERVO_SLEW_US_PER_S = 900.0   # 限速，防止舵机冲击
PID_KP, PID_KI, PID_KD = 35.0, 0.0, 10.0
PID_OUTPUT_LIMIT_US = 200.0   # 输出限幅 ±200us
PID_INTEGRAL_ZONE_CM = 1.5    # 只在误差 <1.5cm 时积分
PID_DEADBAND_CM     = 0.10    # 死区，抑制抖动
PID_D_FILTER_TAU_S  = 0.08    # 微分一阶滤波
CENTER_X_PIXEL = 540          # 目标中心像素
PIXEL_PER_CM   = 43.2         # 像素/厘米标定值
```

## 保护逻辑

- **启动保持** 1.5 s 内不闭环，让钢珠先稳定
- **丢帧保护**：连续 3 帧检测不到钢珠 → PID 复位、舵机回中
- **输出限速**：900 us/s 的 slew rate，避免舵机猛打
- **积分限幅与抗饱和**：输出饱和时冻结积分累加

## 调参建议

1. 先调 `PIXEL_PER_CM` 和 `CENTER_X_PIXEL`：让静止时显示的位置接近 0
2. `KI` 保持 0，先只用 PD；纯比例就能稳住的场合不必加积分
3. 抖动大 → 加大 `PID_D_FILTER_TAU_S` 或减小 `KP`
4. 回中慢 → 提高 `KP`，但要配合 `SERVO_SLEW_US_PER_S` 一起看
5. 有稳态误差且需要消除时，再小步加 `KI`（0.1 起），并收紧 `PID_INTEGRAL_ZONE_CM`

## 屏幕显示

脚本会在 LCD 上叠加显示：钢珠位置、目标、误差、P/I/D 各项输出、舵机脉宽与估算角度、舵机行程条。调参时不用接串口也能看到实时状态。

## 注意

- `SERVO_NEUTRAL_US = 2150` 是这台机构的实测中位，**换舵机或换摆杆后必须重新标定**
- 舵机供电要独立，MG996R 堵转电流可达 2 A 以上，直接从 K230 板取电会掉压复位
