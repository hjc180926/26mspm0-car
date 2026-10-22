"""
庐山派 K230 钢珠视觉PID舵机闭环测试
- 基于 servo_test.py 扩展
- 舵机参数：中位2150us，限幅500-2500us(无限制)
- 屏幕显示：位置/目标/误差/PID各项/舵机脉宽/角度
"""

import os
import ujson
import aicube
from media.sensor import *
from media.display import *
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import gc
from machine import PWM, FPIOA, UART


# =========================== 舵机配置 ===========================
SERVO_GPIO = 46
                                             # 舵机GPIO引脚号
SERVO_PWM_CH = 2
                                            # PWM通道号
SERVO_FREQ_HZ = 50
                                        # PWM频率(Hz)，标准舵机50Hz

SERVO_NEUTRAL_US = 2150
                                 # 舵机中位脉宽(us)，摆杆水平时的值
SERVO_MIN_US = 1850
                                     # 舵机最小脉宽(us)，硬件安全兜底
SERVO_MAX_US = 2450
                                    # 舵机最大脉宽(us)，硬件安全兜底

SERVO_DIRECTION = 1
                                      # 闭环方向：1=正向，-1=反向
SERVO_SLEW_US_PER_S = 2200.0
# =========================== UART2 配置 ===========================
UART_ID = UART.UART2
UART_TX_GPIO = 05
UART_RX_GPIO = 06
UART_BAUDRATE = 115200

# =========================== 第3项 序列状态 ===========================
SEQ_IDLE = 0
SEQ_GO_NEG5 = 1
SEQ_WAIT_NEG5 = 2
SEQ_GO_POS5 = 3
SEQ_DONE = 4

seq_state = SEQ_IDLE
seq_arrive_count = 0
SEQ_ARRIVE_THRESHOLD_CM = 0.5
SEQ_ARRIVE_FRAMES = 5
                             # 舵机最大变化速率(us/s)，防止突变冲击

# =========================== 视觉标定 ===========================
CENTER_X_PIXEL = 562
                                    # 摆杆中心(0cm刻度)在AI画面中的X像素坐标
PIXEL_PER_CM = 43.2
                                     # 每1cm对应的像素数，需实际标定
VALID_POS_LIMIT_CM = 13.0
                              # 有效钢珠位置范围(cm)，超出视为无效

# =========================== PID 参数 ===========================
PID_KP = 46.0
                                             # 比例系数(us/cm)
PID_KI = 0.05
                                              # 积分系数(us/(cm·s))
PID_KD = 26.0
                                             # 微分系数(us·s/cm)
PID_OUTPUT_LIMIT_US = 320.0
                             # PID输出限幅(us)，有效范围=中位±此值
PID_INTEGRAL_LIMIT = 6.0
                                # 积分状态限幅(cm·s)，防止积分饱和
PID_INTEGRAL_ZONE_CM = 2.0
                            # 积分分离区(cm)，仅此误差范围内积分
PID_DEADBAND_CM = 0.03
                                 # 位置死区(cm)，误差小于此值不输出
PID_D_FILTER_TAU_S = 0.025
                             # D项低通滤波时间常数(s)，抑制高频噪声

# =========================== 目标与保护 ===========================
TARGET_POS_CM = 0.0
                                    # 上电默认目标位置(cm)，0=摆杆中心
CONTROL_ENABLE = True
                                   # 上电是否自动启用闭环
CONTROL_STARTUP_HOLD_MS = 1500
                         # 启动后保持中位时间(ms)
DETECTION_LOST_FRAMES = 3
                             # 连续丢帧数，超过后回中
DEBUG_PRINT_INTERVAL_MS = 300
                        # 终端状态打印间隔(ms)

# =========================== 显示配置 ===========================
display_mode = "lcd"
if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1080, 16)
OUT_RGB888P_HEIGH = 720

# AI 模型配置
root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"
debug_mode = 0

color_four = [(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
        (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
        (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
        (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
        (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157),
        (255, 110, 76, 0), (255, 174, 57, 255), (255, 199, 100, 0), (255, 72, 0, 118),
        (255, 255, 179, 240), (255, 0, 125, 92), (255, 209, 0, 151), (255, 188, 208, 182),
        (255, 0, 220, 176), (255, 255, 99, 164), (255, 92, 0, 73), (255, 133, 129, 255),
        (255, 78, 180, 255), (255, 0, 228, 0), (255, 174, 255, 243), (255, 45, 89, 255),
        (255, 134, 134, 103), (255, 145, 148, 174), (255, 255, 208, 186),
        (255, 197, 226, 255), (255, 171, 134, 1), (255, 109, 63, 54), (255, 207, 138, 255),
        (255, 151, 0, 95), (255, 9, 80, 61), (255, 84, 105, 51), (255, 74, 65, 105),
        (255, 166, 196, 102), (255, 208, 195, 210), (255, 255, 109, 65), (255, 0, 143, 149),
        (255, 179, 0, 194), (255, 209, 99, 106), (255, 5, 121, 0), (255, 227, 255, 205),
        (255, 147, 186, 208), (255, 153, 69, 1), (255, 3, 95, 161), (255, 163, 255, 0),
        (255, 119, 0, 170), (255, 0, 182, 199), (255, 0, 165, 120), (255, 183, 130, 88),
        (255, 95, 32, 0), (255, 130, 114, 135), (255, 110, 129, 133), (255, 166, 74, 118),
        (255, 219, 142, 185), (255, 79, 210, 114), (255, 178, 90, 62), (255, 65, 70, 15),
        (255, 127, 167, 115), (255, 59, 105, 106), (255, 142, 108, 45), (255, 196, 172, 0),
        (255, 95, 54, 80), (255, 128, 76, 255), (255, 201, 57, 1), (255, 246, 0, 122),
        (255, 191, 162, 208)]


# =============================================================================
#                           工具函数
# =============================================================================

def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def pulse_to_angle_deg(pulse_us):
    """脉宽转估算角度。500≈-60°, 2150≈0°, 2500≈+60°"""
    if pulse_us >= SERVO_NEUTRAL_US:
        return (pulse_us - SERVO_NEUTRAL_US) / (SERVO_MAX_US - SERVO_NEUTRAL_US) * 60.0
    else:
        return (pulse_us - SERVO_NEUTRAL_US) / (SERVO_NEUTRAL_US - SERVO_MIN_US) * 60.0

# =============================================================================
#                           UART2 通信
# =============================================================================

uart_dev = None
uart_rx_buf = ""

def uart_init():
    global uart_dev, uart_rx_buf
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)
    uart_dev = UART(
        UART_ID,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0
    )
    uart_rx_buf = ""
    print("[UART] UART2 TX=GPIO%d RX=GPIO%d %d@8N1" %
          (UART_TX_GPIO, UART_RX_GPIO, UART_BAUDRATE))


def uart_deinit():
    global uart_dev
    if uart_dev is not None:
        try:
            uart_dev.deinit()
        except:
            pass
    uart_dev = None


def uart_send(msg):
    if uart_dev is not None:
        uart_dev.write(msg + "\r\n")


def uart_poll():
    """非阻塞接收，返回收到的完整行列表"""
    global uart_dev, uart_rx_buf
    lines = []
    if uart_dev is None:
        return lines
    try:
        data = uart_dev.read()
        if data:
            try:
                uart_rx_buf += data.decode("utf-8")
            except:
                pass
            if len(uart_rx_buf) > 256:
                uart_rx_buf = uart_rx_buf[-256:]
            while "\n" in uart_rx_buf:
                line, uart_rx_buf = uart_rx_buf.split("\n", 1)
                line = line.replace("\r", "").strip()
                if line:
                    lines.append(line.upper())
    except Exception as e:
        print("[UART] RX异常:", e)
    return lines


# =============================================================================
#                           第3项 序列控制
# =============================================================================

def start_requirement3():
    """启动第3项：球从0→-5cm→+5cm，停在+5cm"""
    global seq_state, seq_arrive_count, TARGET_POS_CM
    seq_state = SEQ_GO_NEG5
    seq_arrive_count = 0
    TARGET_POS_CM = -5.0
    pid.reset()
    uart_send("OK F1 TARGET=-5")
    print("[SEQ3] 启动: 0 -> -5cm")


def sequence_update(ball_pos_cm):
    """每帧调用，检测是否到达目标并切换"""
    global seq_state, seq_arrive_count, TARGET_POS_CM
    if seq_state == SEQ_IDLE or seq_state == SEQ_DONE:
        return

    if abs(ball_pos_cm - TARGET_POS_CM) <= SEQ_ARRIVE_THRESHOLD_CM:
        seq_arrive_count += 1
    else:
        seq_arrive_count = 0

    if seq_arrive_count >= SEQ_ARRIVE_FRAMES:
        seq_arrive_count = 0
        if seq_state == SEQ_GO_NEG5:
            seq_state = SEQ_GO_POS5
            TARGET_POS_CM = 5.0
            pid.reset()
            uart_send("OK ARRIVE NEG5 NOW TARGET=+5")
            print("[SEQ3] 到达-5cm，折返 -> +5cm")
        elif seq_state == SEQ_GO_POS5:
            seq_state = SEQ_DONE
            uart_send("OK DONE POS5")
            print("[SEQ3] 到达+5cm，完成")


def seq_state_name():
    if seq_state == SEQ_IDLE:
        return "待命"
    elif seq_state == SEQ_GO_NEG5:
        return "去-5cm"
    elif seq_state == SEQ_GO_POS5:
        return "去+5cm"
    elif seq_state == SEQ_DONE:
        return "完成+5cm"
    return "?"


# =============================================================================
#                           舵机驱动
# =============================================================================

servo_pwm = None
servo_current_us = SERVO_NEUTRAL_US
servo_last_update_ms = None


def pulse_us_to_duty_u16(pulse_us):
    period_us = 1000000.0 / SERVO_FREQ_HZ
    return int(clamp(pulse_us / period_us * 65535.0, 0.0, 65535.0))


def servo_init():
    global servo_pwm, servo_current_us, servo_last_update_ms
    fpioa = FPIOA()
    fpioa.set_function(SERVO_GPIO, getattr(FPIOA, "PWM%d" % SERVO_PWM_CH))
    servo_pwm = PWM(SERVO_PWM_CH)
    servo_pwm.freq(SERVO_FREQ_HZ)
    servo_current_us = SERVO_NEUTRAL_US
    servo_last_update_ms = utime.ticks_ms()
    servo_pwm.duty_u16(pulse_us_to_duty_u16(servo_current_us))
    print("[SERVO] GPIO%d -> PWM%d, %dHz, neutral=%dus" %
          (SERVO_GPIO, SERVO_PWM_CH, SERVO_FREQ_HZ, SERVO_NEUTRAL_US))


def servo_write_us(target_us, now_ms=None, use_slew=True):
    global servo_current_us, servo_last_update_ms
    if servo_pwm is None:
        return servo_current_us

    target_us = float(clamp(target_us, SERVO_MIN_US, SERVO_MAX_US))
    if now_ms is None:
        now_ms = utime.ticks_ms()

    if use_slew and servo_last_update_ms is not None:
        dt = utime.ticks_diff(now_ms, servo_last_update_ms) / 1000.0
        dt = clamp(dt, 0.0, 0.20)
        max_step = SERVO_SLEW_US_PER_S * dt
        target_us = clamp(target_us, servo_current_us - max_step, servo_current_us + max_step)

    servo_current_us = target_us
    servo_last_update_ms = now_ms
    servo_pwm.duty_u16(pulse_us_to_duty_u16(servo_current_us))
    return servo_current_us


def servo_deinit():
    global servo_pwm
    if servo_pwm is None:
        return
    try:
        servo_write_us(SERVO_NEUTRAL_US, use_slew=False)
        time.sleep(0.2)
        servo_pwm.duty_u16(0)
        servo_pwm.deinit()
    except Exception as e:
        print("[SERVO] 释放异常:", e)
    servo_pwm = None


# =============================================================================
#                           PID 控制器
# =============================================================================

class PracticalPID:
    def __init__(self, kp, ki, kd, output_limit, integral_limit, integral_zone, deadband, d_tau):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.integral_zone = abs(integral_zone)
        self.deadband = abs(deadband)
        self.d_tau = max(d_tau, 0.0)
        self.reset()

    def reset(self, measurement=None, now_ms=None):
        self.integral = 0.0
        self.d_filtered = 0.0
        self.last_measurement = measurement
        self.last_ms = now_ms
        self.last_dt = 0.0
        self.last_error = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_output = 0.0

    def update(self, target, measurement, now_ms):
        if self.last_measurement is None or self.last_ms is None:
            self.reset(measurement, now_ms)
            return 0.0

        dt = utime.ticks_diff(now_ms, self.last_ms) / 1000.0
        if dt <= 0.0 or dt > 0.50:
            self.reset(measurement, now_ms)
            return 0.0
        self.last_dt = dt

        error = target - measurement
        control_error = 0.0 if abs(error) <= self.deadband else error

        d_measurement = (measurement - self.last_measurement) / dt
        d_raw = -d_measurement
        if self.d_tau > 0.0:
            alpha = dt / (self.d_tau + dt)
            self.d_filtered += alpha * (d_raw - self.d_filtered)
        else:
            self.d_filtered = d_raw

        integral_candidate = self.integral
        if abs(error) <= self.integral_zone:
            integral_candidate += control_error * dt
            integral_candidate = clamp(integral_candidate, -self.integral_limit, self.integral_limit)

        unsaturated = (self.kp * control_error +
                       self.ki * integral_candidate +
                       self.kd * self.d_filtered)
        output = clamp(unsaturated, -self.output_limit, self.output_limit)

        if (output == unsaturated or
            (output >= self.output_limit and control_error < 0.0) or
            (output <= -self.output_limit and control_error > 0.0)):
            self.integral = integral_candidate

        self.last_error = error
        self.p_term = self.kp * control_error
        self.i_term = self.ki * self.integral
        self.d_term = self.kd * self.d_filtered
        self.last_output = output
        self.last_measurement = measurement
        self.last_ms = now_ms
        return output


# =============================================================================
#                           OSD 显示
# =============================================================================

def draw_osd(osd_img, ball_pos_cm, ball_cx_pixel, score, position_valid,
             disp_x1, disp_y1, disp_x2, disp_y2, disp_cx, disp_cy, cls_id,
             pid, servo_pwm_us):
    osd_img.clear()

    # --- 检测框和中心点 ---
    if position_valid:
        osd_img.draw_rectangle(disp_x1, disp_y1, disp_x2 - disp_x1, disp_y2 - disp_y1,
                               color=color_four[cls_id][1:])
        osd_img.draw_circle(disp_cx, disp_cy, 4, color=(255, 255, 0, 255), thickness=2)

    # --- 左侧：位置信息 ---
    x0 = 15
    y = 25
    if position_valid:
        osd_img.draw_string_advanced(x0, y, 30,
                                     "位置: %+.2f cm" % ball_pos_cm,
                                     color=(255, 255, 255, 255))
    else:
        osd_img.draw_string_advanced(x0, y, 30,
                                     "未检测到钢珠", color=(255, 255, 0, 0))

    y += 40
    osd_img.draw_string_advanced(x0, y, 26,
                                 "目标: %+.2f cm" % TARGET_POS_CM,
                                 color=(255, 200, 200, 200))
    y += 32
    err = TARGET_POS_CM - ball_pos_cm if position_valid else 0
    osd_img.draw_string_advanced(x0, y, 26,
                                 "误差: %+.2f cm" % err,
                                 color=(255, 200, 200, 200))

    # --- 右侧：PID 输出 ---
    rx = 440
    ry = 25
    osd_img.draw_string_advanced(rx, ry, 26,
                                 "P: %+.1f us" % pid.p_term,
                                 color=(255, 255, 150, 150))
    ry += 32
    osd_img.draw_string_advanced(rx, ry, 26,
                                 "I: %+.1f us" % pid.i_term,
                                 color=(255, 150, 255, 150))
    ry += 32
    osd_img.draw_string_advanced(rx, ry, 26,
                                 "D: %+.1f us" % pid.d_term,
                                 color=(255, 150, 150, 255))
    ry += 32
    osd_img.draw_string_advanced(rx, ry, 26,
                                 "PID总: %+.1f us" % pid.last_output,
                                 color=(255, 255, 255, 150))

    # --- 底部：舵机状态 ---
    servo_angle = pulse_to_angle_deg(servo_pwm_us)
    sy = 360
    osd_img.draw_string_advanced(x0, sy, 30,
                                 "舵机: %d us  |  %+.1f deg" % (int(servo_pwm_us), servo_angle),
                                 color=(255, 0, 255, 0))
    sy += 38

    # 舵机进度条
    bar_x, bar_w, bar_h = 30, 740, 18
    osd_img.draw_rectangle(bar_x, sy, bar_w, bar_h, color=(255, 60, 60, 60), thickness=1)
    ratio = (servo_pwm_us - SERVO_MIN_US) / (SERVO_MAX_US - SERVO_MIN_US)
    fill_w = int(ratio * bar_w)
    osd_img.draw_rectangle(bar_x, sy, fill_w, bar_h, color=(255, 0, 200, 100))
    # 中位线
    mid_x = int(bar_x + (SERVO_NEUTRAL_US - SERVO_MIN_US) / (SERVO_MAX_US - SERVO_MIN_US) * bar_w)
    osd_img.draw_line(mid_x, sy - 4, mid_x, sy + bar_h + 4, color=(255, 255, 255, 255), thickness=2)

    # 底部标注
    sy += bar_h + 6
    osd_img.draw_string_advanced(bar_x, sy, 18,
                                 "%dus" % SERVO_MIN_US, color=(255, 140, 140, 140))
    osd_img.draw_string_advanced(mid_x - 28, sy, 18,
                                 "%dus" % SERVO_NEUTRAL_US, color=(255, 140, 140, 140))
    osd_img.draw_string_advanced(bar_x + bar_w - 40, sy, 18,
                                 "%dus" % SERVO_MAX_US, color=(255, 140, 140, 140))

    # 状态指示
    sy += 30
    status = "闭环运行" if CONTROL_ENABLE else "闭环停止"
    sc = (255, 0, 255, 0) if CONTROL_ENABLE else (255, 255, 100, 100)
    osd_img.draw_string_advanced(x0, sy, 24, status, color=sc)

    # 第3项序列状态
    if seq_state != SEQ_IDLE:
        sy += 28
        sname = seq_state_name()
        osd_img.draw_string_advanced(x0, sy, 24,
                                 "SEQ3: " + sname, color=(255, 255, 200, 0))


# =============================================================================
#                           AI 检测 + PID 闭环
# =============================================================================

def read_deploy_config(config_path):
    with open(config_path, 'r') as json_file:
        config = ujson.load(json_file)
    return config


class ScopedTiming:
    def __init__(self, info="", enable_profile=True):
        self.info = info
        self.enable_profile = enable_profile

    def __enter__(self):
        if self.enable_profile:
            self.start_time = time.time_ns()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enable_profile:
            elapsed_time = time.time_ns() - self.start_time
            print(f"{self.info} took {elapsed_time / 1000000:.2f} ms")


def detection():
    print("=== 钢珠视觉PID闭环 | 舵机2150us中位 ===")
    pid = PracticalPID(PID_KP, PID_KI, PID_KD,
                       PID_OUTPUT_LIMIT_US, PID_INTEGRAL_LIMIT,
                       PID_INTEGRAL_ZONE_CM, PID_DEADBAND_CM,
                       PID_D_FILTER_TAU_S)
    lost_frames = 0
    control_start_ms = utime.ticks_ms()
    last_debug_print_ms = control_start_ms
    pid_output_us = 0.0
    last_ball_pos_cm = 0.0
    last_position_valid = False
    last_score = 0.0

    deploy_conf = read_deploy_config(config_path)
    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]

    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    strides = [8, 16, 32]

    ori_w = OUT_RGB888P_WIDTH
    ori_h = OUT_RGB888P_HEIGH
    width = kmodel_frame_size[0]
    height = kmodel_frame_size[1]
    ratiow = float(width) / ori_w
    ratioh = float(height) / ori_h
    ratio = ratiow if ratiow < ratioh else ratioh
    new_w = int(ratio * ori_w)
    new_h = int(ratio * ori_h)
    dw = float(width - new_w) / 2
    dh = float(height - new_h) / 2
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))

    kpu = nn.kpu()
    ai2d = nn.ai2d()
    kpu.load_kmodel(root_path + kmodel_name)
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build([1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH], [1, 3, height, width])

    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=False)
    else:
        Display.init(Display.LT9611, to_ide=False)

    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    try:
        servo_init()
        uart_init()
        MediaManager.init()
        sensor.run()
        rgb888p_img = None
        ai2d_input_tensor = None
        data = np.ones((1, 3, width, height), dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)

        while True:
            # UART命令轮询
            for cmd in uart_poll():
                if cmd in ("F1", "START", "RUN3"):
                    start_requirement3()

            # 第3项序列状态机
            if seq_state not in (SEQ_IDLE, SEQ_DONE) and last_position_valid:
                sequence_update(last_ball_pos_cm)

            with ScopedTiming("total", debug_mode > 0):
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)

                if rgb888p_img.format() == image.RGBP888:
                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)
                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)

                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    kpu.run()

                    results = []
                    for i in range(kpu.outputs_size()):
                        out_data = kpu.get_output_tensor(i)
                        result = out_data.to_numpy()
                        result = result.reshape((result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3]))
                        del out_data
                        results.append(result)
                    gc.collect()

                    if model_type == "AnchorBaseDet":
                        det_boxes = aicube.anchorbasedet_post_process(
                            results[0], results[1], results[2],
                            kmodel_frame_size, frame_size, strides,
                            num_classes, confidence_threshold, nms_threshold, anchors, nms_option)
                    elif model_type == "GFLDet":
                        det_boxes = aicube.gfldet_post_process(
                            results[0], results[1], results[2],
                            kmodel_frame_size, frame_size, strides,
                            num_classes, confidence_threshold, nms_threshold, nms_option)
                    else:
                        det_boxes = aicube.anchorfreedet_post_process(
                            results[0], results[1], results[2],
                            kmodel_frame_size, frame_size, strides,
                            num_classes, confidence_threshold, nms_threshold, nms_option)

                    if det_boxes:
                        box = det_boxes[0]
                        cls_id = int(box[0])
                        score = box[1]
                        x1, y1, x2, y2 = box[2], box[3], box[4], box[5]

                        ball_cx_pixel = (x1 + x2) / 2
                        ball_cy_pixel = (y1 + y2) / 2
                        ball_pos_cm = (ball_cx_pixel - CENTER_X_PIXEL) / PIXEL_PER_CM

                        now_ms = utime.ticks_ms()
                        position_valid = abs(ball_pos_cm) <= VALID_POS_LIMIT_CM
                        last_ball_pos_cm = ball_pos_cm
                        last_position_valid = position_valid
                        last_score = score

                        if position_valid:
                            lost_frames = 0
                            if (CONTROL_ENABLE and
                                utime.ticks_diff(now_ms, control_start_ms) >= CONTROL_STARTUP_HOLD_MS):
                                pid_output_us = pid.update(TARGET_POS_CM, ball_pos_cm, now_ms)
                                servo_target_us = (SERVO_NEUTRAL_US +
                                                   SERVO_DIRECTION * pid_output_us)
                                servo_write_us(servo_target_us, now_ms, use_slew=True)
                            else:
                                pid.reset(ball_pos_cm, now_ms)
                                pid_output_us = 0.0
                                servo_write_us(SERVO_NEUTRAL_US, now_ms, use_slew=True)
                        else:
                            lost_frames += 1
                            if lost_frames >= DETECTION_LOST_FRAMES:
                                pid.reset()
                                pid_output_us = 0.0
                                servo_write_us(SERVO_NEUTRAL_US, now_ms, use_slew=True)

                        disp_x1 = int(x1 * DISPLAY_WIDTH / OUT_RGB888P_WIDTH)
                        disp_y1 = int(y1 * DISPLAY_HEIGHT / OUT_RGB888P_HEIGH)
                        disp_x2 = int(x2 * DISPLAY_WIDTH / OUT_RGB888P_WIDTH)
                        disp_y2 = int(y2 * DISPLAY_HEIGHT / OUT_RGB888P_HEIGH)
                        disp_cx = int(ball_cx_pixel * DISPLAY_WIDTH / OUT_RGB888P_WIDTH)
                        disp_cy = int(ball_cy_pixel * DISPLAY_HEIGHT / OUT_RGB888P_HEIGH)

                        draw_osd(osd_img, ball_pos_cm, ball_cx_pixel, score, position_valid,
                                 disp_x1, disp_y1, disp_x2, disp_y2, disp_cx, disp_cy, cls_id,
                                 pid, servo_current_us)

                        if utime.ticks_diff(now_ms, last_debug_print_ms) >= DEBUG_PRINT_INTERVAL_MS:
                            print("pos:%+.2fcm tgt:%+.2fcm PID:%+.1fus PWM:%dus score:%.2f" %
                                  (ball_pos_cm, TARGET_POS_CM, pid_output_us,
                                   int(servo_current_us), score))
                            last_debug_print_ms = now_ms
                    else:
                        last_position_valid = False
                        last_score = 0.0
                        lost_frames += 1
                        now_ms = utime.ticks_ms()
                        if lost_frames >= DETECTION_LOST_FRAMES:
                            pid.reset()
                            pid_output_us = 0.0
                            servo_write_us(SERVO_NEUTRAL_US, now_ms, use_slew=True)
                        draw_osd(osd_img, 0, 0, 0, False,
                                 0, 0, 0, 0, 0, 0, 0,
                                 pid, servo_current_us)

                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                    gc.collect()

                rgb888p_img = None

    except Exception as e:
        print(f"运行异常: {e}")
        import sys
        sys.print_exception(e)
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        del ai2d_input_tensor
        del ai2d_output_tensor
        servo_deinit()
        uart_deinit()
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        gc.collect()
        time.sleep(1)
        nn.shrink_memory_pool()
    print("检测结束")
    return 0


if __name__ == "__main__":
    detection()
