import cv2
import numpy as np
import serial
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# 듀티 사이클 범위 (0~65535: 16비트 범위)
PWM_MIN = 1500
PWM_MAX = 12000

# 스티어링과 쓰로틀의 입력 범위
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 자동/수동 모드 기준값
SWITCH_THRESHOLD = 1350

# 값 매핑 함수
def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# PWM 설정 함수
def set_motor_pwm(steer_value, throttle_value):
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

# OpenCV 함수들
def grayscale(frame):
    return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

def gaussian_blur(frame, kernel_size):
    return cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)

def canny(frame, low_threshold, high_threshold):
    return cv2.Canny(frame, low_threshold, high_threshold)

def region_of_interest(frame, vertices):
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(frame, mask)

def hough_lines(frame, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(frame, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    return lines

def calculate_center_line(lines):
    if lines is None:
        return None
    left_lines, right_lines = [], []
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else None
            if slope is None or abs(slope) < 0.5:
                continue
            if slope < 0:
                left_lines.append(line)
            else:
                right_lines.append(line)
    return (np.mean(left_lines, axis=0) if left_lines else None, 
            np.mean(right_lines, axis=0) if right_lines else None)

def draw_lines(frame, lines):
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = map(int, line)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

def follow_line(frame):
    gray = grayscale(frame)
    blur = gaussian_blur(gray, 5)
    edges = canny(blur, 50, 150)

    height, width = edges.shape
    roi_vertices = np.array([[
        (0, height),
        (width / 2 - 50, height / 2 + 50),
        (width / 2 + 50, height / 2 + 50),
        (width, height)
    ]], dtype=np.int32)

    roi = region_of_interest(edges, roi_vertices)
    lines = hough_lines(roi, 1, np.pi / 180, 20, 15, 10)
    center_lines = calculate_center_line(lines)
    draw_lines(frame, center_lines)
    return frame

# 메인 루프
def main():
    gst_pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to read frame. Check the camera.")
        return

    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        seri.reset_input_buffer()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame. Check the camera.")
                break

            try:
                # 아두이노 데이터 수신
                content = seri.readline().decode(errors='ignore').strip()
                values = content.split(',')

                if len(values) < 3:
                    print("Invalid data format:", content)
                    continue

                steer_duration, throttle_duration, switch_bt_duration = map(int, values[:3])
                mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"
                print(f"Mode: {mode}")

                if mode == "MANUAL":
                    set_motor_pwm(steer_duration, throttle_duration)
                else:
                    processed_frame = follow_line(frame)
                    cv2.imshow("Lane Tracking", processed_frame)

            except Exception as e:
                print("Error:", e)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
