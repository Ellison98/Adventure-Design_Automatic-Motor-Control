import cv2
import numpy as np
import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# 듀티 사이클 범위
PWM_MIN = 1500
PWM_MAX = 12000

# 수동 모드 입력 범위
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 자동/수동 모드 전환 기준값
SWITCH_THRESHOLD = 1350

# OpenCV 관련 함수
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

def calcaulate_slope_intercept(line):
    x1, y1, x2, y2 = line
    if x2 - x1 == 0:
        return None, None
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    return slope, intercept

def filter_and_average_line(lines, width, height):
    left_lines = []
    right_lines = []

    for line in lines:
        for x1, y1, x2, y2 in line:
            slope, intercept = calcaulate_slope_intercept((x1, y1, x2, y2))
            if slope is None:
                continue
            if 0.5 < abs(slope) < 2:
                if slope < 0:  # 왼쪽 라인
                    left_lines.append((slope, intercept))
                elif slope > 0:  # 오른쪽 라인
                    right_lines.append((slope, intercept))

    def average_lines(lines):
        if not lines:
            return None
        avg_slope = np.mean([line[0] for line in lines])
        avg_intercept = np.mean([line[1] for line in lines])
        return avg_slope, avg_intercept

    left_avg = average_lines(left_lines)
    right_avg = average_lines(right_lines)

    def calculate_line_coordinate(slope, intercept):
        if slope is None or intercept is None:
            return None
        y1 = height
        y2 = int(height * 0.6)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return (x1, y1, x2, y2)

    left_line = calculate_line_coordinate(*left_avg) if left_avg else None
    right_line = calculate_line_coordinate(*right_avg) if right_avg else None
    return left_line, right_line

def calculate_center_line(left_line, right_line):
    if left_line is None or right_line is None:
        return None
    x1_left, y1_left, x2_left, y2_left = left_line
    x1_right, y1_right, x2_right, y2_right = right_line

    x1_center = (x1_left + x1_right) // 2
    y1_center = (y1_left + y1_right) // 2
    x2_center = (x2_left + x2_right) // 2
    y2_center = (y2_left + y2_right) // 2
    return (x1_center, y1_center, x2_center, y2_center)

# PWM 제어 함수
def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

def follow_line_using_opencv(frame):
    height, width = frame.shape[:2]
    gray = grayscale(frame)
    blur = gaussian_blur(gray, 5)
    edges = canny(blur, 50, 150)

    vertices = np.array([[
        (width * 0.1, height),
        (width * 0.45, height * 0.6),
        (width * 0.55, height * 0.6),
        (width * 0.9, height)
    ]], dtype=np.int32)
    roi = region_of_interest(edges, vertices)
    lines = hough_lines(roi, 1, np.pi / 180, 30, 20, 200)

    if lines is not None:
        left_line, right_line = filter_and_average_line(lines, width, height)
        center_line = calculate_center_line(left_line, right_line)
        if center_line is not None:
            x1, y1, x2, y2 = center_line
            steer = (x1 + x2) // 2
            throttle = height - y1
            set_motor_pwm(steer, throttle)

# 메인 실행 함수
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

    seri = serial.Serial('/dev/ttyACM0', 9600, timeout=None)
    seri.reset_input_buffer()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame. Check the camera.")
            break

        frame = cv2.resize(frame, (640, 480))

        try:
            content = seri.readline().decode(errors='ignore').strip()
            values = content.split(',')
            if len(values) >= 3:
                switch_bt_duration = int(values[2].strip())
                mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"
            else:
                mode = "AUTO"

        except Exception as e:
            print(f"Serial read error: {e}")
            mode = "AUTO"

        if mode == "AUTO":
            follow_line_using_opencv(frame)
        else:
            if len(values) >= 2:
                steer_value = int(values[0].strip())
                throttle_value = int(values[1].strip())
                set_motor_pwm(steer_value, throttle_value)

        cv2.imshow("Lane Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    seri.close()

if __name__ == "__main__":
    main()
