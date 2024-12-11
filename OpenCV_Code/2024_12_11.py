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

SWITCH_THRESHOLD = 1350  # 자동/수동 전환 기준값

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

# 메인 실행 함수
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    seri = serial.Serial('/dev/ttyACM0', 9600, timeout=None)
    seri.reset_input_buffer()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라 프레임을 읽을 수 없습니다.")
            break

        frame = cv2.resize(frame, (640, 480))
        height, width = frame.shape[:2]

        gray = grayscale(frame)
        blur = gaussian_blur(gray, 5)
        edges = canny(blur, 50, 150)

        vertices = np.array([[
            (0, height),
            (width * 0.1, height * 0.6),
            (width * 0.9, height * 0.6),
            (width, height)
        ]], dtype=np.int32)
        roi = region_of_interest(edges, vertices)
        lines = hough_lines(roi, 1, np.pi / 180, 30, 20, 200)

        mode = "AUTO"
        try:
            content = seri.readline().decode(errors='ignore').strip()
            values = content.split(',')
            if len(values) >= 3:
                switch_bt_duration = int(values[2].strip())
                mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"

        except Exception as e:
            print(f"시리얼 데이터 읽기 오류: {e}")

        if mode == "AUTO":
            if lines is not None:
                # 라인 처리 및 중앙 라인 계산
                left_line, right_line = filter_and_average_line(lines, width, height)
                center_line = calculate_center_line(left_line, right_line)
                if center_line is not None:
                    x1, y1, x2, y2 = center_line
                    steer = (x1 + x2) // 2
                    throttle = height - y1
                    set_motor_pwm(steer, throttle)

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
