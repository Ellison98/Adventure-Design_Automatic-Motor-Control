import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import cv2
import numpy as np
import math

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# 듀티 사이클 범위 (0~65535: 16비트 범위)
PWM_MIN = 1500
PWM_MAX = 12000

# 스티어링과 쓰로틀의 입력 범위 (예: 아두이노에서 오는 값)
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 자동/수동 모드 기준값
SWITCH_THRESHOLD = 1350

# 조향 변화 제한
PREV_STEER = 1390  # 초기값 (중립 상태)
MAX_STEERING_CHANGE = 100  # 허용하는 최대 조향 변화

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑하는 함수"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수"""
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)

    # PWM 값 설정
    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

def grayscale(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def gaussian_blur(frame, kernel_size):
    return cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)

def canny(frame, low_threshold, high_threshold):
    return cv2.Canny(frame, low_threshold, high_threshold)

def region_of_interest(frame, vertices, color3=(255, 255, 255), color1=255):
    mask = np.zeros_like(frame)
    color = color3 if len(frame.shape) > 2 else color1
    cv2.fillPoly(mask, vertices, color)
    return cv2.bitwise_and(frame, mask)

def filter_and_average_line(lines, width, height):
    left_lines = []
    right_lines = []

    for line in lines:
        for x1, y1, x2, y2 in line:
            slope, intercept = calculate_slope_intercept((x1, y1, x2, y2))
            if slope is None:
                continue
            if 0.5 < abs(slope) < 2:
                if slope < 0:
                    left_lines.append((slope, intercept))
                elif slope > 0:
                    right_lines.append((slope, intercept))

    def average_lines(lines):
        if not lines:
            return None
        slope = np.mean([line[0] for line in lines])
        intercept = np.mean([line[1] for line in lines])
        return slope, intercept

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

def calculate_slope_intercept(line):
    x1, y1, x2, y2 = line
    if x2 - x1 == 0:
        return None, None
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1
    return slope, intercept

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

def follow_line(center_x, frame_width, lt, rt):
    """OpenCV에서 검출된 중심선을 기반으로 라인을 따라 움직이는 함수"""
    if center_x is not None:
        # 중심선이 화면 중앙에 가까우면 직진
        if abs(center_x - frame_width // 2) < 20:
            print("직진 중...")
            pca.channels[1].duty_cycle = map_value(1400, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
            pca.channels[0].duty_cycle = map_value(1390, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
            lt = False
            rt = True
        # 중심선이 왼쪽에 치우쳐 있으면 왼쪽으로 회전
        elif center_x < frame_width // 2:
            print("왼쪽으로 조금 회전")
            pca.channels[1].duty_cycle = map_value(1400, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
            pca.channels[0].duty_cycle = map_value(1440, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
            lt = True
            rt = False
        # 중심선이 오른쪽에 치우쳐 있으면 오른쪽으로 회전
        else:
            print("오른쪽으로 조금 회전")
            pca.channels[1].duty_cycle = map_value(1400, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
            pca.channels[0].duty_cycle = map_value(1520, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
            lt = False
            rt = True
    else:
        # 중심선을 찾지 못했을 때
        print("라인을 찾지 못했습니다.")
        if lt:
            print("왼쪽으로 유지")
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
            pca.channels[0].duty_cycle = map_value(1240, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
        elif rt:
            print("오른쪽으로 유지")
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
            pca.channels[0].duty_cycle = map_value(1520, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
        else:
            print("정지 중...")
            pca.channels[1].duty_cycle = map_value(1265, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
            pca.channels[0].duty_cycle = map_value(1390, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)

    return lt, rt

def follow_line_using_opencv(frame, lt, rt):
    """OpenCV를 이용해 라인을 따라 움직이는 함수"""
    height, width = frame.shape[:2]

    # OpenCV 라인 검출
    gray_img = grayscale(frame)
    blur_img = gaussian_blur(gray_img, 3)
    canny_img = canny(blur_img, 50, 150)
    vertices = np.array([[
        (0, height),
        (width * 0.1, height * 0.6),
        (width * 0.9, height * 0.6),
        (width, height)
    ]], dtype=np.int32)
    ROI_img = region_of_interest(canny_img, vertices)
    lines = cv2.HoughLinesP(ROI_img, 1, np.pi / 180, threshold=30, minLineLength=20, maxLineGap=2)

    center_x = None
    if lines is not None:
        left_line, right_line = filter_and_average_line(lines, width, height)
        center_line = calculate_center_line(left_line, right_line)
        if center_line is not None:
            x1, y1, x2, y2 = center_line
            center_x = (x1 + x2) // 2

    lt, rt = follow_line(center_x, width, lt, rt)

    # 시각화
    if center_x is not None:
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)  # 파란선
        cv2.putText(frame, f"Center X: {center_x}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return lt, rt

def running():
    # 시리얼 포트 설정
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        seri.reset_input_buffer()

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("카메라를 열 수 없습니다.")
            return

        lt, rt = False, False
        while True:
            try:
                content = seri.readline().decode(errors='ignore').strip()
                values = content.split(',')

                if len(values) < 6:
                    print(f"잘못된 데이터 형식: {content}")
                    continue

                if all(value.isdigit() for value in values[:6]):
                    steer_duration = int(values[0].strip())
                    throttle_duration = int(values[1].strip())
                    switch_bt_duration = int(values[2].strip())
                    left_sensor = int(values[3].strip())
                    center_sensor = int(values[4].strip())
                    right_sensor = int(values[5].strip())

                    mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"
                    print(f"모드: {mode}")

                    if mode == "MANUAL":
                        set_motor_pwm(steer_duration, throttle_duration)
                    else:
                        ret, frame = cap.read()
                        if not ret:
                            print("프레임을 읽을 수 없습니다.")
                            break

                        frame = cv2.resize(frame, (640, 480))
                        lt, rt = follow_line_using_opencv(frame, lt, rt)
                        cv2.imshow("Lane Detection", frame)

                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

            except Exception as e:
                print(f"예외 발생: {e}")

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    running()
