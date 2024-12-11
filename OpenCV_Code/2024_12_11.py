import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import cv2
import numpy as np

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# PWM 범위
PWM_MIN = 1500
PWM_MAX = 12000

# 스티어링 및 쓰로틀 값 범위
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 모드 전환 기준값
SWITCH_THRESHOLD = 1350

# 스티어링 변화 제한
PREV_STEER = 1390  # 초기값
MAX_STEERING_CHANGE = 100  # 최대 조향 변화

# GStreamer 파이프라인
gst_pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
)

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    """스티어링 및 쓰로틀 값 설정"""
    global PREV_STEER
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
    
    # 스티어링 변화 제한 적용
    steer_pwm = max(PREV_STEER - MAX_STEERING_CHANGE, min(PREV_STEER + MAX_STEERING_CHANGE, steer_pwm))
    PREV_STEER = steer_pwm

    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

def preprocess_frame(frame):
    """프레임 전처리"""
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    edges = cv2.Canny(blur_img, 50, 150)
    return edges

def detect_lane(frame):
    """차선 검출"""
    height, width = frame.shape[:2]
    vertices = np.array([[
        (0, height),
        (width * 0.1, height * 0.6),
        (width * 0.9, height * 0.6),
        (width, height)
    ]], dtype=np.int32)
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, vertices, 255)
    ROI_img = cv2.bitwise_and(frame, mask)
    lines = cv2.HoughLinesP(ROI_img, 1, np.pi / 180, threshold=30, minLineLength=20, maxLineGap=10)

    center_x = None
    if lines is not None:
        left_lines, right_lines = [], []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else 0
                if 0.5 < abs(slope) < 2:
                    (left_lines if slope < 0 else right_lines).append((x1, y1, x2, y2))

        if left_lines and right_lines:
            left_avg = np.mean(left_lines, axis=0).astype(int)
            right_avg = np.mean(right_lines, axis=0).astype(int)
            left_center = (left_avg[0] + left_avg[2]) // 2
            right_center = (right_avg[0] + right_avg[2]) // 2
            center_x = (left_center + right_center) // 2

    return center_x

def control_automatic(frame):
    """자동 모드에서 차량 제어"""
    height, width = frame.shape[:2]
    edges = preprocess_frame(frame)
    center_x = detect_lane(edges)

    if center_x is not None:
        if abs(center_x - width // 2) < 20:
            set_motor_pwm(1400, 1300)  # 직진
        elif center_x < width // 2:
            set_motor_pwm(1500, 1300)  # 좌회전
        else:
            set_motor_pwm(1300, 1300)  # 우회전
    else:
        set_motor_pwm(1400, 1200)  # 정지

    return edges

def running():
    """메인 실행 함수"""
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        seri.reset_input_buffer()
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("GStreamer 카메라를 열 수 없습니다.")
            return

        while True:
            try:
                content = seri.readline().decode(errors='ignore').strip()
                values = content.split(',')

                if len(values) < 3:
                    print(f"잘못된 데이터: {content}")
                    continue

                steer_duration = int(values[0].strip())
                throttle_duration = int(values[1].strip())
                switch_bt_duration = int(values[2].strip())

                mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"
                print(f"모드: {mode}")

                if mode == "MANUAL":
                    set_motor_pwm(steer_duration, throttle_duration)
                else:
                    ret, frame = cap.read()
                    if not ret:
                        print("프레임 읽기 실패")
                        break
                    frame = cv2.resize(frame, (640, 480))
                    edges = control_automatic(frame)
                    cv2.imshow("Lane Detection", edges)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print(f"에러 발생: {e}")

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    running()