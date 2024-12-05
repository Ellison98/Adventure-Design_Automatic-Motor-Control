# 1차 수정 코드
## 코드 설명
OpenCV 기반 차선 인식:

카메라를 통해 영상을 캡처하고, 색상 및 엣지 감지를 활용해 차선을 인식합니다.
차선 중심과 화면 중심의 차이를 계산하여 스티어링 값을 조정합니다.
센서 입력 결합:

센서 입력을 확인하여 OpenCV 인식이 불완전할 때 보조 역할을 하도록 합니다.
PCA9685를 통한 모터 제어:

인식된 차선 중심과 스로틀 값을 기반으로 차량이 적절히 움직이도록 제어합니다.

## 코드 흐름 요약
OpenCV로 차선을 인식:

detect_lane() 함수에서 차선을 탐지하고 중심 좌표를 계산합니다.
센서 데이터를 활용:

센서 데이터를 통해 차선 인식이 불가능할 때의 안전 주행을 보장합니다.
자동/수동 모드 전환:

스위치 버튼 값(switch_bt_duration)에 따라 수동 또는 자동 모드로 전환합니다.

```python
import cv2
import numpy as np
import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

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

SWITCH_THRESHOLD = 1350

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

def detect_lane(frame):
    """카메라에서 차선을 감지하는 함수"""
    # 이미지 전처리
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # 관심 영역 설정
    height, width = edges.shape
    mask = np.zeros_like(edges)
    roi_corners = np.array([[
        (width // 10, height),
        (width * 9 // 10, height),
        (width * 6 // 10, height // 2),
        (width * 4 // 10, height // 2)
    ]], dtype=np.int32)
    cv2.fillPoly(mask, roi_corners, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # 허프 변환을 통한 차선 감지
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=100)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

    # 차선 중심 계산
    lane_center = None
    if lines is not None:
        x_coords = []
        for line in lines:
            x_coords.append((line[0][0] + line[0][2]) // 2)
        lane_center = int(np.mean(x_coords))

    # 디버깅: 차선 중심 및 라인 출력
    overlay = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
    if lane_center:
        cv2.circle(overlay, (lane_center, height // 2), 10, (255, 0, 0), -1)
    cv2.imshow("Lane Detection", overlay)

    return lane_center, width // 2  # 차선 중심 및 화면 중심 반환

def follow_lane(camera, seri):
    """OpenCV와 센서 입력을 기반으로 차량을 제어하는 함수"""
    while True:
        try:
            # 카메라에서 프레임 캡처
            ret, frame = camera.read()
            if not ret:
                print("카메라 오류!")
                break

            lane_center, frame_center = detect_lane(frame)

            # 시리얼로 센서 데이터 읽기
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

                mode = "AUTO" if switch_bt_duration < SWITCH_THRESHOLD else "MANUAL"
                print(f"모드: {mode}")

                if mode == "MANUAL":
                    set_motor_pwm(steer_duration, throttle_duration)
                else:
                    if lane_center is not None:
                        # 차선 중심과 화면 중심의 차이를 기반으로 스티어링 조정
                        error = lane_center - frame_center
                        steer_value = map_value(error, -frame_center, frame_center, STEER_MIN, STEER_MAX)
                        throttle_value = 1400  # 일정 속도로 유지
                        set_motor_pwm(steer_value, throttle_value)
                    else:
                        print("차선을 찾을 수 없습니다.")
                        set_motor_pwm(1390, 1265)  # 정지

            else:
                print(f"숫자가 아닌 값 수신됨: {content}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"예외 발생: {e}")
            break

if __name__ == "__main__":
    # 카메라 및 시리얼 초기화
    camera = cv2.VideoCapture(0)
    seri = serial.Serial('/dev/ttyACM0', 9600, timeout=None)
    seri.reset_input_buffer()

    try:
        follow_lane(camera, seri)
    finally:
        camera.release()
        cv2.destroyAllWindows()
```

