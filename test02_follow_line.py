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

# 스티어링과 쓰로틀의 입력 범위 (예: 아두이노에서 오는 값)
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 자동/수동 모드 기준값
SWITCH_THRESHOLD = 1350

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑하는 함수"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수"""
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)

    # PWM 값 출력 (디버깅용)
    print(f"입력 스티어링: {steer_value}, 입력 쓰로틀: {throttle_value}")
    print(f"변환된 스티어링 PWM: {steer_pwm}, 변환된 쓰로틀 PWM: {throttle_pwm}")

    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

def follow_line(left_sensor, center_sensor, right_sensor):
    """센서 값을 기반으로 라인을 따라가는 함수"""
    if center_sensor == 0:
        print("라인을 따라가고 있습니다.")
        # 중앙 센서가 라인을 인식하면 그대로 직진
        pca.channels[0].duty_cycle = PWM_MIN  # 직진 설정 (예: 최소 PWM 값으로 설정)
    elif left_sensor == 0:
        print("왼쪽으로 회전")
        # 왼쪽 센서가 라인을 인식하면 왼쪽으로 회전
        pca.channels[0].duty_cycle = map_value(1200, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    elif right_sensor == 0:
        print("오른쪽으로 회전")
        # 오른쪽 센서가 라인을 인식하면 오른쪽으로 회전
        pca.channels[0].duty_cycle = map_value(1800, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    else:
        print("라인을 벗어났습니다.")
        # 모든 센서가 라인을 감지하지 못하면 중지 또는 라인 복귀 동작
        pca.channels[1].duty_cycle = 0  # 쓰로틀 PWM을 0으로 설정하여 멈춤

def running():
    # 시리얼 포트 설정 (아두이노에서 데이터 수신)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        # 시리얼 버퍼 비우기
        seri.reset_input_buffer()
        
        while True:
            try:
                # 아두이노로부터 데이터 읽기
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

                    # 모드 구분
                    mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"
                    print(f"모드: {mode} (스위치 값: {switch_bt_duration})")

                    if mode == "MANUAL":
                        if STEER_MIN <= steer_duration <= STEER_MAX and THROTTLE_MIN <= throttle_duration <= THROTTLE_MAX:
                            set_motor_pwm(steer_duration, throttle_duration)
                        else:
                            print(f"비정상적인 값 범위: {steer_duration}, {throttle_duration}")
                    else:
                        follow_line(left_sensor, center_sensor, right_sensor)

                else:
                    print(f"숫자가 아닌 값 수신됨: {content}")

            except ValueError:
                print(f"잘못된 신호: {content}")
            except Exception as e:
                print(f"예외 발생: {e}")

if __name__ == "__main__":
    running()
