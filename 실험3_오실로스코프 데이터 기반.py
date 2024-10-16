import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# PCA9685 주파수 설정
pca.frequency = 60

# 아두이노에서 받은 PWM 범위에 따른 듀티 사이클 설정
STEERING_CHANNEL = 7   # 서보모터 채널 (0~15)
THROTTLE_CHANNEL = 8   # DC 모터 또는 서보모터 채널 (0~15)

# 듀티 사이클 범위
STEER_MIN_PWM = 220  # 서보 모터의 최소 값
STEER_MAX_PWM = 420  # 서보 모터의 최대 값

THROTTLE_MIN_PWM = 240  # 쓰로틀 모터의 최소 값
THROTTLE_MAX_PWM = 380  # 쓰로틀 모터의 최대 값

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑하는 함수"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수"""
    # % 값에 대한 최대, 최소를 지정
    steer_min_percent = 8.3
    steer_max_percent = 11.9
    throttle_min_percent = 6
    throttle_max_percent = 13.1

    # % 값을 PWM 값으로 변환
    steer_pwm = map_value(steer_value, steer_min_percent, steer_max_percent, STEER_MIN_PWM, STEER_MAX_PWM)
    throttle_pwm = map_value(throttle_value, throttle_min_percent, throttle_max_percent, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM)

    # PWM 값 설정
    pca.channels[STEERING_CHANNEL].duty_cycle = steer_pwm
    pca.channels[THROTTLE_CHANNEL].duty_cycle = throttle_pwm

    print(f"스티어링 PWM 값: {steer_pwm}, 쓰로틀 PWM 값: {throttle_pwm}")

def running():
    # 시리얼 포트 설정 (아두이노에서 데이터 수신)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        while True:
            try:
                # 아두이노로부터 데이터 읽기
                content = seri.readline().decode(errors='ignore').strip()
                values = content.split(',')

                # 값이 두 개 미만이면 처리하지 않음
                if len(values) < 2:
                    print("잘못된 데이터 형식")
                    continue

                # 아두이노로부터 받은 값을 정수로 변환
                steer_duration = float(values[0].strip())  # 스티어링 % 값
                throttle_duration = float(values[1].strip())  # 쓰로틀 % 값

                # 모터의 PWM 값 설정
                set_motor_pwm(steer_duration, throttle_duration)

            except ValueError:
                print("잘못된 신호")
            except Exception as e:
                print(f"예외 발생: {e}")

if __name__ == "__main__":
    running()
