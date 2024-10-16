import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# PCA9685 주파수 설정
pca.frequency = 60

# 듀티 사이클 범위 (0~65535: 16비트 범위)
PWM_MIN = 0
PWM_MAX = 65535

# 스티어링과 쓰로틀의 입력 범위 (예: 아두이노에서 오는 값)
STEER_MIN = 1000
STEER_MAX = 2000
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000

# 채널 설정
STEERING_CHANNEL = 0   # 서보모터 채널 (0~15)
THROTTLE_CHANNEL = 1   # DC 모터 또는 서보모터 채널 (0~15)

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑하는 함수"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수"""
    # 스티어링 값을 1000~2000 범위에서 0~65535 범위로 변환
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)

    # 쓰로틀 값을 1000~2000 범위에서 0~65535 범위로 변환
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)

    # 변환된 PWM 값을 PCA9685에 설정
    pca.channels[STEERING_CHANNEL].duty_cycle = steer_pwm
    pca.channels[THROTTLE_CHANNEL].duty_cycle = throttle_pwm

    # 변환된 값 출력
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
                steer_duration = int(values[0].strip())  # 스티어링 값 (1000~2000 범위)
                throttle_duration = int(values[1].strip())  # 쓰로틀 값 (1000~2000 범위)

                # 모터의 PWM 값 설정
                set_motor_pwm(steer_duration, throttle_duration)

            except ValueError:
                print("잘못된 신호")
            except Exception as e:
                print(f"예외 발생: {e}")

if __name__ == "__main__":
    running()
