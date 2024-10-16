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
STEER_MIN = 900
STEER_MAX = 1900
THROTTLE_MIN = 900
THROTTLE_MAX = 1900

# 채널 설정
STEERING_CHANNEL = 7   # 서보모터 채널 (0~15)
THROTTLE_CHANNEL = 8   # DC 모터 또는 서보모터 채널 (0~15)

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑하는 함수"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def percent_to_pwm(steer_percent, throttle_percent):
    """스티어링 및 스로틀 % 값을 PWM 값으로 변환하는 함수"""
    
    # 스티어링 % 값에 대한 PWM 값
    if steer_percent == 8.3:
        steer_pwm = 2200
    elif steer_percent == 9.5:
        steer_pwm = 3000
    elif steer_percent == 11.9:
        steer_pwm = 3932
    else:
        steer_pwm = None  # 잘못된 % 값 처리
    
    # 스로틀 % 값에 대한 PWM 값
    if throttle_percent == 6:
        throttle_pwm = 240
    elif throttle_percent == 9.5:
        throttle_pwm = 320
    elif throttle_percent == 13.1:
        throttle_pwm = 380
    else:
        throttle_pwm = None  # 잘못된 % 값 처리

    return steer_pwm, throttle_pwm

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수"""
    steer_pwm, throttle_pwm = percent_to_pwm(steer_value, throttle_value)

    if steer_pwm is not None and throttle_pwm is not None:
        pca.channels[STEERING_CHANNEL].duty_cycle = steer_pwm
        pca.channels[THROTTLE_CHANNEL].duty_cycle = throttle_pwm
        print(f"스티어링 PWM 값: {steer_pwm}, 쓰로틀 PWM 값: {throttle_pwm}")
    else:
        print("잘못된 % 값으로 PWM 값을 찾을 수 없습니다.")

def running():
    # 시리얼 포트 설정 (아두이노에서 데이터 수신)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        seri.reset_input_buffer()  # 시리얼 입력 버퍼 초기화
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
