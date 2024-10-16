import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# I2C 및 PCA9685 설정
try:
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
except Exception as e:
    print(f"I2C 초기화 오류: {e}")
    exit()

# PCA9685 주파수 설정
pca.frequency = 60

# 아두이노에서 받은 PWM 범위에 따른 듀티 사이클 설정
STEERING_CHANNEL = 7
THROTTLE_CHANNEL = 8

# 듀티 사이클 범위
STEER_MIN_PWM = 220
STEER_MAX_PWM = 420

THROTTLE_MIN_PWM = 240
THROTTLE_MAX_PWM = 380

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    steer_pwm = map_value(steer_value, 1000, 2000, STEER_MIN_PWM, STEER_MAX_PWM)
    throttle_pwm = map_value(throttle_value, 1000, 2000, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM)

    # PWM 값 범위 확인
    steer_pwm = max(min(steer_pwm, 65535), 0)
    throttle_pwm = max(min(throttle_pwm, 65535), 0)

    pca.channels[STEERING_CHANNEL].duty_cycle = steer_pwm
    pca.channels[THROTTLE_CHANNEL].duty_cycle = throttle_pwm

    print(f"스티어링 PWM 값: {steer_pwm}, 쓰로틀 PWM 값: {throttle_pwm}")

def running():
    try:
        with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
            while True:
                content = seri.readline().decode(errors='ignore').strip()
                values = content.split(',')

                if len(values) < 2:
                    print("잘못된 데이터 형식")
                    continue

                try:
                    steer_duration = int(values[0].strip())
                    throttle_duration = int(values[1].strip())
                    set_motor_pwm(steer_duration, throttle_duration)
                except ValueError:
                    print("받은 값이 정수가 아닙니다.")
                    continue

    except serial.SerialException as e:
        print(f"시리얼 포트 오류: {e}")
    except Exception as e:
        print(f"예외 발생: {e}")

if __name__ == "__main__":
    running()
