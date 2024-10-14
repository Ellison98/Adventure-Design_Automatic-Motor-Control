import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
import sys
import time

# 서보 및 스로틀 핀 설정
servo_pin = 0
throttle_pin = 15

# 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# I2C 통신 및 PCA9685 초기화
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 62.5

# 각도 및 속도 초기화
angle = 90  # 기본 각도 (중앙)
offset = -12  # 서보 각도 보정값
speed = 6020  # 기본 속도
back_speed = speed  # 현재 속도 저장

class servo_calc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)
        self.duty_range = int(max_duty - self.min_duty)

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)

def load_data():
    data = arduino.readline()
    try:
        data = data.decode('ascii').split(' ')
        data = [for_a.split(':') for for_a in data]
        return {for_a[0]: int(for_a[1].replace('\r\n', '')) for for_a in data}
    except:
        return {}

servo_angle = servo_calc()

while True:
    try:
        # 조종기에서 데이터 수신
        data_dict = load_data()

        # 수신된 스로틀 값이 있는지 확인
        if '0' in data_dict:
            throttle_value = data_dict['0']
            print(f"Throttle value: {throttle_value}")  # 스로틀 값 출력
            
            # 스로틀 값에 따라 속도 변경
            if throttle_value > 500:  # 스로틀이 임계값보다 크면 앞으로 가기
                speed = 5950  # 앞으로 가는 속도
            elif throttle_value < 500:  # 스로틀이 임계값보다 작으면 멈추기
                speed = 6020  # 멈추는 속도

            # 속도 설정
            if back_speed != speed:
                back_speed = speed
                pca.channels[throttle_pin].duty_cycle = speed
        else:
            print("Throttle value not received.")

        # 서보 각도 설정 (수동 설정)
        calc_angle = angle + offset
        calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

        print(f"Angle: {calc_angle}, Speed: {speed}")
        pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)

    except Exception as e:
        print(f"Error: {e}")

# 종료 시 서보와 모터 초기화
pca.channels[throttle_pin].duty_cycle = 5930
calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[servo_pin].duty_cycle = servo_angle(90)

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = 6020
