import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
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

# 서보 모터 각도 계산 클래스
class servo_calc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)
        self.duty_range = int(max_duty - self.min_duty)

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)

# 데이터 수신 함수
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

        # 스로틀 값과 서보 모터 값 확인
        if '1' in data_dict and '2' in data_dict:
            throttle_value = data_dict['1']  # 스로틀 값 (1번 값)
            servo_input = data_dict['2']  # 서보 모터 입력 값 (2번 값, 100~200)

            print(f"Throttle value: {throttle_value}, Servo input: {servo_input}")

            # 스로틀 값에 따른 속도 세밀 제어 (900 ~ 1700)
            if 900 <= throttle_value <= 1700:
                speed = 6020 - int((throttle_value - 900) * 5)  # 속도 조정
            else:
                speed = 6020  # 멈추는 속도

            # 서보 모터 값이 100~200 사이일 때 각도 변환 (0도 ~ 180도)
            if 100 <= servo_input <= 200:
                angle = (servo_input - 100) * 1.8  # 100~200을 0도~180도로 변환

            # 보정값 자동 설정 (예: 특정 각도 범위에서 자동 조정)
            if angle < 60 or angle > 120:
                offset = -8  # 각도가 많이 틀어졌을 때 보정
            else:
                offset = -12  # 기본 보정값

            # 속도 및 각도 적용
            if back_speed != speed:
                back_speed = speed
                pca.channels[throttle_pin].duty_cycle = speed

            calc_angle = angle + offset
            calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
            pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)

            print(f"Angle: {calc_angle}, Speed: {speed}")
        else:
            print("Throttle or servo input not received.")

    except Exception as e:
        print(f"Error: {e}")

# 종료 시 서보와 모터 초기화
pca.channels[throttle_pin].duty_cycle = 5930
calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[servo_pin].duty_cycle = servo_angle(90)

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = 6020
