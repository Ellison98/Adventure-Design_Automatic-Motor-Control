import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
import time

# 서보 모터 핀 및 스로틀 핀 설정
servo_pin = 0          # PCA9685의 채널 0번: 서보 모터 제어
throttle_pin = 15       # PCA9685의 채널 15번: 스로틀 제어

# 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 50  # 주파수 설정 (50Hz)

# 서보 모터 각도 및 속도 초기화
angle_offset = -12     # 서보 모터 보정 값
base_speed = 6020     # 기본 속도
stop_speed = 5930     # 스로틀 중지 속도

# 서보 각도를 듀티 사이클로 변환하는 클래스
class ServoCalc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)  # 최소 듀티 사이클
        self.max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)  # 최대 듀티 사이클
        self.duty_range = int(self.max_duty - self.min_duty)            # 듀티 사이클 범위

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)     # 각도에 따른 듀티 사이클 반환

# 서보 각도 계산기 초기화
servo_angle = ServoCalc()

# 메인 루프: 조종기 입력에 따라 서보 및 스로틀 제어
while True:
    try:
        data = arduino.readline()  # 시리얼에서 한 줄 읽기
        print(f"Raw data: {data}")  # 수신된 원본 데이터 출력
        try:
            data = data.decode('utf-8').strip()  # UTF-8로 디코딩
        except UnicodeDecodeError:
            print(f"Failed to decode data: {data}")  # 디코딩 실패 시 원본 바이트 출력
            continue

        if not data:  # 데이터가 비어있으면
            continue

        data_dict = {}
        data = data.split(' ')  # 공백으로 분할
        for item in data:
            if ':' in item:  # ':'이 있는 항목만 처리
                key_value = item.split(':')
                if len(key_value) == 2 and key_value[1].isdigit():
                    data_dict[key_value[0]] = int(key_value[1])  # 딕셔너리에 추가

        # 조종기 입력 처리
        if '0' in data_dict and '1' in data_dict:
            throttle_value = data_dict['0']
            angle_value = data_dict['1']

            # 스로틀 속도 및 서보 각도 설정
            if 900 <= throttle_value <= 1700:  # 스로틀 값이 유효한 범위일 때
                speed = int((throttle_value - 900) / 800 * (base_speed - stop_speed) + stop_speed)  # 속도 변환
                pca.channels[throttle_pin].duty_cycle = speed  # 스로틀 속도 적용
            else:
                pca.channels[throttle_pin].duty_cycle = stop_speed  # 스로틀 중지

            # 서보 각도 변환
            if 900 <= angle_value <= 1700:  # 각도 값이 유효한 범위일 때
                angle = int((angle_value - 900) / 800 * 180)  # 0~180으로 변환
                calc_angle = angle + angle_offset
                pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)  # 서보 각도 적용
            else:
                pca.channels[servo_pin].duty_cycle = servo_angle(90 + angle_offset)  # 서보 중앙 위치로 초기화

        print(f"Throttle Value: {throttle_value}, Speed: {speed}, Servo Angle: {calc_angle}")

    except Exception as e:
        print(e)

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = stop_speed  # 스로틀 중지
pca.channels[servo_pin].duty_cycle = servo_angle(90 + angle_offset)  # 서보 모터 중앙 위치로 초기화

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = base_speed  # 스로틀 초기화
