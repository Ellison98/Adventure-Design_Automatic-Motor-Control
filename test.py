import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
import sys
import time

# 서보 모터 핀 및 스로틀 핀 설정
servo_pin = 0          # PCA9685의 채널 0번: 서보 모터 제어
throttle_pin = 15      # PCA9685의 채널 15번: 스로틀 제어

# 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 62.5  # 주파수 설정 (62.5Hz)

# 모터 각도 및 속도 초기화
angle = 90       # 서보 모터 초기 각도
offset = -12     # 서보 모터 보정 값
speed = 6020     # 스로틀 초기 속도

# 서보 각도를 듀티 사이클로 변환하는 클래스
class servo_calc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)  # 최소 듀티 사이클
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)      # 최대 듀티 사이클
        self.duty_range = int(max_duty - self.min_duty)                # 듀티 사이클 범위

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)     # 각도에 따른 듀티 사이클 반환

# 시리얼 통신으로 데이터 읽기 함수
def load_data():
    data = arduino.readline()  # 시리얼에서 한 줄 읽기
    print(f"Raw data: {data}")  # 수신된 원본 데이터 출력
    try:
        data = data.decode('utf-8').strip()  # UTF-8로 디코딩
    except UnicodeDecodeError:
        print(f"Failed to decode data: {data}")  # 디코딩 실패 시 원본 바이트 출력
        return {}

    if not data:  # 데이터가 비어있으면
        return {}

    # 조종기 데이터의 예시 형식: "0:1500 1:1600"
    data = data.split(' ')  # 공백으로 분할
    result = {}
    for for_a in data:
        if ':' in for_a:  # ':'이 있는 항목만 처리
            key_value = for_a.split(':')  # 키-값 분할
            if len(key_value) == 2 and key_value[1].isdigit():  # 두 항목이 있고 두 번째 항목이 숫자인지 확인
                result[key_value[0]] = int(key_value[1])  # 딕셔너리에 추가
    return result  # 딕셔너리 반환

# 서보 각도 계산기 초기화
servo_angle = servo_calc()

# 메인 루프: 조종기 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        data_dict = load_data()  # 시리얼에서 데이터를 읽어옴
        print(f"Received data: {data_dict}")  # 수신된 데이터 출력
        
        if '0' in data_dict:  # 조종기 데이터가 존재하는 경우
            throttle_value = data_dict['0']  # 조종기에서 받은 스로틀 값
            if 900 <= throttle_value <= 1700:
                speed = int((throttle_value - 900) / 800 * (6020 - 5930) + 5930)  # 900~1700을 5930~6020으로 변환
                pca.channels[throttle_pin].duty_cycle = speed  # 스로틀 제어
            else:
                print("Throttle value out of range.")

            if '1' in data_dict:  # 서보 모터 각도를 제어하기 위해 추가적인 채널을 사용할 수 있음
                servo_value = data_dict['1']  # 조종기에서 받은 서보 값
                if 900 <= servo_value <= 1700:
                    angle = int((servo_value - 900) / 800 * (180 - 0) + 0)  # 900~1700을 0~180으로 변환
                    calc_angle = angle + offset
                    calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
                    pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)  # 서보 모터 제어

        time.sleep(0.1)  # 약간의 지연

    except Exception as e:
        print(f"오류 발생: {e}")

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 5930  # 스로틀 중지
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화
