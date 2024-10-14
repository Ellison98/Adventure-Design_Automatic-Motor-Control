import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
import sys
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

# 모터 각도 및 속도 초기화
angle = 90       # 서보 모터 초기 각도
offset = -12     # 서보 모터 보정 값
speed = 6020     # 스로틀 초기 속도

# 서보 각도를 듀티 사이클로 변환하는 클래스
class ServoCalc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)  # 최소 듀티 사이클
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)      # 최대 듀티 사이클
        self.duty_range = int(max_duty - self.min_duty)                # 듀티 사이클 범위

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)     # 각도에 따른 듀티 사이클 반환

def load_data():
    # 시리얼에서 데이터를 읽어오는 함수 (예시)
    data = "0:1500 1:1500 2:1500"  # 예시 데이터
    data = data.split(' ')  # 공백으로 분할
    print(f"Split data: {data}")  # 분할된 데이터 출력
    
    # ':'이 포함된 데이터만 처리
    result = {}
    for for_a in data:
        if ':' in for_a:  # ':'이 있는 항목만 처리
            key_value = for_a.split(':')  # 키-값 분할
            if len(key_value) == 2 and key_value[1].isdigit():  # 두 항목이 있고 두 번째 항목이 숫자인지 확인
                result[key_value[0]] = int(key_value[1])  # 딕셔너리에 추가
            else:
                print(f"Invalid data: {key_value}")  # 유효하지 않은 데이터 출력

    return result  # 딕셔너리 반환

def read_controller_data():
    # 조종기에서 데이터를 읽어오는 로직을 구현합니다.
    # 예시 데이터 반환
    return {
        'angle': 90,  # 조종기에서 받은 각도 값
        'speed': 50   # 조종기에서 받은 속도 값
    }

def control_motor(angle, speed):
    # 모터 제어 로직을 구현합니다.
    print(f"모터 각도: {angle}, 속도: {speed}")

# 서보 각도 계산기 초기화
servo_angle = ServoCalc()

# 메인 루프: 조종기 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        # 조종기에서 데이터를 읽어옴
        controller_data = read_controller_data()
        
        # 조종기 데이터에 따라 모터 각도와 속도 제어
        angle = controller_data['angle']
        speed = controller_data['speed']
        
        # 모터 제어 로직
        if angle < 75:
            angle = 45
        elif angle > 105:
            angle = 135
        
        # 속도 제어 로직
        if speed < 20:
            speed = 20
        elif speed > 100:
            speed = 100
        
        # 모터 제어 함수 호출
        control_motor(angle, speed)
        
    except Exception as e:
        print(f"오류 발생: {e}")
        continue  # 오류 발생 시 계속 실행

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 5930  # 스로틀 중지
calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = 6020  # 스로틀 초기화
