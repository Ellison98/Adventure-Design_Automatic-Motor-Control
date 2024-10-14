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
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

data_dict = {}

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 50  # 서보 모터를 위해 50Hz로 설정

# 모터 각도 및 속도 초기화
angle = 90       # 서보 모터 초기 각도
offset = -12     # 서보 모터 보정 값
speed = 6020     # 스로틀 초기 속도
d_speed = 5950   # 기본 속도
c_speed = 5950   # 센터 속도
s_speed = 5950   # 서보 속도
back_speed = speed
st = 0           # 상태 플래그
last = {}        # 마지막 데이터 저장용

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
    try:
        data = arduino.readline()  # 시리얼에서 한 줄 읽기
        data = data.decode('ascii').strip()  # 아스키 형식으로 디코딩하고 양쪽 공백 제거
        data = data.split(' ')  # 공백으로 분할
        data_dict = {}
        for for_a in data:
            if ':' in for_a:  # ':'가 포함된 경우만 처리
                key, value = for_a.split(':')
                data_dict[key] = int(value.replace('\r\n', ''))  # 딕셔너리로 반환
        return data_dict
    except Exception as e:
        print(f"데이터 읽기 오류: {e}")
        return {}  # 오류 발생 시 빈 딕셔너리 반환

# 서보 각도 계산기 초기화
servo_angle = servo_calc()

# 메인 루프: 센서 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        data_dict = load_data()  # 시리얼에서 데이터를 읽어옴

        # 데이터 길이 체크
        if len(data_dict) < 3:
            print("데이터가 충분하지 않습니다:", data_dict)
            continue  # 데이터가 부족할 경우 다음 루프 실행

        # 센서 데이터에 따라 모터 각도와 속도 제어
        if data_dict.get('0', 0) < 200:  # 조종기를 밀었을 때
            angle = 105  # 오른쪽으로 회전
            speed = d_speed  # 전진 속도
        elif data_dict.get('0', 0) > 200:  # 조종기를 당겼을 때
            angle = 75  # 왼쪽으로 회전
            speed = d_speed  # 후진 속도
        else:
            # 기본 정지 동작
            angle = 90
            speed = 5930  # 정지 속도

        # 서보 모터 각도 범위 설정
        angle = 180 if angle > 180 else (0 if angle < 0 else angle)
        calc_angle = angle + offset
        calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

        # 모터에 각도와 속도 적용
        print(data_dict, angle, speed, calc_angle, st)
        pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)

        # 스로틀에 설정된 속도 적용
        if back_speed != speed:
            back_speed = speed
            pca.channels[throttle_pin].duty_cycle = speed

    except Exception as e:
        print(f"오류 발생: {e}")

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 5930  # 스로틀 중지
calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = 6020  # 스로틀 초기화
