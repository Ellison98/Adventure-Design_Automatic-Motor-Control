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

data_dict = {}

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 62.5  # 주파수 설정 (62.5Hz)

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
    data = arduino.readline()  # 시리얼에서 한 줄 읽기
    data = data.decode('ascii').split(' ')  # 아스키 형식으로 디코딩하고 공백으로 분할
    data = [for_a.split(':') for for_a in data]  # ':'을 기준으로 키-값 분할
    return {for_a[0]: int(for_a[1].replace('\r\n', '')) for for_a in data}  # 딕셔너리로 반환

# 서보 각도 계산기 초기화
servo_angle = servo_calc()

# 메인 루프: 센서 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        if last != {}:
            data_dict = last
            last = {}
        else:
            data_dict = load_data()  # 시리얼에서 데이터를 읽어옴

        # 센서 데이터에 따라 모터 각도와 속도 제어
        if data_dict['0'] < 200 and data_dict['1'] < 200 and data_dict['2'] < 200:
            # 000: 장애물 없음
            if angle < 90:
                if angle < 75:
                    angle = 45
                else:
                    angle -= 1
            else:
                if angle > 105:
                    angle = 135
                else:
                    angle += 1
            speed = d_speed
        elif data_dict['0'] > 200 and data_dict['1'] < 200 and data_dict['2'] > 200:
            # 101: 특별한 동작 없음
            pass
        elif data_dict['0'] < 200 and data_dict['1'] > 200 and data_dict['2'] < 200:
            # 010: 정면에 장애물, 직진
            angle = 90
            if st == 0:
                speed = s_speed
                st = 1
        elif data_dict['0'] < 200 and data_dict['1'] < 200 and data_dict['2'] > 200:
            # 001: 오른쪽 회전
            angle = 105
            speed = s_speed
        elif data_dict['0'] < 200 and data_dict['1'] > 200 and data_dict['2'] > 200:
            # 011: 오른쪽 약간 회전
            angle = 95
            speed = s_speed
        elif data_dict['0'] > 200 and data_dict['1'] < 200 and data_dict['2'] < 200:
            # 100: 왼쪽 회전
            angle = 75
            speed = s_speed
        elif data_dict['0'] > 200 and data_dict['1'] > 200 and data_dict['2'] < 200:
            # 110: 왼쪽 약간 회전
            angle = 85
            speed = s_speed
        elif data_dict['0'] > 200 and data_dict['1'] > 200 and data_dict['2'] > 200:
            # 111: 멈춤
            if st == 1:
                break_true = 0
                for for_a in range(0, 20):
                    data_dict = load_data()
                    if data_dict['0'] < 200 and data_dict['1'] > 200 and data_dict['2'] < 200:
                        pass
                    elif data_dict['0'] < 200 and data_dict['1'] < 200 and data_dict['2'] < 200:
                        break
                    else:
                        last = data_dict
                else:
                    break_true = 1

                if break_true == 1:
                    break

        # 서보 모터 각도 범위 설정
        angle = 180 if angle > 180 else (0 if angle < 0 else angle)
        calc_angle = angle + offset
        calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

        # 모터에 각도와 속도 적용
        print(data_dict, angle, speed, calc_angle, st)
        pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)
        if back_speed != speed and st == 1:
            back_speed = speed
            pca.channels[throttle_pin].duty_cycle = speed
    except Exception as e:
        print(e)

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 5930  # 스로틀 중지
calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = 6020  # 스로틀 초기화
