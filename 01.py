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
time.sleep(2)  # 아두이노와의 연결을 위한 대기 시간

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 62.5  # 주파수 설정 (62.5Hz)

# 모터 각도 및 속도 초기화
angle = 90       # 서보 모터 초기 각도
offset = -12     # 서보 모터 보정 값
speed = 6020     # 스로틀 초기 속도

# 서보 모터 및 스로틀 속도 조절 함수
def set_servo_angle(angle):
    pulse_width = angle / 180 * 2 + 0.5  # 각도를 PWM 신호로 변환
    pca.channels[servo_pin].duty_cycle = int(pulse_width * 65535)  # PWM 설정

def set_throttle_speed(speed):
    pulse_width = speed / 10000 * 2 + 0.5  # 속도를 PWM 신호로 변환
    pca.channels[throttle_pin].duty_cycle = int(pulse_width * 65535)  # PWM 설정

try:
    while True:
        if arduino.in_waiting > 0:
            data = arduino.readline().decode('utf-8').strip()  # 조종기 값 읽기
            values = data.split(',')
            if len(values) >= 2:  # 서보 각도와 스로틀 속도가 오는 경우
                angle = int(values[0]) + offset  # 보정된 각도
                speed = int(values[1])  # 스로틀 속도

                # 각도 및 속도 설정
                set_servo_angle(angle)
                set_throttle_speed(speed)
                
            time.sleep(0.1)  # 루프 주기 조정

except KeyboardInterrupt:
    print("프로그램이 종료되었습니다.")
finally:
    pca.deinit()  # PCA9685 해제
    arduino.close()  # 시리얼 통신 종료
