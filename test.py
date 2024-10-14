import time
import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# 서보 및 스로틀 핀 설정
servo_pin = 0
throttle_pin = 1

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

# 조종기에서 데이터를 읽어오는 함수
def read_controller_data():
    try:
        # 시리얼 포트에서 데이터 읽기
        arduino.write(b'R')  # 조종기 데이터 요청 (예시)
        line = arduino.readline().decode('utf-8').strip()
        print(f"조종기 데이터: {line}")  # 디버깅 출력
        data = line.split(',')
        return {
            'angle': int(data[0]),  # 조종기에서 받은 각도 값
            'speed': int(data[1])   # 조종기에서 받은 속도 값
        }
    except Exception as e:
        print(f"조종기 데이터 읽기 오류: {e}")
        return {
            'angle': 90,  # 기본 각도 값
            'speed': 50   # 기본 속도 값
        }

# 모터를 제어하는 함수 (예시)
def control_motor(angle, speed):
    # 모터 제어 로직을 구현합니다.
    print(f"모터 각도: {angle}, 속도: {speed}")
    pca.channels[servo_pin].duty_cycle = servo_angle(angle)
    pca.channels[throttle_pin].duty_cycle = speed

# 서보 각도 계산기 초기화
servo_angle = servo_calc()

# 메인 루프: 조종기 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        # 조종기에서 데이터를 읽어옴
        controller_data = read_controller_data()
        
        # 조종기 데이터에 따라 모터 각도와 속도 제어
        angle = controller_data['angle']
        speed = controller_data['speed']
        
        # 모터 제어 로직 (예시)
        if angle < 75:
            angle = 45
        elif angle > 105:
            angle = 135
        
        # 속도 제어 로직 (예시)
        if speed < 20:
            speed = 20
        elif speed > 100:
            speed = 100
        
        # 모터 제어 함수 호출
        control_motor(angle, speed)
        
    except Exception as e:
        print(f"오류 발생: {e}")
        break

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 5930  # 스로틀 중지
calc_angle = 90 + offset
calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화

time.sleep(1.2)
pca.channels[throttle_pin].duty_cycle = 6020  # 스로틀 초기화