import time
import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# 서보 및 스로틀 핀 설정
servo_pin = 0
throttle_pin = 1

# 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

# I2C 통신 및 PCA9685 초기화
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 0x3E  # 주파수 설정 (62.5)

# 각도 및 속도 초기화
offset = -0xC  # 서보 각도 보정값 (-12)
max_speed = 0x1774  # 최대 속도 (6020)
min_speed = 0x0  # 최소 속도 (0)

# 서보 모터 각도 계산 클래스
class ServoCalc:
    def __init__(self):
        self.min_duty = int((0x2EE * pca.frequency) / 0x1000000 * 0xFFFF)  # 750
        max_duty = int((0x8D2 * pca.frequency) / 0x1000000 * 0xFFFF)  # 2250
        self.duty_range = int(max_duty - self.min_duty)

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)

# 조종기에서 데이터를 읽어오는 함수
def read_controller_data():
    try:
        arduino.write(b'R')  # 조종기 데이터 요청 (예시)
        line = arduino.readline().decode('utf-8').strip()
        print(f"조종기 데이터: {line}")  # 디버깅 출력

        if "Throttle Motor Speed:" in line and "Throttle Duration (HIGH):" in line:
            throttle_speed = int(line.split("Throttle Motor Speed:")[1].split()[0])
            direction = line.split("Throttle Direction:")[1].strip()  # 방향을 가져옵니다.
            
            # 각도 설정 (왼쪽, 중간, 오른쪽)
            if direction == "LEFT":
                angle = 0  # 왼쪽
            elif direction == "RIGHT":
                angle = 180  # 오른쪽
            else:
                angle = 90  # 중앙 (직진)

            return {
                'angle': angle,
                'speed': throttle_speed
            }
        else:
            print("조종기 데이터 형식이 올바르지 않습니다.")
            return {
                'angle': 90,  # 기본 각도 값
                'speed': 0    # 기본 속도 값
            }
    except Exception as e:
        print(f"조종기 데이터 읽기 오류: {e}")
        return {
            'angle': 90,  # 기본 각도 값
            'speed': 0    # 기본 속도 값
        }

# 모터를 제어하는 함수
def control_motor(angle, speed):
    print(f"모터 각도 (16진수): {hex(angle)}, 속도 (16진수): {hex(speed)}")
    pca.channels[servo_pin].duty_cycle = servo_angle(angle)
    
    # 스로틀 속도 설정
    if speed > 0:  # 전진
        pca.channels[throttle_pin].duty_cycle = speed
    elif speed < 0:  # 후진
        pca.channels[throttle_pin].duty_cycle = -speed  # 후진 속도는 음수 처리
    else:  # 정지
        pca.channels[throttle_pin].duty_cycle = 0

# 서보 각도 계산기 초기화
servo_angle = ServoCalc()

# 메인 루프: 조종기 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        controller_data = read_controller_data()
        
        angle = controller_data['angle']
        speed = controller_data['speed']
        
        # 각도 제어: 0에서 180도 사이의 범위로 제한
        angle = max(0, min(angle, 180))
        
        # 속도 제어: 최소 및 최대 속도 제한
        speed = max(min_speed, min(speed, max_speed))
        
        # 모터 제어 함수 호출
        control_motor(angle, speed)

        # 모터 작동 상태 출력
        if speed > 0:
            print("모터가 작동 중입니다.")
        else:
            print("모터가 정지했습니다.")

    except Exception as e:
        print(f"오류 발생: {e}")
        break

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 0  # 스로틀 중지
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화
time.sleep(1.2)

# 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = 0
