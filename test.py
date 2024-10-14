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

# 모터 각도 및 속도 초기화
angle = 0x5A        # 서보 모터 초기 각도 (90도)
offset = 0xFFFFFFF4  # 서보 모터 보정 값 (-12도)
speed = 0x1774      # 스로틀 초기 속도 (6020)
d_speed = 0x1732    # 기본 속도 (5950)
c_speed = 0x1732    # 센터 속도 (5950)
s_speed = 0x1732    # 서보 속도 (5950)
back_speed = speed   # 현재 속도 저장
st = 0x0            # 상태 플래그 (0)
last = {}           # 마지막 데이터 저장용

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
    try:
        data = arduino.readline().decode('utf-8').strip()  # 시리얼에서 한 줄 읽기
        print(f"수신된 데이터: {data}")  # 디버깅 출력
        data = data.split(',')  # ','로 분할하여 각 값 추출
        data_dict = {}
        for item in data:
            key_value = item.split(':')
            if len(key_value) == 2:  # key:value 형식인지 확인
                key = key_value[0].strip()
                value = int(key_value[1].strip())
                data_dict[key] = value  # 딕셔너리에 추가
        return data_dict
    except Exception as e:
        print(f"데이터 읽기 오류: {e}")
        return {}  # 오류 발생 시 빈 딕셔너리 반환

# 모터를 제어하는 함수
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
        data_dict = load_data()

        # 데이터가 충분한지 확인
        if 'angle' not in data_dict or 'speed' not in data_dict:
            print("유효하지 않은 데이터:", data_dict)
            continue

        angle = data_dict['angle']
        speed = data_dict['speed']

        # 모터 각도 및 속도 제어 범위 설정
        angle = max(0, min(180, angle))  # 각도 범위 0-180
        speed = max(0, min(100, speed))  # 속도 범위 0-100 (임의의 범위)

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
