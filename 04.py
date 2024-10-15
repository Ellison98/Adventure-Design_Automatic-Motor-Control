import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
import time

# 서보 모터 핀 및 스로틀 핀 설정
servo_pin = 0  # PCA9685의 채널 0번: 서보 모터 제어
throttle_pin = 1  # PCA9685의 채널 1번: 스로틀 제어

# 시리얼 통신 설정 (아두이노와 통신)
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 50  # 주파수 설정 (50Hz)

# 모터 속도 및 각도 초기화
back_speed = 5930  # 후진 속도
stop_speed = 5950  # 정지 속도
forward_speed = 6020  # 전진 속도
offset = -12  # 서보 모터 보정 값

# 서보 각도를 듀티 사이클로 변환하는 클래스
class servo_calc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)  # 최소 듀티 사이클
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)  # 최대 듀티 사이클
        self.duty_range = int(max_duty - self.min_duty)  # 듀티 사이클 범위

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)  # 각도에 따른 듀티 사이클 반환

# 서보 각도 계산기 초기화
servo_angle = servo_calc()

# 시리얼 데이터 파싱 함수
def parse_data(data_line):
    try:
        # " | "를 기준으로 문자열을 분할
        steering_data, throttle_data = data_line.split(" | ")

        # 스티어링 값 추출
        steering_duration = int(steering_data.split(": ")[1].strip())
        steering_speed = int(steering_data.split(": ")[2].strip())

        # 쓰로틀 값 추출
        throttle_duration = int(throttle_data.split(": ")[1].strip())
        throttle_speed = int(throttle_data.split(": ")[2].strip())

        return steering_duration, steering_speed, throttle_duration, throttle_speed
    except (IndexError, ValueError) as e:
        print(f"Error parsing data: {e}")
        return None

# 듀티 사이클 계산 함수
def calculate_duty_cycle(high_duration, low_duration):
    total_duration = high_duration + low_duration
    if total_duration == 0:
        return 0
    return int((high_duration / total_duration) * 65535)  # 듀티 사이클 값 반환

# 메인 루프: 조종기 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        # 시리얼에서 한 줄 읽기
        data_line = arduino.readline().decode('utf-8').strip()

        if data_line:
            # 데이터를 파싱하여 스티어링과 쓰로틀 값 추출
            parsed_data = parse_data(data_line)

            if parsed_data:
                steering_duration, steering_speed, throttle_duration, throttle_speed = parsed_data

                # 쓰로틀 값을 기반으로 모터 제어 (후진, 정지, 전진)
                if throttle_duration < 1200:
                    # 후진
                    pca.channels[throttle_pin].duty_cycle = back_speed
                elif 1201 <= throttle_duration <= 1694:
                    # 정지
                    pca.channels[throttle_pin].duty_cycle = stop_speed
                else:
                    # 전진
                    pca.channels[throttle_pin].duty_cycle = forward_speed

                # 서보 모터 각도 범위 설정
                angle = int((steering_duration - 1000) * (180 / 1000))  # 각도 0~180도 범위로 변환
                angle = 0 if angle < 0 else (180 if angle > 180 else angle)  # 각도 제한
                calc_angle = angle + offset
                calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

                # 모터에 각도와 속도 적용
                pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)

                # 파싱된 데이터 출력 (디버깅용)
                print(f"Steering Duration: {steering_duration}, Speed: {steering_speed}")
                print(f"Throttle Duration: {throttle_duration}, Speed: {throttle_speed}")

        time.sleep(0.1)

    except Exception as e:
        print(e)
        break

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = stop_speed  # 스로틀 중지
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화
