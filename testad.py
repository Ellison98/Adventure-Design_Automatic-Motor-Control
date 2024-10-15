import serial
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

def running():
    # I2C 버스 설정
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    
    # PCA9685 주파수 설정
    pca.frequency = 63

    # 방향 및 속도 값 설정 (duty_cycle 값)
    left = 1093  # 왼쪽으로 회전
    center = 0x170C  # 직진 (중앙, 예시로 계속 사용할 수 있음)
    right = 1893  # 오른쪽으로 회전
    forward = 1200  # 전진
    stop = 0x1758  # 정지 (예시로 계속 사용할 수 있음)
    backward = 1694  # 후진

    # 기본 값 설정 (정지, 중심)
    pca.channels[0].duty_cycle = center
    pca.channels[1].duty_cycle = stop

    # 시리얼 통신 설정 (아두이노와 연결)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        while True:
            content = seri.readline().decode(errors='ignore').strip()
            try:
                values = content.split(',')
                steer_value = int(values[0].strip())  # 첫 번째 값은 스티어링
                throttle_value = int(values[1].strip())  # 두 번째 값은 스로틀
                print(f"스티어링: {steer_value}, 스로틀: {throttle_value}")

                # 스티어링 값에 따른 방향 설정
                if steer_value < 1400:  # 왼쪽으로 회전
                    pca.channels[0].duty_cycle = left
                elif steer_value > 1600:  # 오른쪽으로 회전
                    pca.channels[0].duty_cycle = right
                else:  # 중앙
                    pca.channels[0].duty_cycle = center

                # 스로틀 값에 따른 속도 설정
                if throttle_value < 1200:  # 후진
                    pca.channels[1].duty_cycle = backward
                elif throttle_value > 1694:  # 전진
                    pca.channels[1].duty_cycle = forward
                else:  # 정지
                    pca.channels[1].duty_cycle = stop

            except ValueError:
                print("wrong signal")

if __name__ == "__main__":
    running()
