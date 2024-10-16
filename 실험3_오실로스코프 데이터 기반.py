import serial  # 시리얼 통신을 위한 라이브러리
import busio  # I2C 통신을 위한 라이브러리
from board import SCL, SDA  # 보드의 SCL 및 SDA 핀을 가져옴
from adafruit_pca9685 import PCA9685  # PCA9685 모듈을 위한 라이브러리

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)  # I2C 버스 초기화
pca = PCA9685(i2c_bus)  # PCA9685 객체 생성

# PCA9685 주파수 설정
pca.frequency = 60  # PWM 주파수를 60Hz로 설정

# 아두이노에서 받은 PWM 범위에 따른 듀티 사이클 설정
STEERING_CHANNEL = 0   # 서보모터에 해당하는 PCA9685의 채널 번호
THROTTLE_CHANNEL = 1   # DC 모터 또는 서보모터에 해당하는 채널 번호

# 듀티 사이클 범위
STEER_MIN_PWM = 220  # 서보 모터의 최소 PWM 값
STEER_MAX_PWM = 420  # 서보 모터의 최대 PWM 값

THROTTLE_MIN_PWM = 240  # 쓰로틀 모터의 최소 PWM 값
THROTTLE_MAX_PWM = 380  # 쓰로틀 모터의 최대 PWM 값

# % 값에 대한 PWM 맵핑 정의
steering_pwm_mapping = {
    6: 1100,  # 6%에 해당하는 PWM 값
    9.5: 2200,  # 9.5%에 해당하는 PWM 값
    11.9: 3932  # 11.9%에 해당하는 PWM 값
}

throttle_pwm_mapping = {
    6: 240,  # 6%에 해당하는 PWM 값
    9.5: 320,  # 9.5%에 해당하는 PWM 값
    13.1: 380  # 13.1%에 해당하는 PWM 값
}

def get_pwm_from_percent(value, mapping):
    """주어진 % 값에 대해 PWM 값을 반환하는 함수
    
    Args:
        value (float): 변환할 % 값
        mapping (dict): % 값과 PWM 값의 매핑

    Returns:
        int: 변환된 PWM 값, 매핑에 없을 경우 None
    """
    return mapping.get(value, None)  # 매핑에서 % 값을 검색하여 PWM 값 반환

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수
    
    Args:
        steer_value (float): 스티어링 % 값
        throttle_value (float): 쓰로틀 % 값
    """
    # % 값을 PWM 값으로 변환
    steer_pwm = get_pwm_from_percent(steer_value, steering_pwm_mapping)
    throttle_pwm = get_pwm_from_percent(throttle_value, throttle_pwm_mapping)

    # PWM 값이 None이 아닐 경우에만 설정
    if steer_pwm is not None and throttle_pwm is not None:
        # PCA9685의 채널에 PWM 값 설정
        pca.channels[STEERING_CHANNEL].duty_cycle = steer_pwm
        pca.channels[THROTTLE_CHANNEL].duty_cycle = throttle_pwm
        print(f"스티어링 PWM 값: {steer_pwm}, 쓰로틀 PWM 값: {throttle_pwm}")  # 디버깅 메시지
    else:
        print("올바른 PWM 값을 찾을 수 없습니다.")  # 유효하지 않은 PWM 값일 경우 경고 메시지

def running():
    """주 실행 함수, 아두이노로부터 데이터를 읽어와 모터를 제어하는 함수"""
    # 시리얼 포트 설정 (아두이노에서 데이터 수신)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        while True:  # 무한 루프
            try:
                # 아두이노로부터 데이터 읽기
                content = seri.readline().decode(errors='ignore').strip()  # 시리얼 통신으로부터 데이터 읽기
                values = content.split(',')  # 쉼표로 구분된 문자열 분리

                # 값이 두 개 미만이면 처리하지 않음
                if len(values) < 2:
                    print("잘못된 데이터 형식")  # 데이터 형식이 잘못된 경우 경고 메시지
                    continue

                # 아두이노로부터 받은 값을 실수로 변환
                steer_duration = float(values[0].strip())  # 스티어링 % 값
                throttle_duration = float(values[1].strip())  # 쓰로틀 % 값

                # 모터의 PWM 값 설정
                set_motor_pwm(steer_duration, throttle_duration)  # 변환된 PWM 값을 모터에 설정

            except ValueError:
                print("잘못된 신호")  # 값 변환 실패 시 경고 메시지
            except Exception as e:
                print(f"예외 발생: {e}")  # 기타 예외 발생 시 경고 메시지

if __name__ == "__main__":
    running()  # 프로그램 실행
