## 아두이노 Code
```python
#include <Servo.h>

// 서보 모터와 스로틀 모터에 대한 설정
int throttlePin = 9;
int steerPin = 10;

Servo throttleServo;
Servo steerServo;

// 조종기 입력 값을 처리하는 함수
String getCommandFromRemote() {
  int x = analogRead(A0);  // 조종기의 x축 값 (좌우)
  int y = analogRead(A1);  // 조종기의 y축 값 (전진, 후진)
  
  if (y > 800) {
    return "FORWARD";
  } else if (y < 200) {
    return "BACKWARD";
  } else if (x > 800) {
    return "RIGHT";
  } else if (x < 200) {
    return "LEFT";
  } else {
    return "CENTER";
  }
}

void setup() {
  Serial.begin(9600);  // 라즈베리파이와 시리얼 통신 시작
  throttleServo.attach(throttlePin);  // 스로틀 서보 모터 핀 연결
  steerServo.attach(steerPin);  // 스티어 서보 모터 핀 연결
}

void loop() {
  String command = getCommandFromRemote();  // 조종기 입력을 명령으로 변환
  Serial.println(command);  // 라즈베리파이에 명령 전송
  delay(100);  // 데이터를 일정 간격으로 보내기 위해 딜레이 추가
}
```


## vsCode
```
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import serial
import sys
import time

# 서보 모터 핀 및 스로틀 핀 설정
servo_pin = 0          # PCA9685의 채널 0번: 서보 모터 제어
throttle_pin = 1       # PCA9685의 채널 1번: 스로틀 제어

# 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

# I2C 및 PCA9685 설정
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40)  # PCA9685의 I2C 주소는 0x40
pca.frequency = 62.5  # 주파수 설정 (62.5Hz)

# 모터 각도 및 속도 초기화
angle = 90       # 서보 모터 초기 각도
offset = -12     # 서보 모터 보정 값
d_speed = 5950   # 기본 속도
c_speed = 5950   # 센터 속도
s_speed = 5950   # 서보 속도
back_speed = d_speed

# 서보 각도를 듀티 사이클로 변환하는 클래스
class servo_calc:
    def __init__(self):
        self.min_duty = int((750 * pca.frequency) / 1000000 * 0xFFFF)  # 최소 듀티 사이클
        max_duty = int((2250 * pca.frequency) / 1000000 * 0xFFFF)      # 최대 듀티 사이클
        self.duty_range = int(max_duty - self.min_duty)                # 듀티 사이클 범위

    def __call__(self, angle):
        return self.min_duty + int((angle / 180) * self.duty_range)     # 각도에 따른 듀티 사이클 반환

# 서보 각도 계산기 초기화
servo_angle = servo_calc()

# 조종기 입력 값을 처리하는 함수
def load_remote_control_data():
    data = arduino.readline()  # 시리얼에서 한 줄 읽기
    data = data.decode('ascii').strip()  # 아스키 형식으로 디코딩하고 공백 제거
    return data  # 조종기에서 받은 데이터를 그대로 반환

# 메인 루프: 조종기 입력에 따라 서보 및 스로틀 제어
while True:
    try:
        data = load_remote_control_data()  # 조종기에서 데이터를 읽어옴

        if data == 'FORWARD':
            speed = d_speed
            angle = 90  # 직진
        elif data == 'BACKWARD':
            speed = d_speed
            angle = 90  # 후진
        elif data == 'LEFT':
            angle = 45  # 왼쪽 회전
            speed = c_speed
        elif data == 'RIGHT':
            angle = 135  # 오른쪽 회전
            speed = c_speed
        elif data == 'CENTER':
            angle = 90  # 중심
            speed = c_speed
        else:
            continue  # 알 수 없는 입력이면 무시

        # 서보 모터 각도 범위 설정
        calc_angle = angle + offset
        calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

        # 모터에 각도와 속도 적용
        print(data, angle, speed, calc_angle)
        pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)
        if back_speed != speed:
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

```