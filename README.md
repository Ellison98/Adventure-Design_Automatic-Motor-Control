## 아두이노 Code
```python
int steer = 7;      // 스티어링 입력
int throttle = 8;   // 쓰로틀 입력
int motor1Pin = 9;  // 모터 1 출력 (PWM 핀)
int motor2Pin = 10; // 모터 2 출력 (PWM 핀)

unsigned long steer_duration;
unsigned long steer_overall_duration;
unsigned long throttle_duration;
unsigned long throttle_overall_duration;

void setup() {
  Serial.begin(9600);
  pinMode(steer, INPUT);
  pinMode(throttle, INPUT);
  pinMode(motor1Pin, OUTPUT);  // 모터 1 핀을 출력으로 설정
  pinMode(motor2Pin, OUTPUT);  // 모터 2 핀을 출력으로 설정
}

void loop() {
  // 스티어링의 HIGH 신호 지속 시간 측정
  steer_duration = pulseIn(steer, HIGH);
  steer_overall_duration = pulseIn(steer, LOW) + steer_duration;

  // 쓰로틀의 HIGH 신호 지속 시간 측정
  throttle_duration = pulseIn(throttle, HIGH);
  throttle_overall_duration = pulseIn(throttle, LOW) + throttle_duration;

  // 스티어링 및 쓰로틀 값을 0~255 범위의 PWM 값으로 변환
  int motorSpeed1 = map(throttle_duration, 1000, 2000, 0, 255);  // 쓰로틀을 모터 속도로 변환
  int motorSpeed2 = map(steer_duration, 1000, 2000, 0, 255);     // 스티어링을 모터 속도로 변환

  // 모터에 PWM 신호 전달 (속도 제어)
  analogWrite(motor1Pin, motorSpeed1);
  analogWrite(motor2Pin, motorSpeed2);

  // 시리얼 모니터에 스티어링, 쓰로틀의 PWM 값 및 변환된 모터 속도 출력
  Serial.print("\t 스티어링 PWM Duration (HIGH): ");
  Serial.print(steer_duration);
  //Serial.print("\t 조향 모터 속도 (mapped): ");
  //Serial.print(motorSpeed2);  // 변환된 모터 속도 출력
  Serial.println();

  Serial.print("\t 전/후진 PWM Duration (HIGH): ");
  Serial.print(throttle_duration);
  Serial.print("\t 전/후진 모터 속도 (mapped): ");
  Serial.print(motorSpeed1);  // 변환된 모터 속도 출력
  Serial.println();
 
  delay(1000);  // 1000ms 지연
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