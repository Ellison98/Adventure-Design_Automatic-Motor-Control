## 아두이노 Code
```python
int steer = 7;      // 스티어링 입력
int throttle = 8;   // 쓰로틀 입력

unsigned long steer_duration;
unsigned long throttle_duration;

void setup() {
  Serial.begin(9600);
  pinMode(steer, INPUT);
  pinMode(throttle, INPUT);
}

void loop() {
  // 스티어링의 HIGH 신호 지속 시간 측정
  steer_duration = pulseIn(steer, HIGH, 100000);
  
  // 쓰로틀의 HIGH 신호 지속 시간 측정
  throttle_duration = pulseIn(throttle, HIGH, 100000);

  // 시리얼 모니터에 스티어링, 쓰로틀의 PWM 값 및 변환된 모터 속도 출력

  Serial.print(steer_duration);
  Serial.print(',');
  Serial.print(throttle_duration);
  Serial.print('\n');

 
  delay(100);  // 100ms 지연
}
```


## 아두이노 스티어링 확인 코드
```
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

  // 스티어링 및 쓰로틀 값 출력
  Serial.print("\t 스티어링 PWM Duration (HIGH): ");
  Serial.print(steer_duration);
  Serial.print("\t 스티어링 모터 속도 (mapped): ");
  Serial.print(motorSpeed2);
  Serial.println();

  Serial.print("\t 전/후진 PWM Duration (HIGH): ");
  Serial.print(throttle_duration);
  Serial.print("\t 전/후진 모터 속도 (mapped): ");
  Serial.print(motorSpeed1);
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

# 듀티 사이클 계산 함수
def calculate_duty_cycle(high_duration, low_duration):
    total_duration = high_duration + low_duration
    if total_duration == 0:
        return 0
    return int((high_duration / total_duration) * 65535)  # 듀티 사이클 값 반환

# 시리얼 통신으로 데이터 읽기 함수
def load_data():
    data = arduino.readline()  # 시리얼에서 한 줄 읽기
    if data:
        data = data.decode('ascii').split('\t')  # 아스키 형식으로 디코딩하고 탭으로 분할
        steer_duration = int(data[1].strip())  # 스티어링 지속 시간
        throttle_duration = int(data[3].strip())  # 쓰로틀 지속 시간
        return steer_duration, throttle_duration
    return None, None

# 메인 루프: 조종기 데이터를 기반으로 서보 및 스로틀 제어
while True:
    try:
        steer_duration, throttle_duration = load_data()  # 조종기에서 데이터를 읽어옴

        if steer_duration and throttle_duration:
            # 조종기에서 받은 HIGH/LOW 신호를 기반으로 듀티 사이클 계산
            steer_duty_cycle = calculate_duty_cycle(steer_duration, 20000 - steer_duration)  # 스티어링 값 계산
            throttle_duty_cycle = calculate_duty_cycle(throttle_duration, 20000 - throttle_duration)  # 쓰로틀 값 계산

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
            angle = int((steer_duration - 1000) * (180 / 1000))  # 각도 0~180도 범위로 변환
            angle = 0 if angle < 0 else (180 if angle > 180 else angle)  # 각도 제한
            calc_angle = angle + offset
            calc_angle = 180 if calc_angle > 180 else (0 if calc_angle < 0 else calc_angle)

            # 모터에 각도와 속도 적용
            pca.channels[servo_pin].duty_cycle = servo_angle(calc_angle)

            print(f"Steering Duty Cycle: {steer_duty_cycle}, Throttle Duty Cycle: {throttle_duty_cycle}")
            print(f"Steering: {steer_duration}, Throttle: {throttle_duration}, Angle: {angle}, Speed: {pca.channels[throttle_pin].duty_cycle}")

        time.sleep(0.1)

    except Exception as e:
        print(e)
        break

# 종료 시 서보 및 스로틀 초기화
pca.channels[throttle_pin].duty_cycle = stop_speed  # 스로틀 중지
pca.channels[servo_pin].duty_cycle = servo_angle(90)  # 서보 모터 중앙 위치로 초기화
```
