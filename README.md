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
  steer_duration = pulseIn(steer, HIGH, 90000);
  
  // 쓰로틀의 HIGH 신호 지속 시간 측정
  throttle_duration = pulseIn(throttle, HIGH, 900000);

  // 시리얼 모니터에 스티어링, 쓰로틀의 PWM 값 및 변환된 모터 속도 출력

  Serial.print(steer_duration);
  Serial.print(',');
  Serial.print(throttle_duration);
  Serial.print('\n');

 
  delay(100);  // 100ms 지연
}
```

```python
    # 방향 및 속도 값 설정 (duty_cycle 값)
    left = 0x0C80  # 왼쪽으로 회전 (3200)
    center = 0x1388  # 직진 (5000)
    right = 0x1F40  # 오른쪽으로 회전 (8000)
    
    forward = 0x1F40  # 전진 (8000)
    stop = 0x1388  # 정지 (5000)
    backward = 0x0C80  # 후진 (3200)
```


## vsCode
```
import serial
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

def initialize_pca9685():
    """PCA9685 초기화 및 주파수 설정."""
    i2c_bus = busio.I2C(SCL, SDA)
    pca_controller = PCA9685(i2c_bus)
    pca_controller.frequency = 50  # 서보 모터의 기본 주파수
    return pca_controller

def set_motor_controls(pca_controller, steering_input, throttle_input):
    """스티어링 및 스로틀 입력에 따라 모터 제어."""
    # 모터의 PWM 값 설정
    left_pwm = 1093        # 왼쪽으로 회전
    center_pwm = 0x170C    # 중앙 (직진)
    right_pwm = 1893       # 오른쪽으로 회전
    forward_pwm = 1200     # 전진
    stop_pwm = 0x1758      # 정지
    backward_pwm = 1694    # 후진

    # 스티어링 입력에 따른 방향 설정
    if steering_input < 1400:  # 왼쪽으로 회전
        pca_controller.channels[0].duty_cycle = left_pwm
    elif steering_input > 1600:  # 오른쪽으로 회전
        pca_controller.channels[0].duty_cycle = right_pwm
    else:  # 중앙
        pca_controller.channels[0].duty_cycle = center_pwm

    # 스로틀 입력에 따른 속도 설정
    if throttle_input < 1200:  # 후진
        pca_controller.channels[1].duty_cycle = backward_pwm
    elif throttle_input > 1694:  # 전진
        pca_controller.channels[1].duty_cycle = forward_pwm
    else:  # 정지
        pca_controller.channels[1].duty_cycle = stop_pwm

# PCA9685 초기화
pca_controller = initialize_pca9685()

# 기본 값 설정 (정지, 중앙)
pca_controller.channels[0].duty_cycle = 0x170C  # 중앙
pca_controller.channels[1].duty_cycle = 0x1758  # 정지

# 시리얼 통신 설정 (아두이노와 연결)
with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as serial_connection:
    while True:
        data_line = serial_connection.readline().decode(errors='ignore').strip()
        try:
            # 시리얼 데이터에서 스티어링 및 스로틀 값 추출
            steering_input, throttle_input = map(int, data_line.split(','))
            print(f"스티어링: {steering_input}, 스로틀: {throttle_input}")

            # 모터 제어
            set_motor_controls(pca_controller, steering_input, throttle_input)

        except ValueError:  
            print("잘못된 신호")

```
