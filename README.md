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

## 함수변경
```python
void setup() {
  Serial.begin(9600);
  
  pinMode(steer, INPUT);
  pinMode(throttle, INPUT);
}

void loop() {
  int steer_value = digitalRead(steer);
  int throttle_value = digitalRead(throttle);
  
  Serial.print("Steering Value: ");
  Serial.print(steer_value);
  Serial.print(", Throttle Value: ");
  Serial.println(throttle_value);
  
  delay(100);
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

def running():
    # I2C 버스 설정
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    
    # PCA9685 주파수 설정
    pca.frequency = 60

    # 방향 및 속도 값 설정 (duty_cycle 값)
    left = 0x0C80  # 왼쪽으로 회전 (3200)
    center = 0x1760  # 직진 (6000)
    right = 0x1F40  # 오른쪽으로 회전 (8000)
    
    forward = 0x1B72  # 전진 (7038)
    stop = 0x15D4  # 정지 (5588)
    backward = 0x1274 # 후진 (3200)

    # 기본 값 설정 (정지, 중심)
    pca.channels[0].duty_cycle = center
    pca.channels[1].duty_cycle = stop

    # 시리얼 통신 설정 (아두이노와 연결)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        while True:
            content = seri.readline().decode(errors='ignore').strip()
            try:
                values = content.split(',')
                if len(values) < 2:  # 스티어링과 스로틀 값이 충분하지 않은 경우
                    print("잘못된 데이터 형식")
                    continue
                
                # 스티어링 및 스로틀 값 안전하게 변환
                steer_value = int(values[0].strip())  # 첫 번째 값은 스티어링
                throttle_value = int(values[1].strip())  # 두 번째 값은 스로틀

                print(f"스티어링: {steer_value}, 스로틀: {throttle_value}")

                # 스티어링 값에 따른 방향 설정 -> 아두이노에서 전달되는 신호를 PWM신호로 봐꿔서 젯슨 나노에서 범위지정
                if steer_value < 1200:  # 왼쪽으로 회전
                    pca.channels[0].duty_cycle = left
                elif steer_value > 1600:  # 오른쪽으로 회전
                    pca.channels[0].duty_cycle = right
                else:  # 중앙
                    pca.channels[0].duty_cycle = center

                # 스로틀 값에 따른 속도 설정
                if throttle_value < 1100:  # 후진
                    pca.channels[1].duty_cycle = backward
                elif throttle_value < 1500:  # 정지
                    pca.channels[1].duty_cycle = stop
                else:  # 전진
                    pca.channels[1].duty_cycle = forward

            except ValueError as e:
                print(f"잘못된 신호: {e}")
            except Exception as e:
                print(f"예외 발생: {e}")

if __name__ == "__main__":
    running()
```
