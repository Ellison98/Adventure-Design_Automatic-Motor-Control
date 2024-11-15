## 현재 적용중인 코드

## 아두이노 Code
```python

// 센서 신호
#define LEFT_SENSORPIN A1
#define CENTER_SENSORPIN A2
#define RIGHT_SENSORPIN A3

// 조종기 수신 신호
int steer = 7;      // 스티어링 입력
int throttle = 8;   // 쓰로틀 입력
int switch_bt = 9;


unsigned long steer_duration;
unsigned long throttle_duration;
unsigned long switch_bt_duration;


void setup() {
  Serial.begin(9600);
  pinMode(steer, INPUT);
  pinMode(throttle, INPUT);
  pinMode(switch_bt, INPUT);
  pinMode(LEFT_SENSORPIN, INPUT);
  pinMode(CENTER_SENSORPIN, INPUT);
  pinMode(RIGHT_SENSORPIN, INPUT);
}

void loop() {
  // 스티어링의 HIGH 신호 지속 시간 측정
  steer_duration = pulseIn(steer, HIGH, 90000);
  
  // 쓰로틀의 HIGH 신호 지속 시간 측정
  throttle_duration = pulseIn(throttle, HIGH, 900000);
  
  switch_bt_duration = pulseIn(switch_bt, HIGH, 90000);

  // 시리얼 모니터에 스티어링, 쓰로틀의 PWM 값 및 변환된 모터 속도 출력
  byte leftSensor=digitalRead(LEFT_SENSORPIN);
  byte centorSensor=digitalRead(CENTER_SENSORPIN);
  byte rightSensor=digitalRead(RIGHT_SENSORPIN);

  Serial.print(steer_duration);
  Serial.print(',');
  Serial.print(throttle_duration);
  Serial.print(',');
  Serial.print(switch_bt_duration);
  Serial.print(',');
  Serial.print(leftSensor);
  Serial.print(',');
  Serial.print(centorSensor);
  Serial.print(',');
  Serial.print(rightSensor);
  Serial.print('\n');
 
 
  delay(100);  // 100ms 지연
}
```

## 젯슨나노 Code VsCode
```python
import serial
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# I2C 및 PCA9685 설정
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 60

# 듀티 사이클 범위 (0~65535: 16비트 범위)
PWM_MIN = 1500
PWM_MAX = 12000

# 스티어링과 쓰로틀의 입력 범위 (예: 아두이노에서 오는 값)
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 자동/수동 모드 기준값
SWITCH_THRESHOLD = 1350

def map_value(value, in_min, in_max, out_min, out_max):
    """값을 특정 범위에서 다른 범위로 매핑하는 함수"""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def set_motor_pwm(steer_value, throttle_value):
    """모터의 PWM 값을 설정하는 함수"""
    steer_pwm = map_value(steer_value, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)
    throttle_pwm = map_value(throttle_value, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)

    # PWM 값 출력 (디버깅용)
    print(f"입력 스티어링: {steer_value}, 입력 쓰로틀: {throttle_value}")
    print(f"변환된 스티어링 PWM: {steer_pwm}, 변환된 쓰로틀 PWM: {throttle_pwm}")

    pca.channels[0].duty_cycle = steer_pwm
    pca.channels[1].duty_cycle = throttle_pwm

def follow_line(left_sensor, center_sensor, right_sensor,lt,rt):
    """센서 값을 기반으로 라인을 따라 천천히 움직이는 함수"""
    if center_sensor == 1 and left_sensor==0 and right_sensor==0:
        print("직진 중...")
        # # 중앙 센서가 라인을 인식하면 직진 (스로틀 1700)
        pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)
        pca.channels[0].duty_cycle = map_value(1390, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 스티어링 중립 위치
        lt= False
        rt= True
    elif left_sensor == 1 and right_sensor==1:
        pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
        pca.channels[0].duty_cycle = map_value(1390, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 왼쪽 회전
        lt= False
        rt= False


    elif left_sensor == 1:
        print("왼쪽으로 조금 회전")
        # 왼쪽 센서가 라인을 인식하면 왼쪽으로 회전
        if center_sensor==1:
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
            pca.channels[0].duty_cycle = map_value(1250, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 왼쪽 회전
        else:
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
            pca.channels[0].duty_cycle = map_value(1240, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 왼쪽 회전
        lt=True
        rt=False

    elif right_sensor == 1:
        print("오른쪽으로 조금 회전")
        # 오른쪽 센서가 라인을 인식하면 오른쪽으로 회전
        if center_sensor==1:
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
            pca.channels[0].duty_cycle = map_value(1510, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 오른쪽 회전
        else:
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
            pca.channels[0].duty_cycle = map_value(1520, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 오른쪽 회전
        lt=False
        rt=True
        
    elif left_sensor==0 and center_sensor==0 and right_sensor==0:
        if lt:
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
            pca.channels[0].duty_cycle = map_value(1240, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 왼쪽 회전
        elif rt:
            pca.channels[1].duty_cycle = map_value(1300, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 계속 진행
            pca.channels[0].duty_cycle = map_value(1520, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 오른쪽 회전
        else:
            print("정지 중...")
            # 모든 센서가 라인을 감지하지 못하면 정지 (스로틀 1400)
            pca.channels[1].duty_cycle = map_value(1265, THROTTLE_MIN, THROTTLE_MAX, PWM_MIN, PWM_MAX)  # 정지
            pca.channels[0].duty_cycle = map_value(1390, STEER_MIN, STEER_MAX, PWM_MIN, PWM_MAX)  # 스티어링 중립 위치
    return lt,rt

def running():
    # 시리얼 포트 설정 (아두이노에서 데이터 수신)
    with serial.Serial('/dev/ttyACM0', 9600, timeout=None) as seri:
        # 시리얼 버퍼 비우기
        seri.reset_input_buffer()
        lt,rt=False,False
        while True:
            try:
                # 아두이노로부터 데이터 읽기
                content = seri.readline().decode(errors='ignore').strip()
                values = content.split(',')

                if len(values) < 6:
                    print(f"잘못된 데이터 형식: {content}")
                    continue

                if all(value.isdigit() for value in values[:6]):
                    steer_duration = int(values[0].strip())
                    throttle_duration = int(values[1].strip()) - 48
                    switch_bt_duration = int(values[2].strip())
                    left_sensor = int(values[3].strip())
                    center_sensor = int(values[4].strip())
                    right_sensor = int(values[5].strip())

                    # 모드 구분
                    mode = "MANUAL" if switch_bt_duration >= SWITCH_THRESHOLD else "AUTO"
                    print(f"모드: {mode} (스위치 값: {switch_bt_duration})")

                    if mode == "MANUAL":
                        if STEER_MIN <= steer_duration <= STEER_MAX and THROTTLE_MIN <= throttle_duration <= THROTTLE_MAX:
                            set_motor_pwm(steer_duration, throttle_duration)
                        else:
                            print(f"비정상적인 값 범위: {steer_duration}, {throttle_duration}")
                    else:
                        lt,rt=follow_line(left_sensor, center_sensor, right_sensor,lt,rt)

                else:
                    print(f"숫자가 아닌 값 수신됨: {content}")

            except ValueError:
                print(f"잘못된 신호: {content}")
            except Exception as e:
                print(f"예외 발생: {e}")

if __name__ == "__main__":
    running()
```


sudo apt-get update
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad

```
import cv2

# GStreamer 파이프라인을 이용해 카메라 스트림 열기
# 비디오 캡처 장치로 v4l2 (Video4Linux2) 사용
gst_str = "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink"

# OpenCV에서 GStreamer 파이프라인을 사용하여 비디오 캡처
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

# 카메라가 제대로 열리지 않은 경우
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 영상 출력 루프
while True:
    # 프레임 읽기
    ret, frame = cap.read()

    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 읽은 프레임을 화면에 표시
    cv2.imshow('Camera Stream', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제 및 창 닫기
cap.release()
cv2.destroyAllWindows()
```


sudo apt-get install v4l2-utils
v4l2-ctl --list-devices


sudo apt install build-essential
sudo apt install libv4l-dev

git clone https://github.com/umlaeute/v4l2-utils.git
cd v4l2-utils
