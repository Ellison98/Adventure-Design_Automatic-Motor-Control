```python
def load_data():
    data = arduino.readline()  # 시리얼에서 한 줄 읽기
    print(f"Raw data: {data}")  # 수신된 원본 데이터 출력
    try:
        data = data.decode('utf-8').strip()  # UTF-8로 디코딩
    except UnicodeDecodeError:
        print(f"Failed to decode data: {data}")  # 디코딩 실패 시 원본 바이트 출력
        return {}

    if not data:  # 데이터가 비어있으면
        return {}

    data = data.split(' ')  # 공백으로 분할
    print(f"Split data: {data}")  # 분할된 데이터 출력
    
    # ':'이 포함된 데이터만 처리
    result = {}
    for for_a in data:
        if ':' in for_a:  # ':'이 있는 항목만 처리
            key_value = for_a.split(':')  # 키-값 분할
            if len(key_value) == 2 and key_value[1].isdigit():  # 두 항목이 있고 두 번째 항목이 숫자인지 확인
                result[key_value[0]] = int(key_value[1])  # 딕셔너리에 추가
            else:
                print(f"Invalid data: {key_value}")  # 유효하지 않은 데이터 출력

    return result  # 딕셔너리 반환
```


<br>



```python
int steer = 7;      // 스티어링 입력
int throttle = 8;   // 쓰로틀 입력
int motor1Pin = 9;  // 모터 1 출력 (PWM 핀)
int motor2Pin = 10; // 모터 2 출력 (PWM 핀)

unsigned long steer_duration;
unsigned long throttle_duration;

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
  
  // 쓰로틀의 HIGH 신호 지속 시간 측정
  throttle_duration = pulseIn(throttle, HIGH);

  // 스티어링 및 쓰로틀 값을 0~255 범위의 PWM 값으로 변환
  int motorSpeed1 = map(throttle_duration, 1000, 2000, 0, 255);  // 쓰로틀을 모터 속도로 변환
  int motorSpeed2 = map(steer_duration, 1000, 2000, 0, 255);     // 스티어링을 모터 속도로 변환

  // 속도를 범위 내로 제한
  motorSpeed1 = constrain(motorSpeed1, 0, 255);
  motorSpeed2 = constrain(motorSpeed2, 0, 255);

  // 모터에 PWM 신호 전달 (속도 제어)
  analogWrite(motor1Pin, motorSpeed1);
  analogWrite(motor2Pin, motorSpeed2);

  // 시리얼 모니터에 스티어링, 쓰로틀의 PWM 값 및 변환된 모터 속도 출력
  Serial.print("Steering Duration (HIGH): ");
  Serial.print(steer_duration);
  Serial.print(" | Steering Motor Speed: ");
  Serial.println(motorSpeed2);

  Serial.print("Throttle Duration (HIGH): ");
  Serial.print(throttle_duration);
  Serial.print(" | Throttle Motor Speed: ");
  Serial.println(motorSpeed1);
 
  delay(100);  // 100ms 지연
}
```