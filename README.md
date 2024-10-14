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
    data_dict = load_data();  // 시리얼에서 데이터를 읽어옴

    // 조종기 값 읽기
    int joystick_x_value = data_dict.get('0', 0); // 기본값 0
    int joystick_y_value = data_dict.get('1', 0); // 기본값 0

    // 조종기 값을 PWM 신호로 변환
    int motor_speed_x = map(joystick_x_value, 900, 1700, 0, 255);
    int motor_speed_y = map(joystick_y_value, 900, 1700, 0, 255);

    // 모터에 PWM 신호 전달
    analogWrite(motor1Pin, motor_speed_x);
    analogWrite(motor2Pin, motor_speed_y);

    // 디버깅 로그
    Serial.print("X Speed: ");
    Serial.println(motor_speed_x);
    Serial.print("Y Speed: ");
    Serial.println(motor_speed_y);

    delay(100);  // 간단한 지연
}

```