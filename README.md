# TEST

git clone https://github.com/adafruit/Adafruit_CircuitPython_BusIO.git
cd Adafruit_CircuitPython_BusIO
python3 setup.py install


<br>


pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-busdevice


<br>


pip3 uninstall adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-pca9685



<br>

```python
def load_data():
    data = arduino.readline()
    print(f"Received data: {data}")  # 추가된 디버깅 출력
```

<br>

print(data_dict)  # 데이터 확인


<br>

```python
def load_data():
    data = arduino.readline()  # 바이트 배열로 읽기
    try:
        data = data.decode('utf-8').strip()  # UTF-8로 디코딩
    except UnicodeDecodeError:
        print("Failed to decode data. Raw bytes:", data)  # 디코딩 실패 시 원본 바이트 출력
        return {}

    if not data:  # 만약 데이터가 비어있다면
        return {}

    data = data.split(' ')  # 공백으로 분할
    data = [for_a.split(':') for for_a in data if ':' in for_a]  # ':'이 포함된 항목만 처리
    return {for_a[0]: int(for_a[1]) for for_a in data if len(for_a) == 2}  # 딕셔너리 반환
```


<br>

```python
def load_data():
    data = arduino.readline()  # 바이트 배열로 읽기
    print(f"Raw data: {data}")  # 수신된 원본 데이터 출력
    try:
        data = data.decode('utf-8').strip()  # UTF-8로 디코딩
    except UnicodeDecodeError:
        print(f"Failed to decode data: {data}")  # 디코딩 실패 시 원본 바이트 출력
        return {}

    if not data:  # 만약 데이터가 비어있다면
        return {}

    data = data.split(' ')  # 공백으로 분할
    print(f"Split data: {data}")  # 분할된 데이터 출력
    data = [for_a.split(':') for for_a in data if ':' in for_a]  # ':'이 포함된 항목만 처리
    return {for_a[0]: int(for_a[1]) for for_a in data if len(for_a) == 2}  # 딕셔너리 반환
```


<br>

```python
const int joystickX = A0; // 조종기의 X축 아날로그 핀
const int joystickY = A1; // 조종기의 Y축 아날로그 핀
const int threshold = 100; // 조종기 감지 임계값

void setup() {
    Serial.begin(9600);
}

void loop() {
    int xValue = analogRead(joystickX); // X축 값 읽기
    int yValue = analogRead(joystickY); // Y축 값 읽기

    // 조종기가 움직였는지 확인
    bool joystickMoved = (abs(xValue - 512) > threshold) || (abs(yValue - 512) > threshold);

    // 조종기의 상태에 따라 전송할 데이터 형식 변경
    if (joystickMoved) {  // 조종기가 움직였을 때
        Serial.print("0:");
        Serial.print(xValue); // X축 값 전송
        Serial.print(" 1:");
        Serial.print(yValue); // Y축 값 전송
        Serial.print(" 2:150\n");  // 추가 데이터 전송
    } else {
        Serial.print("0:0 1:0 2:0\n");  // 조종기가 움직이지 않을 때
    }
    delay(100);  // 전송 주기 조정
}
```

<br>

```python
def load_data():
    data = arduino.readline()  # 바이트 배열로 읽기
    print(f"Raw data: {data}")  # 수신된 원본 데이터 출력
    try:
        data = data.decode('utf-8').strip()  # UTF-8로 디코딩
    except UnicodeDecodeError:
        print(f"Failed to decode data: {data}")  # 디코딩 실패 시 원본 바이트 출력
        return {}

    if not data:  # 만약 데이터가 비어있다면
        return {}

    data = data.split(' ')  # 공백으로 분할
    print(f"Split data: {data}")  # 분할된 데이터 출력
    data = [for_a.split(':') for for_a in data if ':' in for_a]  # ':'이 포함된 항목만 처리
    
    # 유효한 데이터만 필터링
    result = {}
    for for_a in data:
        if len(for_a) == 2 and for_a[1].isdigit():  # 두 개의 항목이 있고, 두 번째 항목이 숫자인지 확인
            result[for_a[0]] = int(for_a[1])
        else:
            print(f"Invalid data: {for_a}")  # 유효하지 않은 데이터 출력

    return result  # 딕셔너리 반환
```