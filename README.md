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