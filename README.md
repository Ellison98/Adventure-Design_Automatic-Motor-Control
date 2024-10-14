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
