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