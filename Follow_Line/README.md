# 아두이노 코드 변경
* 스위치 코드 추가
* 센서 코드 추가

# 젯슨나노 코드 변경
* 아두이노에서 읽어오는 방법 변경 (index 라인 추가)
* follow_line 함수 추가
    - 맵핑함수 활용
* 자동 및 수동 모드 추가
    - 읽어오는 스위치가 변경될떄의 값을 읽어와 자동 및 수동 모드 변경
    * 에러사항
        - 수동에서 자동모드 변환시 값이 튀어 뒷바퀴만 굴러가는 상황 발생

# 1차 시도
* 센서의 위치 문제 (너무 앞쪽에 센서를 설치하여 우회전 좌회전시 미리 움직여 라인 이탈 발생)
* 좌회전은 잘 작동되지만 우회전이 라인 이탈 발생 (직진이 정확하지 X, 판단됨)