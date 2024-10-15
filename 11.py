#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 스티어링 입력 핀 및 모터 핀 설정
int steer = 5;      // 스티어링 입력 (PWM 신호 입력 핀)
int throttle = 6;   // 쓰로틀 입력 (PWM 신호 입력 핀)

unsigned long steer_duration;
unsigned long throttle_duration;

int last_steer_pwm = 0;
int last_throttle_pwm = 0;
int tolerance = 5;  // 허용 오차 범위

void setup() {
  Serial.begin(9600);
  
  // PCA9685 초기화 및 주파수 설정
  pwm.begin();
  pwm.setPWMFreq(50);  // 서보 및 DC 모터 주파수 50Hz 설정
  
  // 스티어링 및 쓰로틀 입력 핀 설정
  pinMode(steer, INPUT);
  pinMode(throttle, INPUT);
}

void loop() {
  // 1. 스티어링 값 측정 및 서보 제어
  steer_duration = pulseIn(steer, HIGH);
  
  // 스티어링 값을 서보모터에 맞는 PWM 값으로 변환 (220 ~ 420)
  int steer_pwm = constrain(map(steer_duration, 1000, 2000, 220, 420), 220, 420);
  
  // 허용 오차보다 큰 변화가 있을 경우에만 업데이트
  if (abs(steer_pwm - last_steer_pwm) > tolerance) {
    pwm.setPWM(3, 0, steer_pwm);  // 서보모터 제어 (스티어링)
    last_steer_pwm = steer_pwm;   // 업데이트한 값을 저장
  }

  // 2. 쓰로틀 값 측정 및 모터 제어
  throttle_duration = pulseIn(throttle, HIGH);
  
  // 쓰로틀 값을 240~380 범위의 PWM 값으로 변환
  int throttle_pwm = constrain(map(throttle_duration, 1000, 2000, 240, 380), 240, 380);
  
  // 허용 오차보다 큰 변화가 있을 경우에만 업데이트
  if (abs(throttle_pwm - last_throttle_pwm) > tolerance) {
    pwm.setPWM(5, 0, throttle_pwm);  // TB6600 모터 속도 제어
    last_throttle_pwm = throttle_pwm; // 업데이트한 값을 저장
  }

  // 3. 시리얼 모니터에 디버깅 정보 출력
  Serial.print("Steering PWM Duration: ");
  Serial.print(steer_duration);
  Serial.print("\tSteering Servo PWM: ");
  Serial.println(steer_pwm);
  
  Serial.print("Throttle PWM Duration: ");
  Serial.print(throttle_duration);
  Serial.print("\tThrottle PWM: ");
  Serial.println(throttle_pwm);
  
  delay(100);
}