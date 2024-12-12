import cv2
import numpy as np
import serial
from adafruit_pca9685 import PCA9685
import board
import busio
import time

# Serial 포트 설정
ser = serial.Serial('/dev/ttyACM0', 9600)

# PCA9685 설정
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 60

PWM_MIN = 1500
PWM_MAX = 12000

# 스티어링과 쓰로틀의 입력 범위 (예: 아두이노에서 오는 값)
STEER_MIN = 800
STEER_MAX = 2100
THROTTLE_MIN = 800
THROTTLE_MAX = 2100

# 스티어링 및 스로틀 값 
stop = 5898
forward = 6123  # 천천히 직진 
mid = 6000
right = 5000 # right 4513 left 7742
left = 6125
right_max = 5100
left_max = 6225

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
    pca.channels[2].duty_cycle = throttle_pwm

def region_of_interest(frame, vertices, color3=(255, 255, 255), color1=255):
    mask = np.zeros_like(frame)
    color = color3 if len(frame.shape) > 2 else color1
    cv2.fillPoly(mask, vertices, color)
    return cv2.bitwise_and(frame, mask)

def calculate_slope_intercept(line):
    x1, y1, x2, y2 = line
    if x2 - x1 == 0:
        return None, None
    slope = (y2 - y1) / (x2 - x1)                                                                                                                                                                     
    intercept = y1 - slope * x1
    return slope, intercept

def filter_and_average_line(lines, width, height):
    left_lines = []
    right_lines = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope, intercept = calculate_slope_intercept((x1, y1, x2, y2))
            if slope is None:
                continue
            if slope < 0:
                left_lines.append((slope, intercept))
            elif slope > 0:
                right_lines.append((slope, intercept))

    def average_lines(lines):
        if not lines:
            return None
        slope = np.mean([line[0] for line in lines])
        intercept = np.mean([line[1] for line in lines])
        return slope, intercept

    left_avg = average_lines(left_lines)
    right_avg = average_lines(right_lines)

    def calculate_line_coordinate(slope, intercept):
        if slope is None or intercept is None:
            return None
        y1 = height

        y2 = int((height * 0.6))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        return (x1, y1, x2, y2)

    left_line = calculate_line_coordinate(*left_avg) if left_avg else None
    right_line = calculate_line_coordinate(*right_avg) if right_avg else None
    return left_line, right_line

def calculate_center_line(left_line, right_line):
    if left_line is None or right_line is None:
        return None
    x1_left, y1_left, x2_left, y2_left = left_line
    x1_right, y1_right, x2_right, y2_right = right_line
    x1_center = (x1_left + x1_right) // 2
    y1_center = (y1_left + y1_right) // 2
    x2_center = (x2_left + x2_right) // 2
    y2_center = (y2_left + y2_right) // 2
    return (x1_center, y1_center, x2_center, y2_center)

def calculate_steering_angle(center_x, frame_width, max_steering_angle=30):
    deviation = center_x - frame_width / 2
    steering_angle = np.clip(deviation / (frame_width/2) * max_steering_angle, -max_steering_angle, max_steering_angle)
    if -3 <= steering_angle <= 3:
        steering_angle = 0
    return steering_angle

def draw_reference_line(frame, height, width):
    start_point = (int(width*0.5), int(height*0.8))
    end_point = (int(width*0.5), int(height*0.8)+int(height*0.2))
    cv2.line(frame, start_point, end_point, (0,0,255), 3)

def draw_lane_lines(frame, lines, color=(0, 255, 0), thickness=5):
    for line in lines:
        if line is not None:
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1, y1), (x2, y2), color, thickness)

def main():
    gst_pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame.")
            break

        frame = cv2.resize(frame, (640, 480))
        height, width = frame.shape[:2]
        gray_img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blur_img = cv2.GaussianBlur(gray_img, (5, 5), 3)
        canny_img = cv2.Canny(blur_img, 50, 150)

        vertices = np.array([[
            (width * 0.05, height),
            (width * 0.1, height * 0.7),
            (width * 0.9, height * 0.7),
            (width * 0.95, height)
        ]], dtype=np.int32)

        roi_img = region_of_interest(canny_img, vertices)
        lines = cv2.HoughLinesP(roi_img, 1, np.pi/180, threshold=30, minLineLength=20, maxLineGap=5)

        if lines is not None:
            left_line, right_line = filter_and_average_line(lines, width, height)
            center_line = calculate_center_line(left_line, right_line)
            if center_line:
                center_x = (center_line[0] + center_line[2]) // 2
                steering_angle = calculate_steering_angle(center_x, width)
                cv2.putText(frame,f'Steering Angle: {steering_angle:.2f} deg',
                            (50,50),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
                if steering_angle > 0:
                    cv2.putText(frame, 'Steer Left', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    pca.channels[2].duty_cycle = forward+1
                    pca.channels[0].duty_cycle = left
                elif steering_angle < 0:
                    cv2.putText(frame, 'Steer Right', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    pca.channels[2].duty_cycle = forward+1
                    pca.channels[0].duty_cycle = right
                else:
                    cv2.putText(frame, 'Go Straight', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    pca.channels[2].duty_cycle = forward
                    pca.channels[0].duty_cycle = 6200
            elif left_line:
                cv2.putText(frame, 'Finding Right Lane', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                pca.channels[2].duty_cycle = forward+1
                pca.channels[0].duty_cycle = right_max

            elif right_line:
                cv2.putText(frame, 'Finding Left Lane', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                pca.channels[2].duty_cycle = forward+1
                pca.channels[0].duty_cycle = left_max

            else:
                cv2.putText(frame, 'Maintaining Current Action', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                pca.channels[2].duty_cycle = forward
                pca.channels[0].duty_cycle = 6153

            draw_lane_lines(frame, [left_line, right_line])
            draw_lane_lines(frame, [center_line], color=(255, 0, 0), thickness=3)

        draw_reference_line(frame, height, width)

        cv2.imshow("canny", canny_img)
        cv2.imshow("roi",roi_img)
        cv2.imshow("Lane Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("STart")
    pca.channels[2].duty_cycle = 5898
    time.sleep(5)
    print("c")
    pca.channels[2].duty_cycle = 6153
    time.sleep(2)
    main()