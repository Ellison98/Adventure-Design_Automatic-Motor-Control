import cv2
import numpy as np
import serial
import time
import busio
from adafruit_pca9685 import PCA9685
from board import SCL, SDA

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

pca.frequency = 60

pca.channels[0].duty_cycle = 0x15a4 #center
pca.channels[2].duty_cycle = 0x17ed #stop

angle = 5540 #5448
speed = 6125 #6144
s_speed = 6362 # 스피드의 기본값6335


def map_value(value):
    return int ((value / 15865) * 65535) #15865 = H + L 65535

def slow_move(current_value, target_value, step = 50):
    if current_value < target_value:
        return min(current_value + step, target_value)
    if current_value > target_value:
        return max(current_value - step, target_value)
    return current_value    


def calculate_sloope_intercept(line):
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
            slope, intercept = calculate_sloope_intercept((x1, y1, x2, y2))
            if slope is None:
                continue
            if 0.4 < abs(slope) < 30:
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

def calculate_steering_angle(center_x, frame_width, max_steering_angle = 120):
    deviation = center_x - frame_width / 2
    steering_angle = np.clip(deviation / (frame_width / 2) * max_steering_angle, -max_steering_angle, max_steering_angle)

    if -16 <= steering_angle <= 0.5: # qkRna  0.5 ~ 2
        steering_angle = 0

    return steering_angle

def draw_reference_line(frame, height, width):
    start_point = (int(width * 0.5), int(height * 0.8))
    end_point = (int(width * 0.5), int(height * 0.8) + int(height * 0.2))
    cv2.line(frame, start_point, end_point, (0, 0, 255), 3)

def draw_lane_lines(frame, lines, color = (0, 255, 0), thickness = 5):
    """Draw the lane lines on the frame."""
    for line in lines:
        if line is not None:
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1, y1), (x2, y2), color, thickness)

def main():
    with serial.Serial('/dev/ttyACM0', 9600, timeout=1) as seri:
        # 시리얼 버퍼 비우기
        seri.reset_input_buffer()
        gst_pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
        )

        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("Can't open the camera.")
            return
    
        while True:
            content = seri.readline().decode(errors='ignore').strip()
            try:
                # 아두이노로부터 데이터 읽기
                # print("not split: ", content)
                values = content.split(',')
                # print("yes split: ", content)

                steer_value = int(values[0].strip())
                throttle_value = int(values[1].strip())
                change_mode = int(values[2].strip())
                L = int(values[3].strip())
                C = int(values[4].strip())
                R = int(values[5].strip())
                #print(f"L: {L}, C: {C}, R: {R}")
                #print(f"s: {steer_value}, t: {throttle_value}, m: {change_mode}, L: {L}, C: {C}, R: {R}")

                ret, frame = cap.read()
                if not ret:
                    print("no frame.")
                    break

                frame = cv2.resize(frame, (640,480))
                height, width = frame.shape[:2]

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5,5), 27)
                edges = cv2.Canny(blur, 50, 150)

                vertices = np.array([[
                    (width * 0, height),
                    (width * 0, height * 0.5),
                    (width * 1, height * 0.5),
                    (width * 1, height)
                ]], dtype = np.int32)
                roi = np.zeros_like(edges)
                cv2.fillPoly(roi, vertices, 255)
                masked_edges = cv2.bitwise_and(edges, roi)

                lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold = 30, minLineLength = 20, maxLineGap = 2)

                if lines is not None:
                    left_line, right_line = filter_and_average_line(lines, width, height)
                    center_line = calculate_center_line(left_line, right_line)

                    if center_line:
                        center_x = (center_line[0] + center_line[3]) // 2
                        steering_angle = calculate_steering_angle(center_x, width)

                        cv2.putText(frame, f'Steering Angle: {steering_angle: .2f} deg',
                                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                        if steering_angle > 0:
                            cv2.putText(frame, 'Steer Right', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                        elif steering_angle < 0:
                            cv2.putText(frame, 'Steer Left', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    
                        else:
                            cv2.putText(frame, 'Go Straight', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    


                    draw_lane_lines(frame, [left_line, right_line])
                    draw_lane_lines(frame, [center_line], color = (255, 0, 0), thickness = 3)

                draw_reference_line(frame, height, width)

                #cv2.imshow("Masked edges", masked_edges)
                cv2.imshow("Lane Detection", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
                if change_mode > 1400:
                    current_steer_value = pca.channels[0].duty_cycle
                    target_steer_value = map_value(steer_value)
                    pca.channels[0].duty_cycle = slow_move(current_steer_value, target_steer_value)
                    #print(slow_move(current_steer_value, target_steer_value))

                    current_throttle_value = pca.channels[2].duty_cycle
                    target_throttle_value = map_value(throttle_value)
                    pca.channels[2].duty_cycle = slow_move(current_throttle_value, target_throttle_value)
                    #print(slow_move(current_throttle_value, target_throttle_value))

                    #print(f"steer_mapping: {pca.channels[0].duty_cycle}, throttle_mapping: {pca.channels[2].duty_cycle}")
                else:  #stop 5540 , right 4480, left 7200
                    if lines is None:
                        print("No lines detected")
                        speed = s_speed
                    elif left_line is None:
                        print("left")
                        angle = 6080 # 6000
                        speed = s_speed + 3
                    elif right_line is None:
                        print("right")
                        angle = 4850
                        speed = s_speed + 3
                    elif steering_angle == 0:
                        angle = 5540
                        speed = s_speed - 1
                    elif steering_angle > 0:
                        angle = 4900
                        speed = s_speed + 3
                    elif steering_angle < 0:
                        angle = 6080
                        speed = s_speed + 3
                    else:
                        speed = s_speed + 3

                    pca.channels[0].duty_cycle = int(angle)
                    pca.channels[2].duty_cycle = int(speed)


            except (ValueError, IndexError) as e:
                print(f"Error processing values: {e}")
                continue
        

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()