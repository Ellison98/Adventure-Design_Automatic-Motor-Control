import cv2
import numpy as np

def grayscale(frame):
    return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

def gaussian_blur(frame, kernel_size):
    return cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)

def canny(frame, low_threshold, high_threshold):
    return cv2.Canny(frame, low_threshold, high_threshold)

def region_of_interest(frame, vertices, color3=(255, 255, 255), color1=255):
    mask = np.zeros_like(frame)
    color = color3 if len(frame.shape) > 2 else color1
    cv2.fillPoly(mask, vertices, color)
    return cv2.bitwise_and(frame, mask)

def hough_lines(frame, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(frame, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
    if lines is not None:
        draw_lines(line_img, lines)
    return line_img

def draw_lines(frame, lines, color=[0, 0, 255], thickness=2):
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(frame, (x1, y1), (x2, y2), color, thickness)

def weighted_frame(frame, initial_frame, alpha=1., beta=1., gamma=0.):
    return cv2.addWeighted(initial_frame, alpha, frame, beta, gamma)

def calcaulate_slope_intercept(line):
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
            slope, intercept = calcaulate_slope_intercept((x1, y1, x2, y2))
            if slope is None:
                continue
            if 0.5 < abs(slope) < 2:
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

def draw_lane_lines(frame, lines, color = (0, 255, 0), thickness = 5):
    """Draw the lane lines on the frame."""
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
        print("Failed to read frame. check the camera")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("failed to read frame, Check the camera.")
            break

        frame = cv2.resize(frame, (640, 480))
        height, width = frame.shape[:2]

        gray_img = grayscale(frame)
        blur_img = gaussian_blur(gray_img, 3)
        canny_img = canny(blur_img, 50, 150) 

        vertices = np.array([[
            (width * 0.1, height),
            (width * 0.2, height * 0.5),
            (width * 0.8, height * 0.5),
            (width * 0.9, height)
        ]], dtype=np.int32)

        ROI_img = region_of_interest(canny_img, vertices)
        lines = cv2.HoughLinesP(ROI_img, 1, np.pi/180, threshold=30, minLineLength=20, maxLineGap=2)
        if lines is not None:
            left_line, right_line = filter_and_average_line(lines, width, height)
            center_line = calculate_center_line(left_line, right_line)

            draw_lane_lines(frame, [left_line, right_line])
            draw_lane_lines(frame, [center_line], color = (255, 0 ,0), thickness=3)

        cv2.imshow("ROI_img", ROI_img)
        cv2.imshow("Lane Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()