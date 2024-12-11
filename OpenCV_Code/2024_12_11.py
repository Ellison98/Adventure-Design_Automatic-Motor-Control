import cv2
import numpy as np
import math

# ROI 및 원근 변환 설정

def perspective_transform(frame):
    """원근 변환을 통해 차선 모양을 교정"""
    height, width = frame.shape[:2]
    src = np.float32([
        [width * 0.2, height * 0.8],  # 왼쪽 하단
        [width * 0.4, height * 0.5],  # 왼쪽 상단
        [width * 0.6, height * 0.5],  # 오른쪽 상단
        [width * 0.8, height * 0.8]   # 오른쪽 하단
    ])
    dst = np.float32([
        [width * 0.3, height],        # 변환 후 왼쪽 하단
        [width * 0.3, 0],            # 변환 후 왼쪽 상단
        [width * 0.7, 0],            # 변환 후 오른쪽 상단
        [width * 0.7, height]        # 변환 후 오른쪽 하단
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(frame, M, (width, height))
    return warped

def region_of_interest(frame, vertices):
    """관심 영역 설정"""
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(frame, mask)

# 차선 검출 및 중심선 계산

def extend_line(line, width, height):
    """차선을 연장하여 모이도록 보완"""
    if line is None:
        return None
    x1, y1, x2, y2 = line
    slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
    intercept = y1 - slope * x1

    y1_new = height
    y2_new = int(height * 0.6)
    x1_new = int((y1_new - intercept) / slope)
    x2_new = int((y2_new - intercept) / slope)

    return (x1_new, y1_new, x2_new, y2_new)

def calculate_center_line(left_line, right_line, width, height):
    """평행한 차선을 고려하여 중심선 계산"""
    if left_line is None and right_line is None:
        return None
    elif left_line is None:
        return extend_line(right_line, width, height)
    elif right_line is None:
        return extend_line(left_line, width, height)

    x1_left, y1_left, x2_left, y2_left = left_line
    x1_right, y1_right, x2_right, y2_right = right_line

    x1_center = (x1_left + x1_right) // 2
    y1_center = (y1_left + y1_right) // 2
    x2_center = (x2_left + x2_right) // 2
    y2_center = (y2_left + y2_right) // 2

    return (x1_center, y1_center, x2_center, y2_center)

# 디버깅 및 시각화

def draw_debug_info(frame, left_line, right_line, center_line):
    """디버깅 정보를 화면에 표시"""
    if left_line is not None:
        cv2.line(frame, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 255, 0), 2)
    if right_line is not None:
        cv2.line(frame, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 255, 0), 2)
    if center_line is not None:
        cv2.line(frame, (center_line[0], center_line[1]), (center_line[2], center_line[3]), (255, 0, 0), 3)

# 메인 프로세스

def follow_line_using_opencv(frame):
    """OpenCV를 이용해 라인을 따라 움직이는 함수"""
    height, width = frame.shape[:2]

    # 1. 원근 변환
    transformed_frame = perspective_transform(frame)

    # 2. 그레이스케일 및 블러 처리
    gray_img = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

    # 3. Canny 엣지 검출
    canny_img = cv2.Canny(blur_img, 50, 150)

    # 4. 관심 영역 설정
    vertices = np.array([[
        (width * 0.1, height),
        (width * 0.4, height * 0.6),
        (width * 0.6, height * 0.6),
        (width * 0.9, height)
    ]], dtype=np.int32)
    roi_img = region_of_interest(canny_img, vertices)

    # 5. 허프 변환을 통한 라인 검출
    lines = cv2.HoughLinesP(roi_img, 1, np.pi / 180, 30, minLineLength=20, maxLineGap=5)

    left_line, right_line = None, None
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                if slope < 0:  # 왼쪽 차선
                    left_line = extend_line((x1, y1, x2, y2), width, height)
                elif slope > 0:  # 오른쪽 차선
                    right_line = extend_line((x1, y1, x2, y2), width, height)

    # 6. 중심선 계산
    center_line = calculate_center_line(left_line, right_line, width, height)

    # 7. 디버깅 정보 시각화
    draw_debug_info(frame, left_line, right_line, center_line)

    return frame

# 카메라 입력 및 실행

def main():
    cap = cv2.VideoCapture(0)  # 기본 카메라 입력
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        result_frame = follow_line_using_opencv(frame)

        cv2.imshow("Lane Detection", result_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
