import cv2
import numpy as np

class LaneDetection:

    threshold_num = 5
    threshold_left = False
    threshold_right = False
    iterate_count = 0
    iterate_num = 10
    threshold_count = 0
    left_servo = False
    right_servo = False
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
    left_end_x = 0
    right_end_x = 0
    
    def lane_detection(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        edges = cv2.Canny(yellow_mask, threshold1=50, threshold2=100, apertureSize=3)
        mask = np.zeros_like(edges)
        region_of_interest_vertices = np.array([[0, 640], [0, 480], [640, 480], [480, 640]], dtype=np.int32)
        cv2.fillPoly(mask, [region_of_interest_vertices], 255)
        masked_image = cv2.bitwise_and(edges, mask)
        lines = cv2.HoughLinesP(masked_image, rho=1, theta=np.pi / 180, threshold=10, minLineLength=100, maxLineGap=100)

        left_lines = []
        right_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 == x2:
                    continue
                elif abs(x2 - x1) < abs(y2 - y1) + 5:
                    # cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=5)
                    slope = (y2 - y1) / (x2 - x1)
                    if slope < 0:
                        left_lines.append(line)  # (0, 0) is upper-left corner of the image
                    else:
                        right_lines.append(line)
        line_image = np.zeros_like(frame)
        left_xs = []
        left_ys = []
        right_xs = []
        right_ys = []

        for line in left_lines:
            x1, y1, x2, y2 = line[0]
            left_xs.extend([x1, x2])
            left_ys.extend([y1, y2])

        for line in right_lines:
            x1, y1, x2, y2 = line[0]
            right_xs.extend([x1, x2])
            right_ys.extend([y1, y2])

        if left_xs and left_ys and right_xs and right_ys:  # 탐지된 선이 있다면 if문 안 실행
            left_coeffs = np.polyfit(left_xs, left_ys, deg=1, rcond=1e-6)  # polyfit으로 왼쪽 선 구하기
            right_coeffs = np.polyfit(right_xs, right_ys, deg=1, rcond=1e-6)  # polyfit으로 오른쪽 선 구하기

            left_start_y = frame.shape[0]
            left_start_x = int((left_start_y - left_coeffs[1]) / left_coeffs[0])  # 0은 기울기, 1은 절편
            left_end_y = int(frame.shape[0] * 0.6)  # 1 - 0.6 = 0.4 화면 height의 0.4 길이 만큼 선 그리기
            LaneDetection.left_end_x = int((left_end_y - left_coeffs[1]) / left_coeffs[0])

            right_start_y = frame.shape[0]
            right_start_x = int((right_start_y - right_coeffs[1]) / right_coeffs[0])
            right_end_y = int(frame.shape[0] * 0.6)
            LaneDetection.right_end_x = int((right_end_y - right_coeffs[1]) / right_coeffs[0])
            center_start_x = int((left_start_x + right_start_x) / 2)
            center_end_x = int((LaneDetection.left_end_x + LaneDetection.right_end_x) / 2)
            cv2.line(line_image, (left_start_x, left_start_y), (LaneDetection.left_end_x, left_end_y), (0, 255, 0), thickness=5)
            cv2.line(line_image, (right_start_x, right_start_y), (LaneDetection.right_end_x, right_end_y), (0, 255, 0), thickness=5)
            cv2.line(line_image, (center_start_x, frame.shape[0]), (center_end_x, int(frame.shape[0] * 0.6)),
                     (0, 0, 255), thickness=5)



    def lane_keeping(self,frame):

        iterate_count += 1
        if LaneDetection.left_end_x > 480 - LaneDetection.right_end_x and (
                threshold_right == False):  # 왼쪽으로 5도 이상 치우친 경우

            if threshold_left:  # 처음이 아닌 경우, 즉 count 중이라면
                threshold_count += 1
            else:  # 처음인 경우
                threshold_left = True
                threshold_count = 1
                iterate_count = 1
        elif (LaneDetection.left_end_x < 480 - LaneDetection.right_end_x) and (
                threshold_left == False):  # 오른쪽으로 5도 이상 치우친 경우
            if threshold_right:
                threshold_count += 1
            else:
                threshold_right = True
                threshold_count = 1
                iterate_count = 1

        if iterate_count == iterate_num:
            if threshold_left:
                if threshold_count >= threshold_num:  # 10번 중 8번 이상이라면...
                    right_servo = True  # 차량을 오른쪽으로 가게 만들어라 flag
                    # print("Turn right")
                    # print("Turn right")
                    # print("Turn right")
                    # print("Turn right")
                    # print("Turn right")
                threshold_left = False
            elif threshold_right:
                if threshold_count >= threshold_num:  # 10번 중 8번 이상이라면...
                    left_servo = True  # 차량을 왼쪽으로 가게 만들어라 flag
                    # print("---------------Turn left---------------")
                    # print("---------------Turn left---------------")
                    # print("---------------Turn left---------------")
                    # print("---------------Turn left---------------")
                    # print("---------------Turn left---------------")
                threshold_right = False
        return right_servo,left_servo