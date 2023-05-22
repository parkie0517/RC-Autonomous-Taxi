######################################################################
########################## Import libraries ##########################
######################################################################
import Car
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import Paths
import requests
import Roads
import serial
import time
import torch
import math
from pathlib import Path
from PIL import Image
from time import sleep
import warnings

######################################################################
############################ Flask Setup #############################
######################################################################
url_ReceiveCall = 'http://172.26.34.12:777/send_to_eungR'
url_SendCoord = 'http://172.26.34.12:777/coordinate'
# url_ReceiveCall = 'http://172.26.16.25:1004/send_to_eungR'
# url_SendCoord = 'http://172.26.16.25:1004/coordinate'

######################################################################
############################# Path Setup #############################
######################################################################
path = Paths.Paths()  # Create Path object

######################################################################
############################# Road Setup #############################
######################################################################
listRoads = Roads.Roads()  # Create Roads object

######################################################################
########################## Line Stop Setup ###########################
######################################################################
line_stop_y = 550  # const
line_stop_threshold = 3  # const
line_stop_num = 5  # const
line_stop_iterate = 0
line_stop_count = 0
line_stop_flag = False
y = 0
h = 0

######################################################################
############################# LKAS Setup #############################
######################################################################
# Mask setting
polygon_default = np.array([[(0, 420), (0, 640), (480, 640), (480, 420), (480 - 200, 200), (200, 200)]], dtype=np.int32)
polygon = None
left_start_x = None
left_start_y = None
left_end_x = None  # left_end_x will be used in planning section
left_end_y = None
right_start_x = None
right_start_y = None
right_end_x = None  # right_end_x will be used in planning section
right_start_y = None
midpoint=[]
lower_yellow = np.array([20, 100, 100], dtype=np.uint8)  # Lower threshold value for detecting yellow color
upper_yellow = np.array([30, 255, 255], dtype=np.uint8)  # Upper threshold value for detecting yellow color
lower_white = np.array([0, 0, 200], dtype=np.uint8)  # Lower threshold value for detecting white color
upper_white = np.array([255, 30, 255], dtype=np.uint8)  # Upper threshold value for detecting white color
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([90, 100, 100],dtype=np.uint8)
upper_blue = np.array([120, 255, 255],dtype=np.uint8)
threshold_num = 5
threshold_left = False
threshold_right = False
iterate_count = 0
iterate_num = 10
threshold_count = 0
left_servo = False
right_servo = False
change = False
flag_bluedot = False
threshold_time = 0.35
warnings.filterwarnings('ignore')
def convert_to_HSV(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV", hsv)
    return hsv


def detect_edges(frame):
    lower_yellow = np.array([20, 50, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # this mask will filter out everything but blue

    # detect edges
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    edges = cv2.Canny(mask, 60, 100)
    # cv2.imshow("edges", edges)
    return edges



def region_of_interest(edges):
    height, width = edges.shape  # extract the height and width of the edges frame
    mask = np.zeros_like(edges)  # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height * (8 / 8)),
        (0, height * (5 / 8)),
        (width, height * (5 / 8)),
        (width, height * (8 / 8)),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)  # fill the polygon with blue color
    cropped_edges = cv2.bitwise_and(edges, mask)
    # cv2.imshow("roi", cropped_edges)
    return cropped_edges


def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=2, maxLineGap=1)
    return line_segments

# special variables
left_fit_average = None
left_fit = None
right_fit_average = None
right_fit = None
def average_slope_intercept(frame, line_segments):
    lane_lines = []
    global left_fit_average
    global left_fit
    global right_fit_average
    global right_fit

    if line_segments is None:
        # print("no line segment detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1 / 3

    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                # print("skipping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane
    # all coordinate points are in pixels
    return lane_lines


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):  # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:

                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2:  # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0]  # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0]  # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:  # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = (x2 - x1) * .7
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:  # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    cv2.imshow("Taxi Camera", heading_image)
    return heading_image, x1,y1,x2,y2



######################################################################
############################ 2D Map Setup ############################
######################################################################
base_image = Image.open("./Image/Map.jpg")  # Open the larger image
overlay_image1 = Image.open("./Image/Taxi1.png")  # Open the smaller image
overlay_image2 = Image.open("./Image/Taxi2.png")  # Open the smaller image
overlay_image3 = Image.open("./Image/Taxi3.png")  # Open the smaller image
overlay_image4 = Image.open("./Image/Taxi4.png")  # Open the smaller image

######################################################################
########################## Droidcam Setup ############################
######################################################################
cam = cv2.VideoCapture(1)  # Connect to DroidCam client

######################################################################
######################### Bluetooth Setup ############################
######################################################################

print("---------- Bluetooth setup ----------")
bt_cmd = None
port = "COM8"  # Port that is used to send signals to a connected RC car
bluetooth = serial.Serial(port, 9600)  # Connect to a RC car
bluetooth.flushInput()  # Clear Bluetooth input buffer
print("---------- Bluetooth setup complete ----------")


######################################################################
############################# Time Setup #############################
######################################################################
startTime = None
endTime = None
servo_start_time = None
servo_end_time = None
time_to_flask = 1

######################################################################
############################ 2D Map Setup ############################
######################################################################
base_image = Image.open("./Image/Map.jpg")  # Open the larger image
overlay_image1 = Image.open("./Image/Taxi1.png")  # Open the smaller image
overlay_image2 = Image.open("./Image/Taxi2.png")  # Open the smaller image
overlay_image3 = Image.open("./Image/Taxi3.png")  # Open the smaller image
overlay_image4 = Image.open("./Image/Taxi4.png")  # Open the smaller image

######################################################################
########################### R-Mutax Setup ############################
######################################################################
# 2D map realated variables
initY = 930  # Initial Y coordinate of virtual Taxi
initX = 400  # Initial X coordinate of virtual Taxi
initD = 1  # Initial direction of Taxi
# Car system related variables
initM = 0  # Initial mode of Taxi. Do not change this value unless you are a developer
speed = 10  # Normal speed of Taxi. unit is (cm/s)
# Create taxi object
taxi = Car.Taxi()
taxi.setvmapYX(initY, initX)
taxi.setvcarYX(initY, initX)
taxi.setDirection(initD)
taxi.setMode(initM)

######################################################################
############################# V2V Setup ##############################
######################################################################
flag_IntersectionCar = False

######################################################################
########################### OD Flag Setup ############################
######################################################################
flag_StopLine = False
flag_Person = False
flag_StopSign = False
# flag_StopSign_first = False
# StopSign_count = 0


######################################################################
############################# YOLO Setup #############################
######################################################################

# print("---------- YOLO setup ----------")
# threshold_confidence = 0.7
# # yolo_iteration = 0
# model_path = str(Path.cwd() / "yolov5s.pt") # Load YOLOv5
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True) # Load pretrained weight from Github
# with open("coco.names", "r") as f:
#     classes = [line.strip() for line in f.readlines()]
# person_class_index = classes.index("person")
# colors = np.random.uniform(0, 255, size=(len(classes), 3)) # Initialize random colors for detected bounding boxes
# print("---------- YOLO setup complete ----------")


######################################################################
########################### Begin R-Mutax ############################
######################################################################
print("System: Begin R-Mutax")
while True:
    mode = taxi.getMode()  # Get taxi mode
    ############################################################################
    ################################## mode 0 ##################################
    ############################################################################
    if mode == 0:
        print("System: Mode 0 started")
        # Tell R-Mutax to move from parking lot to road L
        bt_cmd = 'm'
        result_bytes = bt_cmd.encode('utf_8')
        # bt# bluetooth.write(result_bytes)
        ETA = 5  # Assumed that it will take 5 seconds for car to complete moving
        startTime = time.time()
        while True:
            # Show taxi front view
            ret, frame = cam.read()
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow('Taxi Camera', frame)
            # Show map
            tempY, tempX = taxi.getvcarYX()
            canvas = Image.new("RGBA", base_image.size, (0, 0, 0, 0))  # Create canvas
            canvas.paste(base_image, (0, 0))  # Paste base image
            tempDir = taxi.getDirection()
            if tempDir == 1:
                canvas.paste(overlay_image1, (tempX, tempY), overlay_image1)  # Paste overlaying image
            elif tempDir == 2:
                canvas.paste(overlay_image2, (tempX, tempY), overlay_image2)
            elif tempDir == 3:
                canvas.paste(overlay_image3, (tempX, tempY), overlay_image3)
            elif tempDir == 4:
                canvas.paste(overlay_image4, (tempX, tempY), overlay_image4)
            np_image = np.array(canvas)  # Convert PIL to np
            cv_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR
            cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Map', 800, 800)
            cv2.imshow('Map', cv_image)  # Show map
            if cv2.waitKey(1) == ord('q'):
                break
            endTime = time.time()
            elapsedTime = endTime - startTime
            # 2D Map YX change
            taxi.setvcarY(int(initY - ((initY - 760) * (elapsedTime / 5))))
            if elapsedTime > ETA:
                break
        print('now on the middle of road L')
        # Setting for mode 2
        # Taxi sysetm related setup
        taxi.setMode(2)
        taxi.setState(1)  # car state: stop state (0->0)
        # Localization related setup
        infoRY, infoRX, infoRD, infoRN, info_real_road_Len = listRoads.getRealRoads(11)
        infoRX = infoRX + int(info_real_road_Len / 2)  # parking에서 도로 l로 나왔을 때 처음 x 좌표
        taxi.setrmapYX(infoRY, infoRX)
        taxi.setrcarYX(infoRY, infoRX)
        # 2D map related setup
        infoVY, infoVX, infoVD, infoVN, info_virtual_road_Len = listRoads.getVirtualRoads(11)
        infoVX = infoVX + int(info_virtual_road_Len / 2)  # parking에서 도로 l로 나왔을 때 처음 x 좌표 380, 225
        taxi.setvmapYX(infoVY, infoVX)
        taxi.setvcarYX(infoVY, infoVX)
        taxi.setDirection(infoVD)
        taxi.setDistance(info_real_road_Len / 2)  # Objective distance
        taxi.setDT(0)  # DT = distance travelled
        taxi.setRoad('L')  # Road name is 'L'
        # LKAS setup
        polygon = polygon_default
        print("---------- Mode 0 ended ----------")
    ############################################################################
    ################################## mode 1 ##################################
    ############################################################################
    elif mode == 1:
        pass
        # new_road = True
        # stop_road = False
        # starting = taxi.getStarting()
        # destination = taxi.getDestination()
        #
        # current_road = taxi.getRoad()
        # cTOs = path.GPP(current_road, starting)  # Current road to Starting
        # # length_cTOs = len(cTOs)
        #
        # sTOd = path.GPP(starting, destination)
        # # length_sTOd = len(sTOd)
        #
        # cur_path = cTOs
        # # len_cur_path = length_cTOs
        #
        # time_to_flask = 1
        #
        # print("---------- Mode 1 started (part 1) ----------")
        # # Begin mode 1 part 1
        # while True:
        #     ######################################################################
        #     ############################# Perception #############################
        #     ######################################################################
        #     # Get frame from camera
        #     ret, frame = cam.read()  # Read frame
        #     frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)  # Rotate frame 90 degrees clockwise
        #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Change BGR -> HSV
        #
        #     # Object detection (using YOLOv5)
        #     # yolov_results = model(frame) # Store result in 'yolov_results'
        #
        #     # Stop Line Detecting
        #     red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
        #     boxes, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #
        #     # line_stop_iterate +=1
        #     y = 0
        #     h = 0
        #     for idx, box in enumerate(boxes):
        #         if idx == 0:
        #             continue
        #         x, y, w, h = cv2.boundingRect(box)
        #         if w > 100 and h < 50:  # 최소한의 넓이 조건을 충족하는 경우
        #             x, y, w, h = cv2.boundingRect(box)
        #             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        #             cv2.putText(frame, 'Stop Line', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        #             if y + h > 630:
        #                 flag_StopLine = True
        #             # if y >= line_stop_y:
        #             #     if line_stop_flag == False:
        #             #         line_stop_flag = True
        #             #         line_stop_iterate = 1
        #             #         line_stop_count = 1
        #             #     else:
        #             #        line_stop_count += 1
        #
        #     # LKAS
        #     yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # Create a yellow mask for detecing road lanes
        #     edges = cv2.Canny(yellow_mask, threshold1=50, threshold2=100,
        #                       apertureSize=3)  # Perform canny edge detection
        #     mask_roi = np.zeros_like(edges)
        #     cv2.fillPoly(mask_roi, polygon, 255)
        #     masked_edges = cv2.bitwise_and(edges, mask_roi)
        #     lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi / 180, threshold=20, minLineLength=50,
        #                             maxLineGap=100)  # Perform hough Transform
        #
        #     left_lines = []  # List used to store left lines that are detected
        #     right_lines = []  # List used to store right lines that are detected
        #     if lines is not None:
        #         for line in lines:
        #             x1, y1, x2, y2 = line[0]
        #             if abs(y1 - y2) < 5:  # y축과 거의 평행인 선
        #                 continue  # 무시하고 for문 다시 돌아라
        #             if x1 == x2:
        #                 continue
        #             slope = (y2 - y1) / (x2 - x1)
        #             if slope < 0:
        #                 left_lines.append(line)  # (0, 0) is upper-left corner of the image
        #             else:
        #                 right_lines.append(line)
        #
        #     left_xs = []
        #     left_ys = []
        #     right_xs = []
        #     right_ys = []
        #
        #     for line in left_lines:
        #         x1, y1, x2, y2 = line[0]
        #         left_xs.extend([x1, x2])
        #         left_ys.extend([y1, y2])
        #
        #     for line in right_lines:
        #         x1, y1, x2, y2 = line[0]
        #         right_xs.extend([x1, x2])
        #         right_ys.extend([y1, y2])
        #
        #     if left_xs and left_ys and right_xs and right_ys:  # 탐지된 선이 있다면 if문 안 실행
        #         left_coeffs = np.polyfit(left_xs, left_ys, deg=1, rcond=1e-6)  # polyfit으로 왼쪽 선 구하기
        #         right_coeffs = np.polyfit(right_xs, right_ys, deg=1, rcond=1e-6)  # polyfit으로 오른쪽 선 구하기
        #
        #         left_start_y = int(left_coeffs[1])
        #         left_start_x = 0  # 0은 기울기, 1은 절편
        #         left_end_y = int(frame.shape[0] * 0.4)  # 1 - 0.6 = 0.4 화면 height의 0.4 길이 만큼 선 그리기
        #         left_end_x = int((left_end_y - left_coeffs[1]) / left_coeffs[0])
        #
        #         right_start_y = int(480 * right_coeffs[0] + right_coeffs[1])
        #         right_start_x = 480
        #         right_end_y = int(frame.shape[0] * 0.4)
        #         right_end_x = int((right_end_y - right_coeffs[1]) / right_coeffs[0])
        #
        #         # Draw detected lines
        #         cv2.line(frame, (left_start_x, left_start_y), (left_end_x, left_end_y), (0, 255, 0), thickness=5)
        #         cv2.line(frame, (right_start_x, right_start_y), (right_end_x, right_end_y), (0, 255, 0), thickness=5)
        #
        #     ######################################################################
        #     ############################## Planning ##############################
        #     ######################################################################
        #     # Object detection
        #     class_ids = []
        #     confidences = []
        #     boxes = []
        #     # yolo_iteration += 1
        #     flag_StopSign = False
        #     """
        #     for yolov_result in yolov_results.xyxy[0]:
        #         label = classes[int(yolov_result[5])]
        #         confidence = yolov_result[4].item()
        #         if confidence > threshold_confidence:
        #             x, y, x1, y1 = map(int, yolov_result[:4]) # slicing. get 0~3 elements
        #             color = colors[int(yolov_result[5])]
        #             cv2.rectangle(frame, (x, y), (x1, y1), color, 2)
        #             cv2.putText(frame, f"{label}: {confidence:.2f}", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        #     """
        #     # Stop Line Detecting
        #     # if line_stop_iterate == 5 and line_stop_flag == True:
        #     #     if line_stop_count >= line_stop_threshold:
        #     #         flag_StopLine = True
        #     #     line_stop_flag = False
        #
        #     # LKAS
        #     lkas_iterate_count += 1
        #     if (left_end_x != None) and (right_end_x != None):  # If lane detection was performed, then....
        #         if (left_end_x > 500 - right_end_x) and (threshold_right == False):  # If the car is tilted left
        #             if threshold_left:  # 처음이 아닌 경우, 즉 count 중이라면
        #                 threshold_count += 1
        #             else:  # 처음인 경우
        #                 threshold_left = True
        #                 threshold_count = 1
        #                 lkas_iterate_count = 1
        #         elif (left_end_x + 20 < 480 - right_end_x) and (threshold_left == False):  # If the car is tilted right
        #             if threshold_right:
        #                 threshold_count += 1
        #             else:
        #                 threshold_right = True
        #                 threshold_count = 1
        #                 lkas_iterate_count = 1
        #
        #     if lkas_iterate_count == iterate_num:  # Check if iterate count has reached iterate num
        #         if threshold_left:
        #             if threshold_count >= threshold_num:  # 10번 반복하면서 5번 이상 탐지 성공했을 경우
        #                 right_servo = True  # 차량을 오른쪽으로 가게 만들어라 flag
        #             threshold_left = False
        #         elif threshold_right:
        #             if threshold_count >= threshold_num:  # 10번 반복하면서 5번 이상 탐지 성공했을 경우
        #                 left_servo = True  # 차량을 왼쪽으로 가게 만들어라 flag
        #             threshold_right = False
        #     ###################################################################
        #     ################################# Check state #####################
        #     ###################################################################
        #     state = taxi.getState()
        #     if state == 1:  # Stop 상태
        #         if (flag_StopLine):  # 계속해서 멈춰있을 필요가 있는지 확인하기
        #             print("Planning: car is stopped and still needs to stop")
        #         else:  # 만약 계속 멈춰 있을 필요가 없다면 조건문 넣기
        #             taxi.setState(2)
        #             taxi.setSpeed(speed)
        #             print("Planning: car needs to go forward")
        #
        #         # 지금 달리는 도로가 staring, 또는 destination일 경우
        #         if new_road:
        #             new_road = False
        #             current_road = taxi.getRoad()
        #             if (current_road == starting) or (current_road == destination):
        #                 stop_road = True
        #     elif state == 2:  # 0 -> 1
        #         print("")
        #     elif state == 3:  # 1 -> 0
        #         taxi.setState(1)  # 차 세워
        #         # 세 가지 케이스를 생각해볼 수 있다.
        #         # case 1 앞 차 때문에 멈춘 상황
        #         # case 2 목적지 도달한 경우 (픽업지, 도착지, 주차장)
        #         # case 3 정지선 도달한 경우
        #     elif state == 4:  # 1 -> 1
        #         # RoI update
        #         if left_end_x:
        #             if h != 0:
        #                 polygon = np.array([[(0, left_start_y - 80), (0, 640), (480, 640), (480, right_start_y - 80),
        #                                      (right_end_x + 10, y + h - 10), (left_end_x - 10, y + h - 10)]],
        #                                    dtype=np.int32)
        #             else:
        #                 polygon = np.array([[(0, left_start_y - 80), (0, 640), (480, 640), (480, right_start_y - 80),
        #                                      (right_end_x + 10, right_end_y - 10), (left_end_x - 10, left_end_y - 10)]],
        #                                    dtype=np.int32)
        #
        #         # Localization 오차 보정도 나중에 추가하기
        #         if (flag_StopCar):  # If there is a car infront of the Taxi. Taxi has to stop immediately
        #             print('Planning: There is a car infront of the taxi')
        #             taxi.setState(3)
        #             taxi.setSpeed(0)
        #         elif (flag_StopLine):  # If there is a stop line.
        #             print('Planning: Stop line detected')
        #             flag_StopLine = False
        #             taxi.setState(5)
        #             taxi.setSpeed(0)
        #         # Localization
        #         endTime = time.time()
        #         elapsedTime = endTime - startTime
        #         DT = elapsedTime * taxi.getSpeed()
        #         taxi.setDT(DT)
        #         # update YX
        #         infoRY, infoRX, infoRD, infoRN, info_real_road_Len = listRoads.getRealRoads(ord(taxi.getRoad()) - 65)
        #         infoVY, infoVX, infoVD, infoVN, info_virtual_road_Len = listRoads.getVirtualRoads(
        #             ord(taxi.getRoad()) - 65)
        #         ratio = info_virtual_road_Len / info_real_road_Len
        #         tempDir = taxi.getDirection()
        #         if tempDir == 1:
        #             # real car
        #             temprcarY = int(taxi.getrmapY() - DT)
        #             taxi.setrcarY(temprcarY)
        #             # virtual car
        #             tempvcarY = int(taxi.getvmapY() - DT * ratio)
        #             taxi.setvcarY(tempvcarY)
        #         elif tempDir == 2:
        #             # real car
        #             temprcarX = int(taxi.getrmapX() + DT)
        #             taxi.setrcarX(temprcarX)
        #             # virtual car
        #             tempvcarX = int(taxi.getvmapX() + DT * ratio)
        #             taxi.setvcarX(tempvcarX)
        #         elif tempDir == 3:
        #             # real car
        #             temprcarY = int(taxi.getrmapY() + DT)
        #             taxi.setrcarY(temprcarY)
        #             # virtual car
        #             tempvcarY = int(taxi.getvmapY() + DT * ratio)
        #             taxi.setvcarY(tempvcarY)
        #         elif tempDir == 4:
        #             # real car
        #             temprcarX = int(taxi.getrmapX() - DT)
        #             taxi.setrcarX(temprcarX)
        #             # virtual car
        #             tempvcarX = int(taxi.getvmapX() - DT * ratio)
        #             taxi.setvcarX(tempvcarX)
        #
        #         # Send the coordinate of eungRitaxi to the Flask server every 1 second
        #         if elapsedTime > time_to_flask:
        #             time_to_flask += 1
        #             y, x = taxi.getvcarYX()
        #             d = taxi.getDirection()
        #             data = {'key1': x, 'key2': y, 'key3': d}
        #             response = requests.post(url_SendCoord, json=data)
        #
        #             if response.status_code == 200:
        #                 pass
        #             else:
        #                 print("Fucking error: " + response.text)
        #
        #         if (DT >= taxi.getDistance()):  # 만약 도달을 했다면
        #             taxi.setState(5)  # 차 세워 당장!flag_StopLine = True # 사실 이거는 요기 있으면 안되고, 나중에 OD에서 해줘야 되는거지만.실험을 위해 넣어둠
        #             print("Planning: Car is now at the stop line")
        #         if stop_road == True:
        #             if DT >= (taxi.getDistance() / 2):
        #                 # <<<요기>>> arduino stop code 넣기
        #                 sleep(5)  # sleep for 5 seconds
        #                 # <<<요기>>> arduino forward code 넣기
        #                 startTime += 5  # since the program slept for 5 sec, we need to add 5 to startime
        #                 stop_road = False
        #     elif state == 5:  # Taxi has arrived at the beginning of the intersection
        #         # Later, write code for checking if there are other cars that are on the intersection
        #         taxi.setState(6)
        #
        #         current_road = taxi.getRoad()
        #         if current_road == starting:
        #             cur_path = sTOd
        #
        #             nextRoad, nextRoadInt, nextRoadWay = path.nextRoad(current_road, cur_path)
        #             taxi.setNextRoad(nextRoad)  # Next road's alphabet name
        #             taxi.setNextRoadInt(nextRoadInt)  # Next road's number
        #             taxi.setNextRoadWay(nextRoadWay)  # which way to go to get to next road
        #         elif current_road == destination:
        #             # randRoad사용
        #             nextRoad, nextRoadInt, nextRoadWay = path.randRoad(current_road)
        #             taxi.setNextRoad(nextRoad)  # Next road's alphabet name
        #             taxi.setNextRoadInt(nextRoadInt)  # Next road's number
        #             taxi.setNextRoadWay(nextRoadWay)  # which way to go to get to next road
        #             # 그리고 주행 모드 2로 설정
        #             taxi.setMode(2)
        #             pass
        #         else:  # 아직 path에서 갈 길이 남은 경우
        #             nextRoad, nextRoadInt, nextRoadWay = path.nextRoad(current_road, cur_path)
        #             taxi.setNextRoad(nextRoad)  # Next road's alphabet name
        #             taxi.setNextRoadInt(nextRoadInt)  # Next road's number
        #             taxi.setNextRoadWay(nextRoadWay)  # which way to go to get to next road
        #     elif state == 6:  # Taxi is starting to drive in the intersection
        #         pass
        #     elif state == 7:  # Taxi has ended driving in intersection. Now back on road
        #         print('Planning: taxi has ended driving in the intersection')
        #         # Flag reset
        #         new_road = True
        #         # RoI reset
        #         polygon = polygon_default
        #         # Taxi sysetm related setup
        #         taxi.setState(1)
        #         # Localization related setup
        #         nextRoad = taxi.getNextRoad()
        #         taxi.setRoad(nextRoad)
        #         # Localization related setup
        #         infoRY, infoRX, infoRD, infoRN, info_real_road_Len = listRoads.getRealRoads(ord(nextRoad) - 65)
        #         taxi.setDistance(info_real_road_Len)
        #         taxi.setDT(0)
        #         taxi.setrmapYX(infoRY, infoRX)
        #         taxi.setrcarYX(infoRY, infoRX)
        #         # 2D map related setup
        #         infoVY, infoVX, infoVD, infoVN, info_virtual_road_Len = listRoads.getVirtualRoads(ord(nextRoad) - 65)
        #         taxi.setvmapYX(infoVY, infoVX)
        #         taxi.setvcarYX(infoVY, infoVX)
        #         taxi.setDirection(infoVD)
        #
        #         if taxi.getMode() != 1:  # Taxi state is 1, and mode is other than 2
        #             break
        #
        #     elif state == 8:  # Taxi is driving in the intersection
        #         endTime = time.time()
        #         elapsedTime = endTime - startTime
        #         nextRoadWay = taxi.getNextRoadWay()
        #         if (nextRoadWay == 'turnLeft'):  # If next way is left
        #             if elapsedTime > 5:
        #                 print('Planning: arrived at next road')
        #                 taxi.setState(7)
        #         elif (nextRoadWay == 'turnRight'):
        #             if elapsedTime > 5:
        #                 print('Planning: arrived at next road')
        #                 taxi.setState(7)
        #         elif (nextRoadWay == 'straight'):
        #             if elapsedTime > 5:
        #                 print('Planning: arrived at next road')
        #                 taxi.setState(7)
        #
        #     ######################################################################
        #     ############################## Control ###############################
        #     ######################################################################
        #     # 제어에서는, 시간 바꾸기, 제어 신호, State 변경만 가능
        #     state = taxi.getState()
        #     if state == 1:  # 차 세워
        #         taxi.setSpeed(0)
        #         taxi.stop()
        #         ########수정!!@#!@#!@#!@#@#
        #     elif state == 2:
        #         taxi.moveForward()
        #         startTime = time.time()
        #         time_to_flask = 1
        #         taxi.setState(4)
        #         print("Control:car moving forward in road", taxi.getRoad())
        #     elif state == 3:
        #         print("")
        #     elif state == 4:
        #         if left_servo:
        #             if change == False:  # 처음이라면
        #                 servo_start_time = time.time()
        #                 change = True
        #                 # make 응애 rc카 go left
        #                 print("turn left")
        #                 result = 'left'
        #                 result_bytes = result.encode('utf_8')
        #                 # bt# bluetooth.write(result_bytes)
        #             else:  # 처음이 아니라면
        #                 servo_end_time = time.time()
        #                 servo_elapsed_time = servo_end_time - servo_start_time
        #                 if servo_elapsed_time >= threshold_time:
        #                     # make 응애 rc카 go straight
        #                     result = 'setup'
        #                     result_bytes = result.encode('utf_8')
        #                     # bt# bluetooth.write(result_bytes)
        #                     left_servo = False
        #                     change = False
        #         elif right_servo:
        #             if change == False:  # 처음이라면
        #                 print("turn right")
        #                 servo_start_time = time.time()
        #                 change = True
        #                 # make 응애 rc카 go right
        #                 result = 'right'
        #                 result_bytes = result.encode('utf_8')
        #                 # bt# bluetooth.write(result_bytes)
        #             else:  # 처음이 아니라면
        #                 servo_end_time = time.time()
        #                 servo_elapsed_time = servo_end_time - servo_start_time
        #                 if servo_elapsed_time >= threshold_time:
        #                     # make 응애 rc카 go straight
        #                     result = 'setup'
        #                     result_bytes = result.encode('utf_8')
        #                     # bt# bluetooth.write(result_bytes)
        #                     right_servo = False
        #                     change = False
        #     elif state == 5:
        #         taxi.stop()
        #         print('Control: stopped the car before entering the intersection')
        #     elif state == 6:
        #         nextRoadWay = taxi.getNextRoadWay()
        #         if (nextRoadWay == 'turnLeft'):  # If next way is left
        #             taxi.turnLeft()
        #             print('Control: taxi is now turning left')
        #         elif (nextRoadWay == 'turnRight'):
        #             taxi.turnRight()
        #             print('Control: taxi is now turning right')
        #         elif (nextRoadWay == 'straight'):
        #             taxi.straight()
        #             print('Control: taxi is now going straight')
        #         taxi.setState(8)  # Taxi driving in intersection
        #         startTime = time.time()
        #         print('Control: taxi now going into intersection')
        #     elif state == 7:
        #         taxi.stop()
        #     elif state == 8:
        #         pass
        #
        #     ######################################################################
        #     ###################### Human-Machine Interface #######################
        #     ######################################################################
        #     # Show taxi camera with OD and LKAS results
        #     roi_mask_3ch = cv2.cvtColor(mask_roi, cv2.COLOR_GRAY2BGR)
        #     combined_result = cv2.addWeighted(frame, 0.8, roi_mask_3ch, 0.2, 0)  # Combine OD image and LKAS image
        #     cv2.imshow('Taxi Camera', combined_result)  # Show combined result
        #
        #     # Show 2D map
        #     tempY, tempX = taxi.getvcarYX()
        #     canvas = Image.new("RGBA", base_image.size, (0, 0, 0, 0))  # Create canvas
        #     canvas.paste(base_image, (0, 0))  # Paste base image
        #     tempDir = taxi.getDirection()
        #     if tempDir == 1:
        #         canvas.paste(overlay_image1, (tempX, tempY), overlay_image1)  # Paste overlaying image
        #     elif tempDir == 2:
        #         canvas.paste(overlay_image2, (tempX, tempY), overlay_image2)
        #     elif tempDir == 3:
        #         canvas.paste(overlay_image3, (tempX, tempY), overlay_image3)
        #     elif tempDir == 4:
        #         canvas.paste(overlay_image4, (tempX, tempY), overlay_image4)
        #     np_image = np.array(canvas)  # Convert PIL to np
        #     cv_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR
        #     cv2.namedWindow('Map', cv2.WINDOW_NORMAL)  # Give a specific name to the window
        #     cv2.resizeWindow('Map', 800, 800)  # Resize window size
        #     cv2.imshow('Map', cv_image)  # Show 2D map
        #
        #     if cv2.waitKey(1) == ord('q'):  # Press Q to close the window. But window will pop back up in next iteration
        #         break
        #
        # print("---------- Mode 1 ended (part 1)----------")

    ############################################################################
    ################################## mode 2 ##################################
    ############################################################################
    elif mode == 2:
        print("---------- Mode 2 started ----------")
        # Mode 2 initialization setting
        threshold_left = False
        threshold_right = False
        iterate_count = 0
        threshold_count = 0
        # Line Stop Setting
        line_stop_iterate = 0
        line_stop_count = 0
        line_stop_flag = False
        flag_bluedot=False
        # yolo_iteration = 0
        # flag_StopSign_first = False
        # StopSign_count = 0

        # Begin mode 2 while loop
        while True:
            ######################################################################
            ############################# Perception #############################
            ######################################################################
            state = taxi.getState()

            # Get frame from camera
            ret, frame = cam.read()  # Read frame
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)  # Rotate frame 90 degrees clockwise
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Change BGR -> HSV

            # blue_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Object detection (using YOLOv5)
            # yolov_results = model(frame) # Store result in 'yolov_results'

            # Stop Line Detecting
            red_mask = cv2.inRange(hsv, lower_red, upper_red) + cv2.inRange(hsv, lower_red2, upper_red2)
            boxes, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # line_stop_iterate +=1
            y = 0
            h = 0
            for idx, box in enumerate(boxes):
                if idx == 0:
                    continue
                x, y, w, h = cv2.boundingRect(box)
                if w > 100 :  # 최소한의 넓이 조건을 충족하는 경우
                    x, y, w, h = cv2.boundingRect(box)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, 'Stop Line', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    if y + h > 560 or y == 640:
                        flag_StopLine = True

                    # if y >= line_stop_y:
                    #     if line_stop_flag == False:
                    #         line_stop_flag = True
                    #         line_stop_iterate = 1
                    #         line_stop_count = 1
                    #     else:
                    #        line_stop_count += 1

            # LKAS
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  # Create a yellow mask for detecing road lanes
            blue_mask = cv2.inRange(hsv, lower_blue,upper_blue)

            # cv2.drawContours(blue_roi, blue_contours, -1, (0, 255, 0), 2)
            edges = detect_edges(hsv)
            # if check = 1, check whether there is a stop sig
            # functions for getting steering angle from from blue line detection
            roi = region_of_interest(edges)
            line_segments = detect_line_segments(roi)
            lane_lines = average_slope_intercept(frame, line_segments)
            lane_lines_image = display_lines(frame, lane_lines)
            steering_angle = (get_steering_angle(frame, lane_lines))
            # input steering angle into pid python file to and return steering PWM, derivative response, and proportional response

            # plot lane lines and steering angle
            heading_image, center_x1, center_y1, center_x2, center_y2 = display_heading_line(lane_lines_image,
                                                                                             steering_angle)
            blue_contours, blue_hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for blue_contour in blue_contours:
                # 검출된 contour에 외접하는 사각형의 좌표를 구합니다.
                x, y, w, h = cv2.boundingRect(blue_contour)

                # 외접하는 사각형을 그립니다.
                rectangles = []
                if y >= 320 and y <= 540 and w >= 10:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    if len(blue_contours) >= 2:
                        # 모든 contour의 모멘트를 계산하여 중점 좌표를 구합니다.
                        rect1 = cv2.boundingRect(blue_contours[0])
                        rect2 = cv2.boundingRect(blue_contours[1])
                        x1, y1, w1, h1 = rect1
                        x2, y2, w2, h2 = rect2
                        if ((x1 + x2) / 2) >= 200 and ((x1 + x2) / 2) <= 280 and state==8:
                            flag_bluedot = True

            # cv2.drawContours(blue_roi, blue_contours, -1, (0, 255, 0), 2)
            # if len(blue_contours) == 2:
            #     # 모든 contour의 모멘트를 계산하여 중점 좌표를 구합니다.
            #     centers = []
            #     for cnt in blue_contours:
            #         M = cv2.moments(cnt)
            #         if M["m00"] != 0:
            #             cX = int(M["m10"] / M["m00"])
            #             cY = int(M["m01"] / M["m00"])
            #             centers.append((cX, cY))
            #
            #     # 중점 좌표를 출력합니다.x
            #     # print("Center points of blue contours: ", centers)
            #     midpoint=[]
            #     if len(centers) == 2:
            #         x1, y1 = centers[0]
            #         x2, y2 = centers[1]
            #         midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)
            #         print(midpoint[0])

            ######################################################################
            ############################## Planning ##############################
            ######################################################################
            # Object detection
            class_ids = []
            confidences = []
            boxes = []
            # yolo_iteration += 1
            flag_StopSign = False
            flag_Person = False
            # person_results = yolov_results.xyxy[0][yolov_results.xyxy[0][:, 5] == person_class_index]
            # if person_results is not None:
            #     for person_result in person_results:
            #         label = classes[int(person_result[5])]
            #         confidence = person_result[4].item()
            #         if confidence > threshold_confidence:
            #             x, y, x1, y1 = map(int, person_result[:4]) # slicing. get 0~3 elements
            #             color = (0,255,0)
            #             cv2.rectangle(frame, (x, y), (x1, y1), color, 2)
            #             cv2.putText(frame, f"{label}: {confidence:.2f}", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            #             if y1 >= 400:
            #                 flag_Person = True

            # Stop Line Detecting
            # if line_stop_iterate == 5 and line_stop_flag == True:
            #     if line_stop_count >= line_stop_threshold:
            #         flag_StopLine = True
            #     line_stop_flag = False

            # LKAS
            iterate_count += 1
            if center_x2 > 260 and (
                    threshold_right == False):  # 왼쪽으로 5도 이상 치우친 경우

                if threshold_left:  # 처음이 아닌 경우, 즉 count 중이라면
                    threshold_count += 1
                else:  # 처음인 경우
                    threshold_left = True
                    threshold_count = 1
                    iterate_count = 1
            elif (center_x2 < 220) and (
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

                    threshold_left = False
                elif threshold_right:
                    if threshold_count >= threshold_num:  # 10번 중 8번 이상이라면...
                        left_servo = True  # 차량을 왼쪽으로 가게 만들어라 flag

                    threshold_right = False

            # Check state
            if state == 1:  # Stop 상태
                if flag_StopLine or flag_Person:  # 계속해서 멈춰있을 필요가 있는지 확인하기
                    print("Planning: car is stopped and still needs to stop")
                    pass
                else:  # 만약 계속 멈춰 있을 필요가 없다면 조건문 넣기
                    taxi.setState(2)
                    taxi.setSpeed(speed)
                    print("Planning: car needs to go forward")
            elif state == 2:  # 0 -> 1
                print("")
            elif state == 3:  # 1 -> 0
                taxi.setState(1)  # 차 세워
                # 세 가지 케이스를 생각해볼 수 있다.
                # case 1 앞 차 때문에 멈춘 상황
                # case 2 목적지 도달한 경우 (픽업지, 도착지, 주차장)
                # case 3 정지선 도달한 경우
            elif state == 4:  # 1 -> 1
                # RoI update

                # Localization 오차 보정도 나중에 추가하기
                if (flag_Person):  # If there is a car infront of the Taxi. Taxi has to stop immediately
                    print('Planning: There is a Person infront of the taxi')

                    taxi.setState(3)
                    taxi.setSpeed(0)
                elif (flag_StopLine):  # If there is a stop line.
                    print('Planning: Stop line detected')
                    flag_StopLine = False
                    taxi.setState(5)
                    taxi.setSpeed(0)
                # Localization
                endTime = time.time()
                elapsedTime = endTime - startTime
                DT = elapsedTime * taxi.getSpeed()
                taxi.setDT(DT)
                # update YX
                infoRY, infoRX, infoRD, infoRN, info_real_road_Len = listRoads.getRealRoads(ord(taxi.getRoad()) - 65)
                infoVY, infoVX, infoVD, infoVN, info_virtual_road_Len = listRoads.getVirtualRoads(
                    ord(taxi.getRoad()) - 65)
                ratio = info_virtual_road_Len / info_real_road_Len
                tempDir = taxi.getDirection()
                if tempDir == 1:
                    # real car
                    temprcarY = int(taxi.getrmapY() - DT)
                    taxi.setrcarY(temprcarY)
                    # virtual car
                    tempvcarY = int(taxi.getvmapY() - DT * ratio)
                    taxi.setvcarY(tempvcarY)
                elif tempDir == 2:
                    # real car
                    temprcarX = int(taxi.getrmapX() + DT)
                    taxi.setrcarX(temprcarX)
                    # virtual car
                    tempvcarX = int(taxi.getvmapX() + DT * ratio)
                    taxi.setvcarX(tempvcarX)
                elif tempDir == 3:
                    # real car
                    temprcarY = int(taxi.getrmapY() + DT)
                    taxi.setrcarY(temprcarY)
                    # virtual car
                    tempvcarY = int(taxi.getvmapY() + DT * ratio)
                    taxi.setvcarY(tempvcarY)
                elif tempDir == 4:
                    # real car
                    temprcarX = int(taxi.getrmapX() - DT)
                    taxi.setrcarX(temprcarX)
                    # virtual car
                    tempvcarX = int(taxi.getvmapX() - DT * ratio)
                    taxi.setvcarX(tempvcarX)

                # Send the coordinate of eungRitaxi to the Flask server every 1 second
                if elapsedTime > time_to_flask:
                    time_to_flask += 1
                    y, x = taxi.getvcarYX()
                    d = taxi.getDirection()
                    data = {'key1': x, 'key2': y, 'key3': d}
                    response = requests.post(url_SendCoord, json=data)
                    # print(data)
                    if response.status_code == 200:
                        pass
                    else:
                        print("Fucking error: " + response.text)

                # if (DT >= taxi.getDistance()):  # 만약 도달을 했다면
                #     taxi.setState(5)  # 차 세워 당장!flag_StopLine = True # 사실 이거는 요기 있으면 안되고, 나중에 OD에서 해줘야 되는거지만.실험을 위해 넣어둠
                #     print("Planning: Car is now at the stop line")

            elif state == 5:  # Taxi has arrived at the beginning of the intersection
                # Later, write code for checking if there are other cars that are on the intersection
                # if (flag_IntersectionCar == False):
                time.sleep(3)
                taxi.setState(6)

                # Check if there is a reservation
                response = requests.get(url_ReceiveCall)  # request data from Flask server
                str1 = response.text  # str1[0] is the starting node, str1[1] is the destination node

                if str1[0] != '0':  # if we got something from the Flask server
                    starting = str1[0]
                    destination = str1[1]
                    print("Call received. From " + starting + " to " + destination)
                    taxi.setStarting(starting)
                    taxi.setDestination(destination)
                    taxi.setMode(1)  # set Mode to 1 for autonomous RC taxi service

                    cTOs = path.GPP(taxi.getRoad(), starting)
                    nextRoad, nextRoadInt, nextRoadWay = path.nextRoad(taxi.getRoad(), cTOs)
                    taxi.setNextRoad(nextRoad)  # Next road's alphabet name
                    taxi.setNextRoadInt(nextRoadInt)  # Next road's number
                    taxi.setNextRoadWay(nextRoadWay)  # which way to go to get to next road
                elif str1 == '00':
                    print("nah keep goin")
                    nextRoad, nextRoadInt, nextRoadWay = path.randRoad(taxi.getRoad())
                    taxi.setNextRoad(nextRoad)  # Next road's alphabet name
                    taxi.setNextRoadInt(nextRoadInt)  # Next road's number
                    taxi.setNextRoadWay(nextRoadWay)  # which way to go to get to next road

            elif state == 6:  # Taxi is starting to drive in the intersection
                pass
            elif state == 7:  # Taxi has ended driving in intersection. Now back on road
                print('Planning: taxi has ended driving in the intersection')
                # RoI reset
                polygon = polygon_default
                # Taxi sysetm related setup
                flag_StopLine = False

                taxi.setState(1)

                # Localization related setup
                nextRoad = taxi.getNextRoad()
                taxi.setRoad(nextRoad)
                # Localization related setup
                infoRY, infoRX, infoRD, infoRN, info_real_road_Len = listRoads.getRealRoads(ord(nextRoad) - 65)
                taxi.setDistance(info_real_road_Len)
                taxi.setDT(0)
                taxi.setrmapYX(infoRY, infoRX)
                taxi.setrcarYX(infoRY, infoRX)
                # 2D map related setup
                infoVY, infoVX, infoVD, infoVN, info_virtual_road_Len = listRoads.getVirtualRoads(ord(nextRoad) - 65)
                taxi.setvmapYX(infoVY, infoVX)
                taxi.setvcarYX(infoVY, infoVX)
                taxi.setDirection(infoVD)

                if taxi.getMode() != 2:  # Taxi state is 1, and mode is other than 2
                    break

            elif state == 8:  # Taxi is driving in the intersection

                nextRoadWay = taxi.getNextRoadWay()

                if (nextRoadWay == 'turnLeft'): #and midpoint:  # If next way is left
                    if len(left_fit) > 0 and len(right_fit) > 0:#midpoint[0] <= 240 and midpoint[0] >= 200:
                        if (-1.0 > left_fit_average[0] > -1.4) and (1.4 > left_fit_average[0] > 1.0) :
                            print('Planning: arrived at next road')
                            taxi.setState(7)
                elif (nextRoadWay == 'turnRight'): #and midpoint:
                    if len(left_fit) > 0 and len(right_fit) > 0:#midpoint[0] <= 240 and midpoint[0] >= 200:
                        if (-1.0 > left_fit_average[0] > -1.4) and (1.4 > left_fit_average[0] > 1.0) :
                            print('Planning: arrived at next road')
                            taxi.setState(7)
                elif (nextRoadWay == 'straight'):# and midpoint:
                    #if midpoint[0] <= 240 and midpoint[0] >= 200:
                    if flag_bluedot ==True:
                        flag_bluedot=False
                        print('Planning: arrived at next road')
                        taxi.setState(7)

            ######################################################################
            ############################## Control ###############################
            ######################################################################
            # 제어에서는, 시간 바꾸기, 제어 신호, State 변경만 가능
            state = taxi.getState()
            if state == 1:  # 차 세워
                taxi.setSpeed(0)
                result = 's'
                result_bytes = result.encode('utf_8')
                bluetooth.write(result_bytes)
                ########수정!!@#!@#!@#!@#@#
            elif state == 2:
                time.sleep(1)
                result = 'i'
                result_bytes = result.encode('utf_8')
                bluetooth.write(result_bytes)
                time.sleep(1)


                result = 'f'
                result_bytes = result.encode('utf_8')
                bluetooth.write(result_bytes)
                time.sleep(1)
                startTime = time.time()
                time_to_flask = 1

                taxi.setState(4)
                print("Control:car moving forward in road", taxi.getRoad())
            elif state == 3:
                print("")
            elif state == 4:
                if left_servo:
                    if change == False:  # 처음이라면
                        servo_start_time = time.time()
                        change = True
                        # make 응애 rc카 go left
                        print("turn left")
                        result = 'l'
                        result_bytes = result.encode('utf_8')
                        bluetooth.write(result_bytes)
                    else:  # 처음이 아니라면
                        servo_end_time = time.time()
                        servo_elapsed_time = servo_end_time - servo_start_time
                        if servo_elapsed_time >= threshold_time:
                            # make 응애 rc카 go straight
                            result = 'i'
                            result_bytes = result.encode('utf_8')
                            bluetooth.write(result_bytes)
                            left_servo = False
                            change = False
                elif right_servo:
                    if change == False:  # 처음이라면
                        print("turn right")
                        servo_start_time = time.time()
                        change = True
                        # make 응애 rc카 go right
                        result = 'r'
                        result_bytes = result.encode('utf_8')
                        bluetooth.write(result_bytes)
                    else:  # 처음이 아니라면
                        servo_end_time = time.time()
                        servo_elapsed_time = servo_end_time - servo_start_time
                        if servo_elapsed_time >= threshold_time:
                            # make 응애 rc카 go straight
                            result = 'i'
                            result_bytes = result.encode('utf_8')
                            bluetooth.write(result_bytes)
                            right_servo = False
                            change = False
            elif state == 5:

                result = 's'
                result_bytes = result.encode('utf_8')
                bluetooth.write(result_bytes)

                print('Control: stopped the car before entering the intersection')
            elif state == 6:
                nextRoadWay = taxi.getNextRoadWay()
                nextRoadWay = 'straight'
                if (nextRoadWay == 'turnLeft'):  # If next way is left
                    result = 'q3'
                    result_bytes = result.encode('utf_8')
                    bluetooth.write(result_bytes)
                    print('Control: taxi is now turning left')
                elif (nextRoadWay == 'turnRight'):
                    result = 'e'
                    result_bytes = result.encode('utf_8')
                    bluetooth.write(result_bytes)
                    print('Control: taxi is now turning right')
                elif (nextRoadWay == 'straight'):
                    result = 'f'
                    result_bytes = result.encode('utf_8')
                    bluetooth.write(result_bytes)
                    print('Control: taxi is now going straight')
                taxi.setState(8)  # Taxi driving in intersection
                startTime = time.time()
                print('Control: taxi now going into intersection')
            elif state == 7:
                time.sleep(0.05)
                result = 's'
                result_bytes = result.encode('utf_8')
                bluetooth.write(result_bytes)
                time.sleep(1)
            elif state == 8:
                pass

            ######################################################################
            ###################### Human-Machine Interface #######################
            ######################################################################
            # Show taxi camera with OD and LKAS results



            # Show 2D map
            tempY, tempX = taxi.getvcarYX()
            canvas = Image.new("RGBA", base_image.size, (0, 0, 0, 0))  # Create canvas
            canvas.paste(base_image, (0, 0))  # Paste base image
            tempDir = taxi.getDirection()
            if tempDir == 1:
                canvas.paste(overlay_image1, (tempX, tempY), overlay_image1)  # Paste overlaying image
            elif tempDir == 2:
                canvas.paste(overlay_image2, (tempX, tempY), overlay_image2)
            elif tempDir == 3:
                canvas.paste(overlay_image3, (tempX, tempY), overlay_image3)
            elif tempDir == 4:
                canvas.paste(overlay_image4, (tempX, tempY), overlay_image4)
            np_image = np.array(canvas)  # Convert PIL to np
            cv_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR
            cv2.namedWindow('Map', cv2.WINDOW_NORMAL)  # Give a specific name to the window
            cv2.resizeWindow('Map', 800, 800)  # Resize window size
            cv2.imshow('Map', cv_image)  # Show 2D map

            if cv2.waitKey(1) == ord('q'):  # Press Q to close the window. But window will pop back up in next iteration
                break

cam.release()
cv2.destroyAllWindows()