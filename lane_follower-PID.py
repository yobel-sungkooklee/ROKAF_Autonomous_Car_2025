#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from math import *
from collections import deque
# QR 연동이 포함된 버전 사용
from aruco_trigger_capture_yolo import ArucoTrigger

distance_threshold = 20
theta_threshold = 10
max_queue_size = 5  # 큐의 최대 크기 설정
weights = [1.0/85.0, 3.0/85.0, 7.0/85.0, 15.0/85.0, 60.0/85.0]

distance_L_queue = deque([], maxlen=5)
distance_R_queue = deque([], maxlen=5)
theta_L_queue = deque([], maxlen=5)
theta_R_queue = deque([], maxlen=5)

def region_of_interest(img, vertices, color3=(255,255,255), color1=255):

    mask = np.zeros_like(img) 
    
    if len(img.shape) > 2:
        color = color3
    else: 
        color = color1

    vertices = np.int32(vertices)
    cv2.fillPoly(mask, vertices, color)
    
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def warpping(image):
    """
        차선을 BEV로 변환하는 함수
        
        Return
        1) _image : BEV result image
        2) minv : inverse matrix of BEV conversion matrix
    """

    # roi_source = np.float32([[86, 150], [554, 150], [640, 400], [0, 400]])
    roi_source = np.float32([[80, 0], [560, 0], [560, 480], [80, 480]])
    # source = np.float32([[200, 210], [20,480], [420,210], [620, 480]])
    source = np.float32([[70, 200], [0, 400], [570, 200], [640, 400]]) # TODO
    destination = np.float32([[0, 0], [0, 480], [480, 0], [480, 480]])
    
    M = cv2.getPerspectiveTransform(source, destination) # BEV 변환 행렬 (원근 변환 행렬)
    Minv = cv2.getPerspectiveTransform(destination, source)
    
    # image = region_of_interest(image, [roi_source])
    
    warp_image = cv2.warpPerspective(image, M, (480, 480), flags=cv2.INTER_LINEAR)
    #warp_image = region_of_interest(warp_image, [roi_source])
    # cv2.rectangle(warp_image, (195, 445), (480, 480), (0, 0, 0), -1)

    return warp_image, Minv

def color_filter(image):
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    lower = np.array([200, 200, 200])
    upper = np.array([255, 255, 255])
    # print(hls[240][240])
    # lower = np.array([40, 185, 10])
    # upper = np.array([255, 255, 255])

    # yellow_lower = np.array([0, 85, 81])
    #yellow_lower = np.array([0, 85, 81])
    #yellow_upper = np.array([190, 255, 255])
    # blue_lower = np.array([0, 51, 90])
    # blue_upper = np.array([255, 204, 255])
    black_lower = np.array([0, 0, 0])        
    black_upper = np.array([180, 255, 50])
    #yellow_mask = cv2.inRange(hls, yellow_lower, yellow_upper)
    #white_mask = cv2.inRange(image, lower, upper)
    #mask = cv2.bitwise_or(yellow_mask, white_mask)
    black_mask = cv2.inRange(hls, black_lower, black_upper)
    masked = cv2.bitwise_and(image, image, mask = black_mask)
    
    return masked

def moving_filter(queue, weights):
    result = [a * b for a, b in zip(queue, weights)]
    return sum(result)

def compare_to_previous_value(queue, data, threshold, max_size):
    if (not queue) or (len(queue) < max_size):
        result = 80 ### 바꿔주세요, 에러 안 뜨게 하려고 임의로 넣었습니다. 
        queue.append(data)
    elif len(queue) == max_size:
        last_data = queue[-1]

        for i in range(len(queue)):
            diff = abs(data - queue[i])
            if diff <= threshold:
                append = True
                break
            else:
                append = False

        if append:
            if len(queue) == max_size:
                queue.popleft()
            queue.append(data)
            result = moving_filter(queue, weights)
        else:
            if len(queue) == max_size:
                queue.popleft()
            queue.append(last_data)
            # queue.append(sum(queue) / max_queue_size)
            result = moving_filter(queue, weights)
    return result


class PIDController(object):
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, current_time):
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = max(current_time - self.prev_time, 0.0)

        if dt > 0.0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0

        self.integral += error * dt
        if self.integral_limit is not None:
            if self.integral > self.integral_limit:
                self.integral = self.integral_limit
            elif self.integral < -self.integral_limit:
                self.integral = -self.integral_limit

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        self.prev_time = current_time
        return output


class lane_detect():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('lane_detection_node', anonymous=False) # # ROS 마스터에 lane_detection_node라는 이름으로 노드 등록 -> 중복 실행/재실행 시 ROS 이름 충돌 때문에 죽지 않도록
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.camera_callback, queue_size=1, tcp_nodelay=True) # USB 카메라 압축 스트림을 구독하고 새 프레임마다 self.camera_callback을 호출. queue_size=1로 최신 한 장만 유지해 지연을 줄이고, tcp_nodelay=True로 작은 패킷도 즉시 전달되게 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) #  cmd_vel 퍼블리셔를 만들어 차선 제어 결과를 곧바로 로봇에 보낼 수 있게

        self.speed = Twist() # 퍼블리시할 메시지 객체를 미리 만들고 각 필드를 반복해서 갱신
        
        # 직진 기본 속도와 허용되는 최대 회전각속도를 정의.
        self.base_linear_speed = 0.1
        self.max_angular_speed = 1.5
        
        # 에러 가중치 합을 위해서, 횡방향 오차와 헤딩 오차를 얼마나 중요하게 다룰지 가중치로 설정, 두 값을 조합해 PID 입력을 구성
        self.lat_weight = 1.2
        self.heading_weight = 0.7
        
        # 조향용 PID를 원하는 Gain과 적분 한계로 초기화 -> self.pid.update()에서 사용
        self.pid = PIDController(kp=0.9, ki=0.001, kd=0.01, integral_limit=2.0) # TODO
        
        # ArucoTrigger 객체 생성: 주행 중 특정 ArUco 마커(또는 QR  코드)를 보게 되면 정해진 회전·이미지 캡처 같은 “미션 동작”을 실행할 수 있도록 -> ArUco 마커를 감지하면 규칙 테이블에 따라 "right", 90° 같은 액션 목록을 큐에 쌓아 둠
        self.aruco_trig = ArucoTrigger(cmd_topic="/cmd_vel")

    
    def camera_callback(self, data):
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8") # 압축된 이미지 메시지를 OpenCV BGR 이미지로 변환
        self.aruco_trig.observe_and_maybe_trigger(self.image) # 현재 프레임에서 ArUco/QR 코드를 감시해, 규칙 상 실행해야 할 액션이 생겼는지 판단하는 단계
        self.lane_detect() # 차선 처리 파이프라인 전체 수행
        

    
    def high_level_detect(self, hough_img):

        nwindows = 15       # window 개수
        margin = 90         # window 가로 길이
        minpix = 15          # 차선 인식을 판정하는 최소 픽셀 수
        lane_bin_th = 225
       
        histogram = np.sum(hough_img[hough_img.shape[0]//2:,:],   axis=0)
        
        midpoint = np.int32(histogram.shape[0]/2)
    
        midx_current = np.argmax(histogram[:])

        # print("left_cur : %.3f, right_cur : %.3f" % (leftx_current, rightx_current))

        # 쌓을 window의 height 설정
        window_height = np.int32(hough_img.shape[0]/nwindows)
        
        # 240*320 픽셀에 담긴 값중 0이 아닌 값을 저장한다.
        # nz[0]에는 index[row][col] 중에 row파트만 담겨있고 nz[1]에는 col이 담겨있다.
        nz = hough_img.nonzero()

        mid_lane_inds = []

        global x,y
        x,y = [],[]

        global out_img
        out_img = np.dstack((hough_img, hough_img, hough_img))*255

        cnt = 0
        
        mid_sum = 0

        total_loop = 0

        for window in range(nwindows-4):
            
            # bounding box 크기 설정
            win_yl = hough_img.shape[0] - (window+1)*window_height
            win_yh = hough_img.shape[0] - window*window_height

            win_xl = midx_current - margin
            win_xh = midx_current + margin

            # out image에 bounding box 시각화
            cv2.rectangle(out_img,(win_xl,win_yl),(win_xh,    win_yh),    (0,255,0), 2) 

            # 흰점의 픽셀들 중에 window안에 들어오는 픽셀인지 여부를 판단하여 
            # good_left_inds와 good_right_inds에 담는다.
            good_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&   (nz    [1] >= win_xl)&(nz[1] < win_xh)).nonzero()    [0]
            
            mid_lane_inds.append(good_inds)

            # nz[1]값들 중에 good_left_inds를 index로 삼는 nz[1]들의 평균을 구해서 leftx_current를 갱신한다.
            if len(good_inds) > minpix:
                midx_current = np.int32(np.mean(nz[1]    [good_inds])   )

            #lx ly rx ry에 x,y좌표들의 중심점들을 담아둔다.
            x.append(midx_current)
            y.append((win_yl + win_yh)/2)

            # left_sum += leftx_current
            mid_sum += midx_current

            total_loop += 1

        mid_lane_inds = np.concatenate(mid_lane_inds)

        fit = np.polyfit(np.array(y[1:]),np.array(x[1:]),2)
        
        #out_img에서 왼쪽 선들의 픽셀값을 BLUE로, 
        #오른쪽 선들의 픽셀값을 RED로 바꿔준다.
        out_img[nz[0][mid_lane_inds], nz[1][mid_lane_inds]] = [255, 0, 0]

        mid_avg = mid_sum / total_loop

        return fit, mid_avg
    
    def lane_detect(self):
                
        cv2.namedWindow('Original')
        cv2.moveWindow('Original', 700, 0)
        
        # 워핑 기준점 표시 - 워핑(Warping): 원본 카메라 영상을 Bird’s Eye View처럼 보이도록 좌표계를 변환하는 작업
        cv2.circle(self.image, (0,400), 10, (0,0,255), -1) #red
        cv2.circle(self.image, (640,400), 10, (0,255,0), -1) #green
        cv2.circle(self.image, (70,200), 10, (255,0,0), -1) #blue
        cv2.circle(self.image, (570,200), 10, (255,0,255), -1) #magenta
        cv2.imshow('Original', self.image)
        
        # 네 개의 기준점(바닥면에서 보고 싶은 사다리꼴 영역의 모서리)을 입력으로 받아 cv2.getPerspectiveTransform을 통해 투시 변환 행렬을 만듦)
        warpped_img, minv = warpping(self.image)
        cv2.namedWindow('BEV')
        cv2.moveWindow('BEV', 0, 0)
        cv2.imshow('BEV', warpped_img)
        
        # 가우시안 블러 적용: 노이즈(고주파) 성분 먼저 줄여서 뒤따르는 단계들이 더 안정적으로 동작하도록.
        blurred_img = cv2.GaussianBlur(warpped_img, (7, 7), 5)
        # cv2.namedWindow('Blurred')
        # cv2.moveWindow('Blurred', 350, 0)
        # cv2.imshow('Blurred', blurred_img)
        
        # BGR 이미지 -> HLS 색 공간으로 변환 후, 지정한 범위(여기서는 black_lower=[0,0,0], black_upper=[180,255,50])에 속하는 픽셀만 남도록 cv2.inRange를 적용
        w_f_img = color_filter(blurred_img)
        # cv2.rectangle(w_f_img, (0, 0), (480, 100), (0, 0, 0), -1)
        # cv2.namedWindow('Color filter')
        # cv2.moveWindow('Color filter', 0, 550)
        # cv2.circle(w_f_img, (240,240), 2, (255,255,255), thickness=-1)
        # cv2.imshow('Color filter', w_f_img)
        
        # 그레이스케일을 통해 밝기 정보만을 이용하기 위함: threshold나 Canny 같은 함수는 단일 채널(흑백) 데이터를 요구하거나 그쪽이 더 명확하게 동작
        grayscale = cv2.cvtColor(w_f_img, cv2.COLOR_BGR2GRAY)
        # print(grayscale[240][240])
        
        # 그레이스케일 이미지를 이진 영상으로 바꾸는 단계입니다. 픽셀 값이 50 이상이면 255(흰색)로, 50 미만이면 0(검정)으로 설정해 “차선 후보 vs 배경”을 분리 -> thresh: 이진화된 결과 이미지
        ret, thresh = cv2.threshold(grayscale, 50, 255, cv2.THRESH_BINARY) #170, 255
        
        # Canny 엣지 검출 적용: 차선의 경계선을 더 뚜렷하게 강조: 이진화된 이미지 thresh를 입력으로 받아, 그라디언트 크기를 계산하고 하위/상위 임계값(10, 100)을 이용해 진짜 엣지와 약한 엣지를 분류해 얇은 에지 선만 남김 ->  이렇게 하면 나중에 cv2.HoughLines가 차선 경계를 더 정확히 찾을 수 있음.
        canny_img = cv2.Canny(thresh, 10, 100)
        # cv2.namedWindow('Canny')
        # cv2.moveWindow('Canny', 700, 600)
        # cv2.imshow('Canny', canny_img)
        # cv2.namedWindow('thresh')
        # cv2.moveWindow('thresh', 700, 600)
        # cv2.imshow('thresh', thresh)
        
        
        # 캐니로 얻은 엣지 이미지에서 직선을 찾는 Hough 변환: 이 함수는 (rho, theta) 쌍들의 배열을 돌려주고, 각 쌍이 이미지 상의 한 직선을 뜻함. 이후 코드는 이 결과를 이용해 차선 후보를 시각화하거나 슬라이딩 윈도를 위한 마스크를 만듦.
        lines = cv2.HoughLines(canny_img, 1, np.pi/180, 80, None, 0, 0)
        
        # Hough 변환 결과를 시각화: cv2.HoughLines가 준 각 (rho, theta) 직선 정보를 실제 이미지 좌표로 변환
        #hough_img = thresh.copy()
        #hough_img = canny_img.copy()
        hough_img = np.zeros((480, 480))
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int (x0 + 1000*(-b))
                y1 = int ((y0) + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                slope = 90 - degrees(atan(b / a))
            
                if abs(slope) < 5:
                    cv2.line(hough_img, (x1, y1), (x2, y2), 0, 30)
                else:    
                    cv2.line(hough_img, (x1, y1), (x2, y2), 255, 8)
        
        # cv2.namedWindow('Hough')
        # cv2.moveWindow('Hough', 700, 0)
        # cv2.imshow('Hough', hough_img)
        
        fit, avg = self.high_level_detect(hough_img) # hough_img의 하단 절반부터 장단으로 올라가면서, 가로로 긴 슬라이딩 윈도우를 여러 개 쌓아 올림. -> 각 윈도 안에서 값이 0이 아닌 픽셀(차선 후보)을 찾아 평균 x 좌표를 계산하면, 해당 높이(y)에서 차선 중심이 어디인지 알 수 있음. 이렇게 15개 정도의 윈도에서 (y, x) 쌍을 모아두고, 마지막에 이 점들을 2차 다항식으로 피팅하면 “차선 중앙 곡선”이 나옴.
        
        # fit = np.polyfit(np.array(y),np.array(x),1)
        # print(fit)
        
        # 다항식 객체를 만든 뒤 line[1]을 읽어 1차 계수(기울기)를 얻음.
        line = np.poly1d(fit)
         
        # atan(line[1])로 차선 곡선의 헤딩 편차(라디안)를 계산 -> 이 heading_rad는 PID 제어에서 사용되는 heading 에러
        heading_rad = atan(line[1])

        cv2.namedWindow('Sliding Window')
        cv2.moveWindow('Sliding Window', 1400, 0)
        cv2.imshow("Sliding Window", out_img)
        cv2.waitKey(1)
        
        # 슬라이딩 슬라이딩 윈도 결과가 유효하지 않을 때(차선을 못 찾았거나 모두 0으로 떨어졌을 때) PID를 초기화하고 제어를 건너뛰기 위한 용도
        if fit[0]==0 and fit[1]==0:
            self.pid.reset()
            return

        # y=480(이미지 하단, 차량 바로 앞)을 기준으로 차선 중심 x 좌표를 계산. 슬라이딩 윈도에서 얻은 2차 곡선을 p(y)로 보고, 맨 아래에서 차선이 화면 중앙(240)에 대해 얼마나 치우쳤는지 확인하는 단계                           
        p = np.poly1d(fit)
        x_at_480 = p(480)

        # 차선 중심과 차량 중심 사이의 횡방향 오차(픽셀)
        distance = - (x_at_480 - 240) # TODO
        print(distance)
        if np.isnan(distance) or np.isnan(heading_rad): # 계산 중 NaN이 발생하면 PID 제어를 건너뛰고 리셋
            self.pid.reset()
            return

        # print(line_angle, distance)

        theta_err = heading_rad # 헤딩 에러: 차량 진행 방향 대비 차선이 어느 방향으로 휘어 있는지
        lat_norm = distance / 240.0 # 횡방향 오차 정규화: 차선 중심이 화면 중앙에서 얼마나 좌우로 벗어났는지를 픽셀 단위 distance로 측정한 뒤, 최대 반폭 240픽셀로 나눠 -1.0~1.0 범위로 정규화한 “횡방향( lateral ) 에러”
        
        # 가중합 에러
        combined_error = (self.lat_weight * lat_norm) + (self.heading_weight * theta_err)

        pid_output = self.pid.update(combined_error, rospy.get_time())
        pid_output = np.clip(pid_output, -self.max_angular_speed, self.max_angular_speed)

        self.speed.linear.x = self.base_linear_speed
        self.speed.angular.z = pid_output
        self.pub.publish(self.speed)
        #print("angle(rad): ", theta_err, "lat_norm: ", lat_norm)
        #print("cmd_ang: ", self.speed.angular.z)
        
        self.aruco_trig.step()
        


if __name__ == "__main__":

    if not rospy.is_shutdown():
        lane_detect()
        rospy.spin()
