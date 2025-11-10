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
    
    M = cv2.getPerspectiveTransform(source, destination)
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
        rospy.init_node('lane_detection_node', anonymous=False)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.camera_callback, queue_size=1, tcp_nodelay=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.speed = Twist()
        self.base_linear_speed = 0.1
        self.max_angular_speed = 1.5
        self.lat_weight = 1.2
        self.heading_weight = 0.7
        self.pid = PIDController(kp=0.9, ki=0.001, kd=0.01, integral_limit=2.0) # TODO
        self.aruco_trig = ArucoTrigger(cmd_topic="/cmd_vel")

    
    def camera_callback(self, data):
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.aruco_trig.observe_and_maybe_trigger(self.image)
        self.lane_detect()
        

    
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
        cv2.circle(self.image, (0,400), 10, (0,0,255), -1) #red
        cv2.circle(self.image, (640,400), 10, (0,255,0), -1) #green
        cv2.circle(self.image, (70,200), 10, (255,0,0), -1) #blue
        cv2.circle(self.image, (570,200), 10, (255,0,255), -1) #magenta
        cv2.imshow('Original', self.image)
        
        
        warpped_img, minv = warpping(self.image)
        cv2.namedWindow('BEV')
        cv2.moveWindow('BEV', 0, 0)
        cv2.imshow('BEV', warpped_img)
        
        blurred_img = cv2.GaussianBlur(warpped_img, (7, 7), 5)
        # cv2.namedWindow('Blurred')
        # cv2.moveWindow('Blurred', 350, 0)
        # cv2.imshow('Blurred', blurred_img)
        
        w_f_img = color_filter(blurred_img)
        # cv2.rectangle(w_f_img, (0, 0), (480, 100), (0, 0, 0), -1)
        # cv2.namedWindow('Color filter')
        # cv2.moveWindow('Color filter', 0, 550)
        # cv2.circle(w_f_img, (240,240), 2, (255,255,255), thickness=-1)
        # cv2.imshow('Color filter', w_f_img)
        
        grayscale = cv2.cvtColor(w_f_img, cv2.COLOR_BGR2GRAY)
        # print(grayscale[240][240])
        ret, thresh = cv2.threshold(grayscale, 50, 255, cv2.THRESH_BINARY) #170, 255
        
        canny_img = cv2.Canny(thresh, 10, 100)
        # cv2.namedWindow('Canny')
        # cv2.moveWindow('Canny', 700, 600)
        # cv2.imshow('Canny', canny_img)
        # cv2.namedWindow('thresh')
        # cv2.moveWindow('thresh', 700, 600)
        # cv2.imshow('thresh', thresh)
        
        lines = cv2.HoughLines(canny_img, 1, np.pi/180, 80, None, 0, 0)
        
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
        
        fit, avg = self.high_level_detect(hough_img)
        
        # fit = np.polyfit(np.array(y),np.array(x),1)
        # print(fit)
        
        line = np.poly1d(fit)
         
        # 좌,우측 차선의 휘어진 각도
        heading_rad = atan(line[1])


        cv2.namedWindow('Sliding Window')
        cv2.moveWindow('Sliding Window', 1400, 0)
        cv2.imshow("Sliding Window", out_img)
        cv2.waitKey(1)
        if fit[0]==0 and fit[1]==0:
            self.pid.reset()
            return

        p = np.poly1d(fit)
        x_at_480 = p(480)

        distance = - (x_at_480 - 240) # TODO
        print(distance)
        if np.isnan(distance) or np.isnan(heading_rad):
            self.pid.reset()
            return

        # print(line_angle, distance)

        theta_err = heading_rad
        lat_norm = distance / 240.0
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
