#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np

from green_object_tracking import ColorFilterTracker
from green_detect_macro import GreenTrigger
from infer_ros import YOLOInferenceNode
from lane_follower import lane_detect
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from detection_msgs.msg import BoundingBox, BoundingBoxes
from math import *

class PID:

    def __init__(self, kp, ki, kd):
        # PID 게인 초기화
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        # 이전 오차 초기화
        self.cte_prev = 0.0
        # 각 오차 초기화
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        # 적분오차 제한값 설정
        self.i_min = -10
        self.i_max = 10

    def pid_control(self, cte):
        # 미분오차 계산
        self.d_error = cte - self.cte_prev
        # 비례오차 계산
        self.p_error = cte
        # 적분오차 계산 및 제한 적용
        self.i_error += cte
        self.i_error = max(min(self.i_error, self.i_max), self.i_min)
        # 이전 오차 업데이트
        self.cte_prev = cte

        # PID 제어 출력 계산
        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error
    
def move(angle, speed, duration):   #매크로함수
    for i in range(int(duration)): 
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angle
        drive_pub.publish(msg)
        time.sleep(0.1)

def bbox_cb(msg):
    global det_bboxes
    det_bboxes = msg

def main():
    global drive_pub, det_bboxes
    rospy.init_node('Track_Driver')
    det_bboxes = BoundingBoxes()
    rospy.Subscriber("/detected_bboxes", BoundingBoxes, bbox_cb)
    drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    lcd_pub_1 = rospy.Publisher("/lcd_str_1", String, queue_size=1)

    rate = rospy.Rate(10)

    lf = lane_detect()          #차선인식
    cft = ColorFilterTracker()  #초록색 표시 인식
    pid = PID(0.01,0.1,0.5)     #회전 pid


    LANE_FOLLOWING = 0      #차선주행모드
    OBJECT_DETECTION = 1    #객체인식모드
    TURN_LEFT = 2           #좌회전
    TURN_RIGHT = 3          #우회전

    mode = 1                #시작모드
    last_green_idx = 0


    while not rospy.is_shutdown():

        if mode == LANE_FOLLOWING: 
            rospy.loginfo("LANE FOLLOWING")
            drive_pub.publish(lf.speed)
            rate.sleep()
            
            if cft.index != last_green_idx:          
                if cft.index == 1 or cft.index == 3 or cft.index == 8:
                    mode = TURN_LEFT
                elif cft.index == 5:
                    mode = TURN_RIGHT
                elif cft.index == 2 or cft.index == 4 or cft.index == 6 or cft.index == 7:
                    move(2.0,0,25)
                    mode = OBJECT_DETECTION
                else:
                    pass
                last_green_idx = cft.index
            

        elif mode == OBJECT_DETECTION:
            rospy.loginfo("OBJECT DETECTION")
            
            
            str_msg_1 = String()    #lcd msg
            start_time = time.time()
            event = 0               
            rot_start = 0          #회전 flag

            while event == 0:       #이벤트 발생할때 까지 회전
                alli = 0
                alli_tank = 0
                enem = 0
                enem_tank = 0
                curr_time = time.time()
                
                
                for bbox in det_bboxes.bounding_boxes: #객체 count 및 위치파악
                    if bbox.probability > 0.5:
                        if bbox.Class == "alli":
                            alli += 1
                        elif bbox.Class == "alli_tank":
                            alli_tank += 1
                        elif bbox.Class == "enem":
                            enem += 1
                        else:
                            enem_tank += 1
                            enem_tank_x = (bbox.xmin+bbox.xmax)/2
                          

                str_msg_1.data = f"alli: {alli}/{alli_tank},enem: {enem}/{enem_tank}"   #lcd string

                if enem_tank:   #이벤트가 있는 경우
                    x_err = 320-enem_tank_x         #화면 중심과 bbox 사이의 거리
                    theta = pid.pid_control(x_err)  #회전 조향각
                    
                    rot_msg = Twist()
                    rot_msg.linear.x = 0
                    rot_msg.angular.z = theta
                    drive_pub.publish(rot_msg)

                    if abs(x_err) < 5:              #bbox가 화면 중심에 위치
                        if rot_start == 0:
                            rot_start_time = curr_time
                            rot_start = 1
                        elif curr_time - rot_start_time > 5:   
                            str_msg_1.data = f"alli: {alli}/{alli_tank},enem: {enem}/{enem_tank} HIT!"  #이벤트 발생 lcd string
                            event = 1
                else:
                    if curr_time - start_time > 5:
                        event = 1
                
                lcd_pub_1.publish(str_msg_1)
                
                
                
                rate.sleep()

                if rospy.is_shutdown():
                    rospy.loginfo("Shutdown requested during object detection.")
                    return

            
            if cft.index == 2:
                move(2.0,0,25)
            else:
                move(-2.0,0,25)

            mode = LANE_FOLLOWING

        elif mode == TURN_LEFT:
            rospy.loginfo("TURN LEFT")
            move(2.0,0,25)
            move(0,0,1)
            mode = LANE_FOLLOWING

        elif mode == TURN_RIGHT:
            rospy.loginfo("TURN RIGHT")
            move(-2.0,0,25)
            move(0,0,1)
            mode = LANE_FOLLOWING
        
        rospy.loginfo(cft.index)


if __name__ == '__main__':
    main()
