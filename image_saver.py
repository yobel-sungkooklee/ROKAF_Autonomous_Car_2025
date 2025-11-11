#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import numpy as np
from datetime import datetime
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver_node', anonymous=True) # ROS 마스터에 image_saver_node라는 이름으로 노드 등록 -> 중복 실행/재실행 시 ROS 이름 충돌 때문에 죽지 않도록
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1) # USB 카메라에서 퍼블리시하는 압축 이미지 토픽을 구독하고, 메시지가 도착할 때마다 self.image_callback 메서드를 호출
        self.bridge = CvBridge() # ROS 이미지 메시지를 OpenCV의 numpy 배열로 바꾸기 위해 CvBridge 객체를 생성(queue_size=1은 최신 프레임만 유지해 지연을 줄임) -> ROS압축이미지를 OpenCV 이미지로 바꿀 준비
        self.latest_frame = None # 최근 프레임을 저장할 변수 초기화

        # 저장 폴더 생성 (없으면 새로 만듦) -> 현재는 yolo_images 폴더에 저장
        self.save_dir = os.path.join(os.getcwd(), "yolo_images")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        print("[INFO] Image saver ready. Press SPACE to capture, ESC to exit.")

    def image_callback(self, msg):
        """Update latest frame from ROS topic"""
        # Option 1: using CvBridge
        self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Option 2: using numpy + cv2 (alternative)
        # np_arr = np.frombuffer(msg.data, np.uint8)
        # self.latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def run(self):
        """스페이스바 누르면 이미지 저장"""
        while not rospy.is_shutdown():
            if self.latest_frame is None:
                continue

            cv2.imshow("Live Feed", self.latest_frame)
            key = cv2.waitKey(1) & 0xFF

            # 스페이스바(32) 누르면 저장
            if key == 32:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = os.path.join(self.save_dir, "image_{}.jpg".format(timestamp))
                cv2.imwrite(filename, self.latest_frame)
                print("[SAVED] {}".format(filename))

            # ESC(27) 누르면 종료
            elif key == 27:
                print("[EXIT] Closing window...")
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    saver = ImageSaver()
    saver.run()
