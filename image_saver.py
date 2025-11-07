#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
from datetime import datetime
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver_node', anonymous=True)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.latest_frame = None

        # 저장 폴더 생성 (없으면 새로 만듦)
        self.save_dir = os.path.join(os.getcwd(), "images")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        print("[INFO] Image saver ready. Press SPACE to capture, ESC to exit.")

    def image_callback(self, msg):
        """ROS 카메라 토픽으로부터 이미지를 받는 콜백"""
        self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

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
