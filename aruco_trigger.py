# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy, time, math, os
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

# OpenCV 버전에 따른 API 차이 호환
try:
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
except AttributeError:
    ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

try:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
except AttributeError:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()

class ArucoDetector(object):
    def __init__(self):
        self.bridge = CvBridge()

    def detect_ids(self, bgr_img):
        """ bgr_img에서 (id, center(x,y), 면적) 리스트 반환 """
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        results = []
        if ids is not None:
            ids = ids.flatten()
            for c, i in zip(corners, ids):
                pts = c.reshape(-1, 2)  # 4x2
                cx = float(np.mean(pts[:, 0]))
                cy = float(np.mean(pts[:, 1]))
                w = float(np.max(pts[:, 0]) - np.min(pts[:, 0]))
                h = float(np.max(pts[:, 1]) - np.min(pts[:, 1]))
                area = abs(w * h)
                results.append({"id": int(i), "center": (cx, cy), "area": area})
        return results


class ArucoTrigger(object):
    """
    - LANE_FOLLOW 상태에서만 마커를 감지해 트리거.
    - 새 ID 등장(혹은 동일 ID의 n번째 등장) + 쿨다운 충족 시 pending_actions 세팅.
    - step()에서 리스트의 액션들을 순차 실행 후 다시 LANE_FOLLOW 복귀.
    """
    def __init__(self, cmd_topic="/cmd_vel"):
        self.rules = {
            # 캡처 액션 포함 예시
            0: {1: [("right", 90)]},
            2: {1: ("right", 90)},
            3: {1: [("left", 90), ("capture", 0)], 2: ("right", 90)},
            4: {2: ("left", 90)},
        }

        self.detector = ArucoDetector()
        self.drive_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        self.mode = "LANE_FOLLOW"
        self.pending_actions = []
        self.seen_counts = {}

        # ✅ 마커별 쿨다운 설정
        self.cooldown_default = 5.0
        self.cooldown_per_id = {0: 6.5, 2: 1.0, 3: 4.0, 4: 1.0}
        self.last_trigger_times = {}

        # 📸 캡처 관련 설정
        # self.capture_target_ids 제거됨
        self.capture_count = {}  # {id: count}
        self.save_dir = os.path.expanduser("~/catkin_ws/src/ROKAF_Autonomous_Car_2025/images")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo("[ArucoTrigger] Created directory: %s", self.save_dir)
            
        # 이미지 저장을 위해 마지막으로 감지된 프레임을 저장할 변수
        self._last_bgr_img = None 
        self._last_marker_id = None # 캡처 이미지에 사용할 마커 ID 저장

        self.required_consecutive = 3
        self._consec = {}
        self.min_area = 80.0
        self.min_y = 60.0
        self.max_y = 460.0

    def _gate(self, det):
        area_ok = det["area"] >= self.min_area
        y = det["center"][1]
        y_ok = (y >= self.min_y) and (y <= self.max_y)
        return area_ok and y_ok
        
    def _capture_image(self):
        """저장된 마지막 프레임과 마커 ID를 사용하여 이미지를 캡처합니다."""
        if self._last_bgr_img is None or self._last_marker_id is None:
            rospy.logwarn("[ArucoTrigger] Cannot capture image: last frame or ID is missing.")
            return
            
        mid = self._last_marker_id
        
        # 캡처 카운트 증가 및 파일 저장 로직
        if mid not in self.capture_count:
            self.capture_count[mid] = 0
        self.capture_count[mid] += 1
        
        filename = os.path.join(self.save_dir, "triggered_object{}_{}.jpg".format(mid, self.capture_count[mid]))
        cv2.imwrite(filename, self._last_bgr_img)
        rospy.loginfo("[ArucoTrigger] Triggered image saved: {}".format(filename))


    def observe_and_maybe_trigger(self, bgr_img):
        # 가장 최근 프레임을 저장합니다. (step()에서 캡처 액션을 위해 사용)
        self._last_bgr_img = bgr_img 
        
        if self.mode != "LANE_FOLLOW":
            return

        now = time.time()
        dets = self.detector.detect_ids(bgr_img)
        if not dets:
            self._consec = {}
            self._last_marker_id = None
            return

        dets = [d for d in dets if self._gate(d)]
        if not dets:
            self._consec = {}
            self._last_marker_id = None
            return

        det = max(dets, key=lambda x: x["area"])
        mid = det["id"]
        
        # 마지막으로 감지된 유효 마커 ID를 저장합니다.
        self._last_marker_id = mid

        # 기존의 self.capture_target_ids를 이용한 이미지 저장 로직은 제거됨

        # 연속 프레임 카운트
        self._consec[mid] = self._consec.get(mid, 0) + 1
        for k in list(self._consec.keys()):
            if k != mid:
                self._consec[k] = 0

        if self._consec[mid] < self.required_consecutive:
            return

        # 쿨다운 확인
        last = self.last_trigger_times.get(mid, 0.0)
        cooldown = self.cooldown_per_id.get(mid, self.cooldown_default)
        if (now - last) < cooldown:
            return

        # 등장 횟수 카운트
        nth = self.seen_counts.get(mid, 0) + 1
        self.seen_counts[mid] = nth

        if (mid in self.rules) and (nth in self.rules[mid]):
            actions = self.rules[mid][nth]
            if isinstance(actions, tuple):
                actions = [actions]
            self.pending_actions = list(actions)
            self.mode = "EXECUTE_ACTION"
            self.last_trigger_times[mid] = now
            self._consec = {}

    def _rotate_in_place(self, direction, degrees, ang_speed=1.0):
        msg = Twist()
        msg.linear.x = 0.0

        if direction == "right":
            msg.angular.z = -abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        elif direction == "left":
            msg.angular.z = abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        elif direction == "turn":
            msg.angular.z = abs(ang_speed)
            duration = 120.0 * math.pi/180.0 / abs(ang_speed)
        elif direction == "turn1":
            msg.angular.z = -abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        else:
            return

        rate = rospy.Rate(20)
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < duration and (not rospy.is_shutdown()):
            self.drive_pub.publish(msg)
            rate.sleep()

        self.drive_pub.publish(Twist())

    def step(self):
        if self.mode == "EXECUTE_ACTION" and self.pending_actions:
            self.drive_pub.publish(Twist())
            rospy.sleep(0.15)
            
            direction, deg = self.pending_actions.pop(0)
            
            # 캡처 액션 처리
            if direction == "capture":
                self._capture_image()
            else:
                self._rotate_in_place(direction, deg, ang_speed=1.0)
                
            if not self.pending_actions:
                self.mode = "LANE_FOLLOW"