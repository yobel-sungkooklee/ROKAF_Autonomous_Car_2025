# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy, time, math
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
                pts = c.reshape(-1, 2)          # 4x2
                cx = float(np.mean(pts[:, 0]))
                cy = float(np.mean(pts[:, 1]))
                # 간단 면적 근사(사각형 bounding area)
                w = float(np.max(pts[:,0]) - np.min(pts[:,0]))
                h = float(np.max(pts[:,1]) - np.min(pts[:,1]))
                area = abs(w * h)
                results.append({"id": int(i), "center": (cx, cy), "area": area})
        return results


class ArucoTrigger(object):
    """
    - LANE_FOLLOW 상태에서만 마커를 감지해 트리거.
    - 새 ID 등장(혹은 동일 ID의 n번째 등장) + 쿨다운 충족 시 pending_actions(리스트) 세팅.
    - step()에서 리스트의 액션들을 순차 실행(제자리 회전) 후 다시 LANE_FOLLOW 복귀.
    """
    def __init__(self, cmd_topic="/cmd_vel"):
        # 규칙 테이블: { id: { nth: action or [actions...] } }
        # action: ("right"| "left" | "turn" | "turn1", degrees)
        #  - 요청사항: id=0 첫 등장 -> 오른쪽 90도 후 즉시 왼쪽 90도
        self.rules = {
            0: {1: [("right", 90), ("left", 90)]},      # 연속 액션
            2: {1: ("right", 90)},
            3: {1: ("left", 90), 2: ("right", 90)}, 
            4: {2: ("left", 90)},
            # 필요 시 계속 추가
        }

        self.detector = ArucoDetector()
        self.drive_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        self.mode = "LANE_FOLLOW"
        self.pending_actions = []
        self.seen_counts = {}          # {id: nth}

        # 🔻 전역 쿨다운 제거하고 per-ID로 교체
        # self.last_trigger_time = 0.0
        # self.trigger_cooldown = 5.0

        # ✅ 기본(디폴트) 쿨다운 + 마커별 오버라이드
        self.cooldown_default = 5.0      # 기본값(초)
        self.cooldown_per_id = {
            0: 6.5,   # id=0은 2초
            2: 1.0,   # id=2는 4초
            3: 4.0,   # id=3은 6초
            4: 1.0,   # id=4는 3초
            # 필요에 따라 추가/수정
        }
        self.last_trigger_times = {}      # {id: last_time}

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

    def observe_and_maybe_trigger(self, bgr_img):
        if self.mode != "LANE_FOLLOW":
            return

        now = time.time()

        # 🔻 (삭제) 전역 쿨다운 체크는 제거
        # if (now - self.last_trigger_time) < self.trigger_cooldown:
        #     return

        dets = self.detector.detect_ids(bgr_img)
        if not dets:
            self._consec = {}
            return

        dets = [d for d in dets if self._gate(d)]
        if not dets:
            self._consec = {}
            return

        det = max(dets, key=lambda x: x["area"])
        mid = det["id"]

        # 연속 프레임 카운트 갱신
        self._consec[mid] = self._consec.get(mid, 0) + 1
        for k in list(self._consec.keys()):
            if k != mid:
                self._consec[k] = 0

        if self._consec[mid] < self.required_consecutive:
            return

        # ✅ 여기서 '해당 마커'의 쿨다운만 확인
        last = self.last_trigger_times.get(mid, 0.0)
        cooldown = self.cooldown_per_id.get(mid, self.cooldown_default)
        if (now - last) < cooldown:
            return

        # 등장 횟수 → 규칙 매칭
        nth = self.seen_counts.get(mid, 0) + 1
        self.seen_counts[mid] = nth

        if (mid in self.rules) and (nth in self.rules[mid]):
            actions = self.rules[mid][nth]
            if isinstance(actions, tuple):
                actions = [actions]
            self.pending_actions = list(actions)
            self.mode = "EXECUTE_ACTION"

            # ✅ 트리거 타임스탬프는 해당 마커 id로 기록
            self.last_trigger_times[mid] = now

            self._consec = {}


    def _rotate_in_place(self, direction, degrees, ang_speed=1.0):
        """
        시간 기반 제자리 회전 (간단 근사)
        - direction: "right"/"left"/"turn"/"turn1"
        - degrees: 회전 각도(양수)
        - ang_speed: rad/s (절댓값 사용)
        """
        msg = Twist()
        msg.linear.x = 0.0

        if direction == "right":
            msg.angular.z = -abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        elif direction == "left":
            msg.angular.z = abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        elif direction == "turn":
            # 네 코드에 맞춰 120°로 유지 (필요하면 180으로 바꿔)
            msg.angular.z = abs(ang_speed)
            duration = 120.0 * math.pi/180.0 / abs(ang_speed)
        elif direction == "turn1":
            # 우회전과 동일 (오른쪽 양수/음수 선택은 너 로봇 좌표계에 따라 맞춰두었음)
            msg.angular.z = -abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        else:
            return

        rate = rospy.Rate(20)
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < duration and (not rospy.is_shutdown()):
            self.drive_pub.publish(msg)
            rate.sleep()

        # stop
        self.drive_pub.publish(Twist())

    def step(self):
        """
        EXECUTE_ACTION 상태일 때 호출하여
        pending_actions에 쌓인 액션들을 차례대로 실행.
        모두 끝나면 LANE_FOLLOW로 복귀.
        """
        if self.mode == "EXECUTE_ACTION" and self.pending_actions:
            # 안전 정지
            self.drive_pub.publish(Twist())
            rospy.sleep(0.15)

            # 맨 앞 액션 수행
            direction, deg = self.pending_actions.pop(0)
            self._rotate_in_place(direction, deg, ang_speed=1.0)

            # 남은 액션이 없으면 복귀
            if not self.pending_actions:
                self.mode = "LANE_FOLLOW"
