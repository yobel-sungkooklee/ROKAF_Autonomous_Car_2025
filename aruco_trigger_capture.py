# -*- coding: utf-8 -*-

#!/usr/bin/env python
import rospy, time, math, os
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


# OpenCV 버전에 따라 ArUco API 이름이 다른 경우가 있어서, 예외 처리를 통해 호환성 확보
try:
    # DICT_4X4_50: 4x4 비트 패턴, 50개의 서로 다른 마커가 정의된 사전(dictionary)
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# 만약 해당 속성이 없다면(구버전), 다른 이름의 API로 대체
except AttributeError:
    ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

try:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
except AttributeError:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()


# ArUco 마커를 검출해주는 전용 헬퍼 클래스 정의
class ArucoDetector(object):
    # 생성자: CvBridge를 초기화하여 ROS 이미지 <-> OpenCV 이미지를 변환 가능하게 함
    def __init__(self):
        self.bridge = CvBridge()

    # BGR 이미지(bgr_img)를 입력받아,
    #  - id: 마커 ID
    #  - center: (x, y) 중심 좌표
    #  - area: 마커의 대략적인 면적(픽셀 단위)
    # 를 담은 딕셔너리 리스트를 반환하는 함수
    def detect_ids(self, bgr_img):
        """ bgr_img에서 (id, center(x,y), 면적) 리스트 반환 """
        # 컬러 이미지를 그레이스케일로 변환 (마커 검출은 보통 흑백으로 수행)
        gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

        # ArUco 마커 검출 수행
        # corners: 각 마커의 꼭짓점 좌표들
        # ids: 각 마커의 ID 배열
        # _ : 사용하지 않는 리턴값(검출된 리젝트 포인트 등)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        # 최종 결과를 담을 리스트 초기화
        results = []

        # 마커가 하나라도 검출된 경우에만 처리
        if ids is not None:
            # ids는 (N,1) 형태일 수 있으므로 1차원 배열로 평탄화
            ids = ids.flatten()
            # corners와 ids를 함께 순회하면서 각 마커에 대한 정보 계산
            for c, i in zip(corners, ids):
                # c는 (1,4,2) 형태이므로 (4,2)로 reshape: 마커 4개의 꼭짓점 (x,y)
                pts = c.reshape(-1, 2)  # 4x2

                # x좌표들의 평균 -> 중심 x 좌표
                cx = float(np.mean(pts[:, 0]))
                # y좌표들의 평균 -> 중심 y 좌표
                cy = float(np.mean(pts[:, 1]))

                # 가로 길이는 x좌표 최댓값 - 최솟값
                w = float(np.max(pts[:, 0]) - np.min(pts[:, 0]))
                # 세로 길이는 y좌표 최댓값 - 최솟값
                h = float(np.max(pts[:, 1]) - np.min(pts[:, 1]))

                # 사각형으로 근사한 면적 = w * h (절댓값으로 안정성 확보)
                area = abs(w * h)

                # id, center, area 정보를 딕셔너리로 묶어서 리스트에 추가
                results.append({"id": int(i), "center": (cx, cy), "area": area})

        # 모든 마커에 대한 정보를 담은 리스트 반환
        return results


# ArucoTrigger: ArUco 마커를 감지해서 로봇의 행동을 트리거하는 핵심 클래스
class ArucoTrigger(object):
    """
    - LANE_FOLLOW 상태에서만 마커를 감지해 트리거.
    - 새 ID 등장(혹은 동일 ID의 n번째 등장) + 쿨다운 충족 시 pending_actions 세팅.
    - step()에서 리스트의 액션들을 순차 실행 후 다시 LANE_FOLLOW 복귀.
    """

    # 생성자: cmd_topic은 로봇 속도를 publish할 ROS 토픽 이름
    def __init__(self, cmd_topic="/cmd_vel"):
        # self.rules: 마커 ID와 등장 횟수(nth)에 따라 실행할 액션을 정의하는 규칙 테이블
        # 형식: { marker_id: { nth: action 또는 [action들] } }
        # 여기서는 예시로 캡처 액션을 포함한 규칙들이 정의되어 있음
        self.rules = {
            # 캡처 액션 포함 예시
            # id=0 마커가 1번째 등장할 때: 오른쪽 90도 회전
            0: {1: [("right", 90)]},
            # id=2 마커가 1번째 등장할 때: 오른쪽 90도 회전
            2: {1: ("right", 90)},
            # id=3 마커가 1번째 등장할 때: 왼쪽 90도 회전 후 캡처,
            3: {1: [("left", 90), ("capture", 0)], 2: ("right", 90)},
            # id=4 마커가 2번째 등장할 때: 왼쪽 90도 회전
            4: {2: ("left", 90)},
        }

        # ArUco 마커를 실제로 검출하는 헬퍼 객체
        self.detector = ArucoDetector()

        # 로봇의 속도 명령(Twist)을 publish하는 ROS Publisher
        self.drive_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        # 현재 동작 모드: 기본은 차선 따라가기(LANE_FOLLOW)
        self.mode = "LANE_FOLLOW"

        # 앞으로 실행할 액션들(action 튜플 리스트)을 담아두는 큐 역할 리스트
        self.pending_actions = []

        # 각 마커 ID별로 지금까지 몇 번 등장했는지 카운트하는 딕셔너리 {id: nth}
        self.seen_counts = {}

        # 마커별 쿨다운 설정: 같은 마커가 너무 자주 트리거되지 않도록 제한
        # 기본 쿨다운 시간(초) – 특정 id에 대한 설정이 없을 때 사용하는 값
        self.cooldown_default = 5.0

        # 각 마커 id에 대해 별도의 쿨다운 시간을 부여하는 딕셔너리
        # 예: id=0은 6.5초, id=2는 1.0초 등
        self.cooldown_per_id = {0: 6.5, 2: 1.0, 3: 4.0, 4: 1.0}

        # 각 마커 id가 마지막으로 트리거된 시간을 기록하는 딕셔너리 {id: last_trigger_time}
        self.last_trigger_times = {}

        # 캡처 관련 설정
        # 예전에는 특정 id만 캡처하는 capture_target_ids를 썼을 수 있으나 지금은 제거됨
        # self.capture_target_ids 제거됨
        # 각 마커 id별로 몇 번 캡처했는지 count를 저장하는 딕셔너리 {id: count}
        self.capture_count = {}  # {id: count}

        # 이미지를 저장할 디렉토리 경로 설정
        # os.path.expanduser를 사용해 ~ 를 실제 홈 디렉토리로 확장
        self.save_dir = os.path.expanduser("~/catkin_ws/src/ROKAF_Autonomous_Car_2025/images")

        # 디렉토리가 존재하지 않으면 새로 생성
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            # 디렉토리를 새로 만들었다는 정보를 ROS 로그로 출력
            rospy.loginfo("[ArucoTrigger] Created directory: %s", self.save_dir)

        # 이미지 저장을 위해 마지막으로 감지된 프레임을 저장할 변수
        # observe_and_maybe_trigger에서 최신 bgr_img를 계속 업데이트해줌
        self._last_bgr_img = None

        # 마지막으로 감지된 유효한 마커 ID를 저장
        # 캡처할 때 파일 이름에 마커 ID를 포함하기 위해 사용
        self._last_marker_id = None # 캡처 이미지에 사용할 마커 ID 저장

        # 연속 프레임에서 몇 번 이상 같은 마커가 조건을 만족해야
        # "진짜로 본 것"으로 인정할지에 대한 임계값 (노이즈 제거용)
        self.required_consecutive = 3

        # 각 마커 id에 대해 현재 연속으로 몇 프레임 동안 조건을 만족했는지 카운트하는 딕셔너리
        self._consec = {}

        # 최소 면적 임계값: 너무 작은 마커(너무 멀거나 노이즈)는 무시
        self.min_area = 80.0

        # 화면 y좌표의 최소값: 너무 위쪽(또는 아래쪽)을 잘라내기 위한 gate
        self.min_y = 60.0

        # 화면 y좌표의 최대값: 이 값을 초과하면 관심 영역 밖으로 간주
        self.max_y = 460.0


    # _gate: 하나의 검출(det)에 대해 "이걸 유효한 마커로 볼 것인가?"를 결정하는 함수
    # det는 {"id": ..., "center": (cx, cy), "area": ...} 구조의 딕셔너리
    def _gate(self, det):
        # 면적 조건: det["area"]가 최소 면적 이상인지 검사
        area_ok = det["area"] >= self.min_area

        # y좌표를 center에서 가져옴 (center는 (x,y) 튜플)
        y = det["center"][1]

        # y좌표가 min_y 이상이고 max_y 이하인지 확인
        # 즉, 세로 위치가 지정된 관심 구역 안에 있는지 판단
        y_ok = (y >= self.min_y) and (y <= self.max_y)

        # 두 조건(면적, y범위)을 모두 만족해야 True
        return area_ok and y_ok


    # _capture_image: 마지막으로 저장된 프레임과 마지막 마커 ID를 이용해서 실제 이미지 파일을 저장
    def _capture_image(self):
        """저장된 마지막 프레임과 마커 ID를 사용하여 이미지를 캡처합니다."""
        # 만약 마지막 프레임이나 마커 ID가 없다면(아직 감지된 적 없음),
        # 캡처를 진행하지 않고 경고 로그만 남김
        if self._last_bgr_img is None or self._last_marker_id is None:
            rospy.logwarn("[ArucoTrigger] Cannot capture image: last frame or ID is missing.")
            return

        # mid: 마지막으로 감지된 마커 ID
        mid = self._last_marker_id

        # 캡처 카운트를 증가시키고, 해당 마커 ID에 대한 누적 저장 횟수를 관리
        # 처음 보는 mid라면 딕셔너리에 0으로 초기화
        if mid not in self.capture_count:
            self.capture_count[mid] = 0

        # 해당 마커 ID에 대한 캡처 횟수를 1 증가
        self.capture_count[mid] += 1

        # 파일 이름을 "triggered_object<ID>_<번호>.jpg" 형태로 구성
        # 예: triggered_object3_1.jpg (id=3, 첫 번째 캡처)
        filename = os.path.join(self.save_dir, "triggered_object{}_{}.jpg".format(mid, self.capture_count[mid]))

        # OpenCV의 imwrite로 실제 이미지 파일을 디스크에 저장
        cv2.imwrite(filename, self._last_bgr_img)

        # 저장 완료를 ROS 로그에 출력
        rospy.loginfo("[ArucoTrigger] Triggered image saved: {}".format(filename))


    # observe_and_maybe_trigger:
    #  - 매 프레임마다 호출되어, 현재 모드가 LANE_FOLLOW일 때 ArUco 마커를 감지
    #  - 조건을 만족하면 self.pending_actions에 액션들을 쌓고 모드를 EXECUTE_ACTION으로 변경
    def observe_and_maybe_trigger(self, bgr_img):
        # 가장 최근 프레임을 저장합니다. (step()에서 캡처 액션을 위해 사용)
        # 캡처 시점에는 카메라 콜백이 이미 지나갔을 수 있으므로, 여기서 항상 최신 프레임을 보관
        self._last_bgr_img = bgr_img

        # 현재 모드가 LANE_FOLLOW가 아닐 경우(이미 액션 수행 중 등),
        # 새롭게 마커를 트리거하지 않고 바로 반환
        if self.mode != "LANE_FOLLOW":
            return

        # 현재 시간(초)을 얻어 쿨다운 계산 등에 사용
        now = time.time()

        # 현재 프레임에서 ArUco 마커들을 검출
        dets = self.detector.detect_ids(bgr_img)

        # 아무 마커도 검출되지 않았다면, 연속 카운트를 초기화하고, 마지막 마커 id도 초기화 후 반환
        if not dets:
            self._consec = {}
            self._last_marker_id = None
            return

        # 검출된 마커들 중에서 _gate 조건(면적, y범위)을 통과하는 것만 남긴다
        dets = [d for d in dets if self._gate(d)]

        # gate를 통과한 마커가 없다면, 연속 카운트와 마지막 마커 id를 초기화 후 반환
        if not dets:
            self._consec = {}
            self._last_marker_id = None
            return

        # 여러 개가 남았다면, 면적이 가장 큰(보통 카메라에 가장 가깝거나 확실한) 마커 하나를 선택
        det = max(dets, key=lambda x: x["area"])

        # 선택된 마커의 id를 mid에 저장
        mid = det["id"]

        # 마지막으로 감지된 유효 마커 ID를 저장합니다.
        # 이후 캡처 이미지 파일 이름 등에 사용됨
        self._last_marker_id = mid

        # 기존의 self.capture_target_ids를 이용한 이미지 저장 로직은 제거됨
        # (지금은 rules와 capture 액션에 따라 캡처를 트리거)

        # 연속 프레임 카운트:
        # 같은 마커 id(mid)가 연속으로 몇 프레임 동안 조건을 만족했는지 기록
        self._consec[mid] = self._consec.get(mid, 0) + 1

        # 현재 프레임에서 선택된 마커(mid)가 아닌 다른 마커들의 연속 카운트는 0으로 리셋
        for k in list(self._consec.keys()):
            if k != mid:
                self._consec[k] = 0

        # 아직 required_consecutive(예: 3프레임 연속) 이상 보지 못했다면,
        # 노이즈 가능성이 있으므로 트리거하지 않고 반환
        if self._consec[mid] < self.required_consecutive:
            return

        # 쿨다운 확인:
        # 마지막으로 이 마커 id(mid)를 트리거한 시간과 지금(now)의 차이가
        # 해당 마커의 쿨다운 시간보다 작으면, 아직 트리거할 수 없는 상태
        last = self.last_trigger_times.get(mid, 0.0)

        # 이 마커 id의 개별 쿨다운 시간을 가져오고, 없으면 기본값을 사용
        cooldown = self.cooldown_per_id.get(mid, self.cooldown_default)

        # (now - last)가 cooldown보다 작으면, 쿨다운이 끝나지 않았으므로 그냥 반환
        if (now - last) < cooldown:
            return

        # 등장 횟수 카운트:
        # 이 마커 id(mid)가 지금까지 몇 번째로 트리거 조건을 만족했는지 계산
        nth = self.seen_counts.get(mid, 0) + 1

        # 딕셔너리에 최신 등장 횟수를 반영
        self.seen_counts[mid] = nth

        # rules에서 (mid, nth) 조합에 해당하는 액션이 정의되어 있는지 확인
        if (mid in self.rules) and (nth in self.rules[mid]):
            # actions: 튜플 하나 또는 튜플 리스트일 수 있음
            actions = self.rules[mid][nth]

            # 만약 단일 튜플 형태라면 리스트로 감싸서 통일된 형식으로 맞춤
            if isinstance(actions, tuple):
                actions = [actions]

            # pending_actions에 복사해 두고, 앞으로 step()에서 하나씩 실행하게 됨
            self.pending_actions = list(actions)

            # 모드를 EXECUTE_ACTION으로 바꿔서,
            # 차선 추종이 아니라 액션 실행 루틴으로 넘어가도록 함
            self.mode = "EXECUTE_ACTION"

            # 이번에 트리거한 시간을 기록 (마커별 쿨다운 관리용)
            self.last_trigger_times[mid] = now

            # 연속 카운트는 초기화해 다음 트리거를 준비
            self._consec = {}


    # _rotate_in_place:
    # 주어진 방향(direction)과 각도(degrees)에 따라 로봇을 제자리에서 회전시키는 함수
    def _rotate_in_place(self, direction, degrees, ang_speed=1.0):
        # Twist 메시지 생성 (선속도/각속도 명령)
        msg = Twist()

        # 회전만 할 것이므로 선속도(linear.x)는 0으로 설정
        msg.linear.x = 0.0

        # direction 값에 따라 회전 방향과 duration(얼마 동안 회전할지)을 설정
        if direction == "right":
            # 오른쪽 회전: z축 각속도를 음수로 설정 (좌표계 기준)
            msg.angular.z = -abs(ang_speed)
            # 회전 시간 = (요구 회전 각도[rad]) / 각속도[rad/s]
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        elif direction == "left":
            # 왼쪽 회전: z축 각속도를 양수로 설정
            msg.angular.z = abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        elif direction == "turn":
            # "turn"은 고정 각도(예: 120도)를 도는 특수 명령
            msg.angular.z = abs(ang_speed)
            duration = 120.0 * math.pi/180.0 / abs(ang_speed)
        elif direction == "turn1":
            # "turn1"은 오른쪽 회전이지만 degrees를 그대로 쓰는 모드
            msg.angular.z = -abs(ang_speed)
            duration = abs(degrees) * math.pi/180.0 / abs(ang_speed)
        else:
            # 정의되지 않은 direction이면 아무 것도 하지 않고 반환
            return

        # 20 Hz(초당 20번)로 publish하기 위한 ROS Rate 객체 생성
        rate = rospy.Rate(20)

        # 현재 시각을 t0로 기록
        t0 = rospy.Time.now().to_sec()

        # (현재 시간 - t0)가 duration보다 작은 동안 계속 회전 명령을 publish
        # 동시에, 노드가 종료되지 않았는지(rospy.is_shutdown)도 체크
        while (rospy.Time.now().to_sec() - t0) < duration and (not rospy.is_shutdown()):
            # 회전 명령 publish
            self.drive_pub.publish(msg)
            # 다음 루프까지 잠시 대기 (20Hz 유지)
            rate.sleep()

        # 회전이 끝난 후에는 Twist() (모든 속도 0) 를 publish해서 로봇을 정지시킴
        self.drive_pub.publish(Twist())


    # step:
    #  - mode가 EXECUTE_ACTION일 때, pending_actions에 쌓여 있는 액션들을 하나씩 꺼내 실행
    #  - 모든 액션이 끝나면 다시 LANE_FOLLOW 모드로 복귀
    def step(self):
        # 현재 모드가 EXECUTE_ACTION이고, 실행할 pending_actions가 남아있는 경우에만 수행
        if self.mode == "EXECUTE_ACTION" and self.pending_actions:
            # 액션 실행 전에 잠깐 정지 명령을 보내서 움직임을 안정화
            self.drive_pub.publish(Twist())

            # 약간의 시간(0.15초) 대기 – 로봇이 완전히 멈출 시간을 주기 위함
            rospy.sleep(0.15)

            # pending_actions 리스트에서 맨 앞의 액션 하나를 꺼냄
            # action 형식: (direction, degrees)
            direction, deg = self.pending_actions.pop(0)

            # 캡처 액션 처리:
            # direction 문자열이 "capture"라면 회전이 아니라 이미지 저장을 수행
            if direction == "capture":
                self._capture_image()
            else:
                # 그 외의 경우("right", "left", "turn", "turn1" 등)은 회전 동작 실행
                self._rotate_in_place(direction, deg, ang_speed=1.0)

            # 이번 액션을 실행한 후, 남은 pending_actions가 없으면
            # 모든 액션이 완료된 것이므로 모드를 다시 LANE_FOLLOW로 돌려놓음
            if not self.pending_actions:
                self.mode = "LANE_FOLLOW"
