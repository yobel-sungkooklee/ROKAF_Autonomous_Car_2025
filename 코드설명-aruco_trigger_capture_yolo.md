# 코드 설명 – `aruco_trigger_capture_yolo.py`

## 1. 파일 개요
- **역할**: 주행 중 카메라 프레임에서 아루코(Aruco) 마커와 QR을 감지해 **회전, 일반 캡처, YOLO 캡처**와 같은 매크로 액션을 실행한다.
- **연동 지점**: `lane_follower-PID.py`가 카메라 콜백마다 `observe_and_maybe_trigger()`를 호출해 이벤트를 큐에 쌓고, 루프 끝에 `step()`으로 한 액션씩 처리한다.
- **출력**: `/cmd_vel`로 회전 명령을 퍼블리시하고, `~/catkin_ws/src/ROKAF_Autonomous_Car_2025/images` 및 `.../yolo_images`에 캡처 이미지를 저장한다.

## 2. 기존 `aruco_trigger_capture.py` 대비 핵심 업데이트
| 구분 | 기존 버전 | `aruco_trigger_capture_yolo.py` |
| --- | --- | --- |
| **Rule 확장** | 회전 + 일반 캡처 중심 규칙 | `("yolo_capture", 0)` 같은 액션을 추가해 ID/n번째 등장에 따라 YOLO용 캡처 시나리오 구성 |
| **저장 경로** | `~/.../images`만 사용 | `yolo_images` 전용 디렉터리를 추가로 생성, ID별 순번을 붙여 저장 |
| **카운터** | `capture_count`만 유지 | `capture_count`, `yolo_capture_count`를 분리해 두 종류의 캡처를 독립적으로 관리 |
| **데이터 연계** | 캡처 후 수동 확인 | `yolo_image.py`가 `yolo_images`를 감시하며 자동으로 YOLO 추론 → 실시간 피드백 가능 |
| **QR 지원** | 없음 | `_process_qr_codes()`로 pyzbar 감지 및 로그 출력(필요 시 비활성화) |
| **안정성** | 단순 검증 | per-ID 쿨다운/연속 프레임 임계치/면적·y 게이트를 통해 노이즈 감지를 방지 |

## 3. 클래스별 상세
### 3.1 `ArucoDetector`
- `CvBridge`를 통해 ROS `CompressedImage` → OpenCV BGR로 변환.
- `detect_ids(bgr_img)`에서 `cv2.aruco.detectMarkers` 결과를 `(id, center, area)` 리스트로 가공해 반환.
- `(1,4,2)` 형태의 코너 좌표를 `reshape(-1, 2)`로 바꿔 중심과 면적을 계산한다.

### 3.2 `ArucoTrigger`
1. **초기화**
   - `self.rules`: ID별·nth별로 회전/캡처/YOLO 캡처 시퀀스를 정의. 예) `0: {1: [("right", 90)], 2: [("right", 20), ("yolo_capture", 0)]}`.
   - `self.cooldown_per_id`: ID마다 쿨다운(6.5s, 1s 등)을 구분해 연속 트리거를 방지.
   - 디렉터리 준비: `images`, `yolo_images` 두 경로를 만들고 각각 `capture_count`/`yolo_capture_count`로 관리.
   - `_last_bgr_img`, `_last_marker_id`: 캡처 시점에 사용할 “가장 최근 프레임/ID”를 보관.
   - `_consec`, `required_consecutive`: 같은 ID가 연속 프레임에서 안정적으로 보였을 때만 이벤트로 인정.
   - QR 설정: pyzbar 사용 가능 여부 확인 후 `_process_qr_codes()`에서 로그 출력.

2. **핵심 메서드**
   - `_gate(det)`: 면적·y 위치 기준으로 노이즈 마커를 걸러냄.
   - `_capture_image()`: 기존 화재/미션 캡처 저장.
   - `_capture_yolo_image()`: 새로 추가된 함수. `_last_bgr_img`와 `_last_marker_id`를 이용해 `yolo_{ID}_{순번}.jpg`를 저장하고 ROS 로그에 남김.
   - `observe_and_maybe_trigger(bgr_img)`:
     - LANE_FOLLOW 모드일 때만 동작.
     - 최신 프레임/QR 처리 후 ArucoDetector 결과를 필터링.
     - 연속 카운트·쿨다운·rule을 모두 통과하면 `pending_actions` 리스트에 액션을 복사하고 `mode="EXECUTE_ACTION"`으로 전환.
   - `step()`:
     - EXECUTE_ACTION 모드에서만 실행.
     - 먼저 0 Twist를 publish + 0.15초 대기 → 안정화.
     - `pending_actions.pop(0)`으로 액션을 꺼내고 `"capture"`/`"yolo_capture"`/회전을 분기 처리.
     - `pending_actions`가 비면 다시 `LANE_FOLLOW`로 복귀.

## 4. 동작 흐름 (lane_follower-PID.py와 연계)
1. 카메라 콜백마다 `self.aruco_trig.observe_and_maybe_trigger(frame)` → 이벤트 조건 판단.
2. 차선/ PID 제어 후 `self.aruco_trig.step()` → 큐에 쌓인 액션을 한 번에 하나씩 수행.
3. `"yolo_capture"`가 실행되면 `yolo_images`에 새 파일이 생김.
4. `yolo_image.py`가 해당 폴더를 감시해 새 파일마다 Ultralytics YOLO 추론을 수행, 콘솔에 per-image/누적 감지 결과를 출력.

## 5. 발표/문서화 시 강조할 차별점
- **데이터 수집 자동화**: 특정 마커에서 즉시 YOLO 캡처, 이어지는 추론 로그로 실시간 피드백.
- **FSM 안정성**: 연속 프레임 확인 + per-ID 쿨다운 + ROI 게이트로 노이즈 트리거를 억제.
- **확장성**: `self.rules`에 액션 튜플을 더 추가하면 손쉽게 미션 시나리오 변경 가능.
- **QR/로그 기능**: 동일 텍스트 재출력 제한(`qr_log_cooldown`)으로 필요한 정보만 콘솔에 남김.

이 문서를 그대로 PPT 슬라이드(리마인드 → 업그레이드 포인트 → 동작 흐름) 구성에 참고하면 된다.
