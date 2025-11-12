# 실행 명령어 for ALL Tasks
```bash
roslaunch mm_cam usb_cam.launch
roslaunch omo_r1mini_bringup omo_r1mini_bringup.launch
python lane_follower-PID.py
(myenv) python yolo_image.py 
```
1) roslaunch mm_cam usb_cam.launch
-  ROS에서 USB 카메라 노드를 띄우는 명령
- mm_cam 패키지 안의  usb_cam.launch가 카메라 드라이버 노드를 실행하면서 토픽 이름·해상도·프레임레이트 같은 설정을 로봇 환경에 맞춰 잡아 줌.

2) roslaunch omo_r1mini_bringup omo_r1mini_bringup.launch
- 로봇 자체를 구동하는 기본 bring-up 단계
  - OMO R1 mini 베이스 드라이버와 IMU, 모터 컨트롤러 노드가 올라오고                                 
  - /cmd_vel 같은 제어 토픽을 로봇 하드웨어로 전달할 수 있게 하며                                    
  - 전원/배터리, tf 브로드캐스트 등 로봇 운영에 필요한 기본 노드들이 초기화됩니다

3) python lane_follower-PID.py
- 카메라 영상을 받아 차선을 감지하고 /cmd_vel용 주행 명령(Twist)을 만들면서, 동시에 여러 디버그 창을 띄워 파이프라인 단계를 확인할 수 있게 해 주는 독립 실행 스크립트
  - /usb_cam/image_raw/compressed 토픽을 구독해 영상을 가져오고                                      
  - 원근 변환(BEV), 컬러 필터링, 이진화, 슬라이딩 윈도 같은 처리로 차선을 찾은 뒤                    
  - PID 계산으로 차량의 선회각과 속도를 정해 Twist 메시지로 퍼블리시하며                             
  - 각 단계별 이미지를 OpenCV 윈도우로 띄워 튜닝/디버깅 시각화를 제공합니다.  
  - 같은 스크립트 안에서 ArUcoTrigger(ArucoTriggerCaptureYOLO)를 함께 구동하여, 주행 중 특정 ArUco/QR 마커가 감지되면 회전·이미지 캡처·YOLO 캡처 같은 매크로 액션을 자동으로 실행하고 다시 차선 모드로 복귀합니다.

4) python yolo_image.py
- ~/catkin_ws/src/ROKAF_Autonomous_Car_2025/yolo_images 디렉터리를 감시하다가 새 이미지가 저장되면 즉시 Ultralytics YOLO로 추론을 수행
- 이미지별 감지 수와 누적 감지 수를 터미널에 출력하므로, lane_follower-PID.py에서 캡처된 장면을 바로 확인하면서 데이터 수집/검증을 병행할 수 있음

5) 참고 (lane_follower-PID.py랑 yolo_image.py 합쳐서 동작하는 Flow)
- lane_follower-PID.py는 차선 주행을 하면서 ArUco/QR 감지를 병행. 마커 규칙에 ("yolo_capture", 0) 액션이 포함돼 있으면, 
해당 ID를 봤을 때 _capture_yolo_image가 호출되어서, ~/catkin_ws/src/ROKAF_Autonomous_Car_2025/yolo_images 폴더에 yolo_{id}_{순번}.jpg 파일이 저장됨.

# 핵심 파일들
- aruco_trigger_capture_yolo.py
- yolo_image.py
- lane_follower-PID.py

# 실행 명령어 Just for Testing yolo_image.py
```bash
roslaunch mm_cam usb_cam.launch
roslaunch omo_r1mini_bringup omo_r1mini_bringup.launch
python image_saver.py
python yolo_image.py
```