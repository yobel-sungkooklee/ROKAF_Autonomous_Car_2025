# 실행 명령어
```bash
roslaunch mm_cam usb_cam.launch
roslaunch omo_r1mini_bringup omo_r1mini_bringup.launch
python lane_follower-PID.py
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