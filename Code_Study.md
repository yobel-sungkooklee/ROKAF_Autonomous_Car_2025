# lane_follower-PID.py 코드 정리

• - if __name__ == "__main__": 블록에서 ROS가 내려가지 않은 상태라면 lane_detect() 
    클래스를 한 번 인스턴스화합니다. 이 순간 __init__이 즉시 실행됩니다.           
  - __init__ 안에서 하는 일 순서:                                                  
      1. self.bridge = CvBridge()로 ROS 압축 이미지를 OpenCV 이미지로 바꿀 준비.   
      2. rospy.init_node('lane_detection_node')로 노드 등록.                       
      3. /usb_cam/image_raw/compressed를 camera_callback으로 구독(큐 1,            
         tcp_nodelay=True).                                                        
      4. /cmd_vel 퍼블리셔 생성.                                                   
      5. Twist 메시지 인스턴스, 기본 속도/가중치, PID 컨트롤러, ArucoTrigger를 초  
         기화.                                                                     
  - 이후 rospy.spin()이 돌면서 카메라 토픽에 새 메시지가 올 때마다 camera_callback 
    이 자동 호출됩니다. 이 콜백은 매 프레임 다음 순서를 실행합니다:                
      1. CvBridge로 이미지를 self.image에 저장.                                    
      2. self.aruco_trig.observe_and_maybe_trigger(self.image)로 현재 프레임에서 아         루코/QR 이벤트 감지.                                                      
      3. self.lane_detect()를 호출.                                                
  - lane_detect()는 차선 처리 파이프라인 전체를 수행합니다: 원본 디버그 표시 →     
    warpping으로 BEV 변환 → 블러·색 필터·이진화·Canny → cv2.HoughLines로 직선 후보 
    생성 → high_level_detect 호출로 슬라이딩 윈도/2차 피팅 → heading/lateral 오차  
    계산 → PID 업데이트와 /cmd_vel 퍼블리시 → self.aruco_trig.step()으로 대기 중 액    션 실행.                                                                       
  - high_level_detect()는 lane_detect() 내부에서 호출되며, Hough 결과를 히스토그램 
    으로 분석한 뒤 15개의 슬라이딩 윈도를 통해 차선 중심점들을 추적하고 2차 다항식 
    을 계산합니다.                                                                 
  - PIDController.update()는 lane_detect()가 조향 명령을 만들 때마다 호출되어 조향 
    각속도를 출력합니다.                                                           
    결국 클래스 인스턴스가 만들어지면, 카메라 콜백→lane_detect()→PID→아루코 액션 순
    환이 ROS 스핀 루프 안에서 지속적으로 반복됩니다.   



# aruco_trigger_capture_yolo.py 코드 정리
• 구조 개요
                                                                                   
  - 상단에서 OpenCV ArUco 모듈을 버전별로 호환되게 불러오고                        
    (cv2.aruco.getPredefinedDictionary, DetectorParameters_create), 실패 시 구버전 
    API로 대체합니다. 이렇게 해 놓으면 OpenCV 3/4 어디서든 같은 코드를 돌릴 수 있습    니다.                                                                          
                                                                                   
  ArucoDetector                                                                    
                                                                                   
  - __init__에서 CvBridge를 만들어 ROS 메시지를 OpenCV 이미지로 변환 준비.
  - detect_ids(bgr_img)는 BGR 이미지를 그레이로 바꾼 뒤 cv2.aruco.detectMarkers로  
    마커를 찾습니다. 각 마커에 대해 꼭짓점(4×2) 배열을 평균/최대·최소로 가공해 중심    (cx, cy)와 대략적 면적 w*h를 구하고, {"id": int(i), "center": (cx, cy), "area":    area} 딕셔너리 리스트로 돌려줍니다.                                            
                                                                                   
  ArucoTrigger 초기화                                                              
                                                                                   
  - self.rules: ID별·nth별로 실행할 액션을 정의한 테이블. 예: ID 0을 처음 보면 오른    쪽 90도, 두 번째 보면 오른쪽 20도 후 YOLO 캡처.                                
  - self.detector = ArucoDetector()로 앞서 정의한 헬퍼 사용.                       
  - self.drive_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)로 /cmd_vel 퍼 
    블리셔 생성. 회전 명령과 캡처 전후 정지 명령을 바로 publish.                   
  - 상태 변수들:                                                                   
      - self.mode: "LANE_FOLLOW" 또는 "EXECUTE_ACTION".                            
      - self.pending_actions: (direction, degrees) 튜플 목록.                      
      - self.seen_counts: ID별 nth 등장 횟수.                                      
      - self.cooldown_default, self.cooldown_per_id, self.last_trigger_times: ID마 
        다 쿨다운 관리.                                                            
  - 캡처 관련:
      - self.save_dir, self.yolo_save_dir를 ~/catkin_ws/src/                       
        ROKAF_Autonomous_Car_2025/...에 생성해 화재·YOLO 이미지 저장.              
      - self.capture_count, self.yolo_capture_count로 ID별 저장 수를 기록.         
      - _last_bgr_img(최근 프레임)과 _last_marker_id(최근 유효 ID)를 저장해 캡처 시        점에 사용.                                                                 
  - 감지 필터:                                                                     
      - required_consecutive=3: 같은 ID가 3프레임 연속 통과해야 트리거.            
      - _consec 사전으로 각 ID 연속 프레임 수 추적.                                
      - min_area, min_y, max_y: 너무 작거나(멀리) 화면 위/아래에 있는 마커를 무시. 
  - QR 지원:                                                                       
      - pyzbar를 import해 두고, 실패 시 경고 후 QR 기능 비활성화.                  
      - _qr_last_logged, qr_log_cooldown으로 같은 QR 텍스트를 너무 자주 찍지 않    
        게 함.                                                                     

  유효성 게이트 _gate                                                              
                                                                                   
  - 면적이 min_area 이상이고 y좌표가 [min_y, max_y] 안에 들어오면 True. 그렇지 않으    면 노이즈로 간주해 버립니다.                                                   
                                                                                   
  이미지 저장 함수                                                                 
                                                                                   
  - _capture_image: _last_bgr_img와 _last_marker_id가 있을 때 triggered_object{ID} 
    _{count}.jpg 이름으로 저장. 없으면 경고.                                       
  - _capture_yolo_image: YOLO용 디렉토리에 yolo_{ID}_{count}.jpg 파일 생성.        
                                                                                   
  observe_and_maybe_trigger                                                        
                                                                                   
  1. 최신 프레임을 _last_bgr_img에 저장, QR 코드도 분석.                           
  2. 현재 모드가 LANE_FOLLOW가 아니면(이미 액션 수행 중) 그냥 반환.                
  3. 감지 리스트를 얻고, 없으면 _consec와 _last_marker_id를 리셋 후 종료.          
  4. _gate 조건을 통과한 마커만 남김. 없다면 리셋 후 종료.                         
  5. 가장 면적 큰 마커 하나를 선택(max(dets, key=area)), ID를 _last_marker_id에    
     저장.                                                                         
  6. _consec[mid] 카운트를 늘리고, 다른 ID 카운트는 0으로 리셋.                    
     required_consecutive 미만이면 노이즈로 보고 종료.                             
  7. ID별 쿨다운(last_trigger_times, cooldown_per_id)을 검사해 아직 쿨다운이면     
     종료.
  8. seen_counts[mid]를 증가시켜 nth 등장 횟수를 업데이트.
  9. self.rules에서 (mid, nth) 조합을 찾고, 있으면 리스트 형태로 pending_actions에 
     복사, mode="EXECUTE_ACTION", last_trigger_times[mid]=now, _consec 초기화.     

  _rotate_in_place                                                                 
                                                                                   
  - direction에 따라 Twist.angular.z 부호를 정하고, 주어진 각도를 원하는 각속도    
    (ang_speed, default 1 rad/s)로 나눠 소요 시간을 계산.                          
  - 20Hz 루프로 해당 시간 동안 명령을 publish한 뒤 0 속도로 정지.                  
  - direction "turn"/"turn1" 등 특수 명령도 처리.                                  
                                                                                   
  step
                                                                                   
  - 모드가 EXECUTE_ACTION이고 pending_actions가 남아 있을 때만 실행.               
  - 먼저 0 Twist를 publish하고 0.15초 기다려 로봇을 안정화.                        
  - 하나의 (direction, deg)를 팝:                                                  
      - "capture" → _capture_image()                                               
      - "yolo_capture" → _capture_yolo_image()                                     
      - 그 외 → _rotate_in_place(direction, deg)                                   
  - 큐가 비면 mode를 "LANE_FOLLOW"로 되돌려 차선 추종 루틴으로 복귀.               
                                                                                   
  _process_qr_codes                                                                
                                                                                   
  - pyzbar로 QR을 디코드하고, 새 텍스트가 감지되면 1초 간격으로만 ROS 로그에 출력. 
    QR이 없거나 오류가 나면 조용히 무시.                                           
                                                                                   
  전체적으로 observe_and_maybe_trigger()가 “해야 할 일(pending_actions)”을 쌓고    
  step()이 그 일을 하나씩 실행합니다. 회전 직전에 로봇을 멈추고, 회전 후에는 자동으
  로 다시 LANE_FOLLOW로 돌아가므로 차선 주행 루프에 자연스럽게 녹아 있습니다.• 구조 개요
                                                                                   
  - 상단에서 OpenCV ArUco 모듈을 버전별로 호환되게 불러오고                        
    (cv2.aruco.getPredefinedDictionary, DetectorParameters_create), 실패 시 구버전 
    API로 대체합니다. 이렇게 해 놓으면 OpenCV 3/4 어디서든 같은 코드를 돌릴 수 있습    니다.                                                                          
                                                                                   
  ArucoDetector                                                                    
                                                                                   
  - __init__에서 CvBridge를 만들어 ROS 메시지를 OpenCV 이미지로 변환 준비.
  - detect_ids(bgr_img)는 BGR 이미지를 그레이로 바꾼 뒤 cv2.aruco.detectMarkers로  
    마커를 찾습니다. 각 마커에 대해 꼭짓점(4×2) 배열을 평균/최대·최소로 가공해 중심    (cx, cy)와 대략적 면적 w*h를 구하고, {"id": int(i), "center": (cx, cy), "area":    area} 딕셔너리 리스트로 돌려줍니다.                                            
                                                                                   
  ArucoTrigger 초기화                                                              
                                                                                   
  - self.rules: ID별·nth별로 실행할 액션을 정의한 테이블. 예: ID 0을 처음 보면 오른    쪽 90도, 두 번째 보면 오른쪽 20도 후 YOLO 캡처.                                
  - self.detector = ArucoDetector()로 앞서 정의한 헬퍼 사용.                       
  - self.drive_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)로 /cmd_vel 퍼 
    블리셔 생성. 회전 명령과 캡처 전후 정지 명령을 바로 publish.                   
  - 상태 변수들:                                                                   
      - self.mode: "LANE_FOLLOW" 또는 "EXECUTE_ACTION".                            
      - self.pending_actions: (direction, degrees) 튜플 목록.                      
      - self.seen_counts: ID별 nth 등장 횟수.                                      
      - self.cooldown_default, self.cooldown_per_id, self.last_trigger_times: ID마 
        다 쿨다운 관리.                                                            
  - 캡처 관련:
      - self.save_dir, self.yolo_save_dir를 ~/catkin_ws/src/                       
        ROKAF_Autonomous_Car_2025/...에 생성해 화재·YOLO 이미지 저장.              
      - self.capture_count, self.yolo_capture_count로 ID별 저장 수를 기록.         
      - _last_bgr_img(최근 프레임)과 _last_marker_id(최근 유효 ID)를 저장해 캡처 시        점에 사용.                                                                 
  - 감지 필터:                                                                     
      - required_consecutive=3: 같은 ID가 3프레임 연속 통과해야 트리거.            
      - _consec 사전으로 각 ID 연속 프레임 수 추적.                                
      - min_area, min_y, max_y: 너무 작거나(멀리) 화면 위/아래에 있는 마커를 무시. 
  - QR 지원:                                                                       
      - pyzbar를 import해 두고, 실패 시 경고 후 QR 기능 비활성화.                  
      - _qr_last_logged, qr_log_cooldown으로 같은 QR 텍스트를 너무 자주 찍지 않    
        게 함.                                                                     

  유효성 게이트 _gate                                                              
                                                                                   
  - 면적이 min_area 이상이고 y좌표가 [min_y, max_y] 안에 들어오면 True. 그렇지 않으    면 노이즈로 간주해 버립니다.                                                   
                                                                                   
  이미지 저장 함수                                                                 
                                                                                   
  - _capture_image: _last_bgr_img와 _last_marker_id가 있을 때 triggered_object{ID} 
    _{count}.jpg 이름으로 저장. 없으면 경고.                                       
  - _capture_yolo_image: YOLO용 디렉토리에 yolo_{ID}_{count}.jpg 파일 생성.        
                                                                                   
  observe_and_maybe_trigger                                                        
                                                                                   
  1. 최신 프레임을 _last_bgr_img에 저장, QR 코드도 분석.                           
  2. 현재 모드가 LANE_FOLLOW가 아니면(이미 액션 수행 중) 그냥 반환.                
  3. 감지 리스트를 얻고, 없으면 _consec와 _last_marker_id를 리셋 후 종료.          
  4. _gate 조건을 통과한 마커만 남김. 없다면 리셋 후 종료.                         
  5. 가장 면적 큰 마커 하나를 선택(max(dets, key=area)), ID를 _last_marker_id에    
     저장.                                                                         
  6. _consec[mid] 카운트를 늘리고, 다른 ID 카운트는 0으로 리셋.                    
     required_consecutive 미만이면 노이즈로 보고 종료.                             
  7. ID별 쿨다운(last_trigger_times, cooldown_per_id)을 검사해 아직 쿨다운이면     
     종료.
  8. seen_counts[mid]를 증가시켜 nth 등장 횟수를 업데이트.
  9. self.rules에서 (mid, nth) 조합을 찾고, 있으면 리스트 형태로 pending_actions에 
     복사, mode="EXECUTE_ACTION", last_trigger_times[mid]=now, _consec 초기화.     

  _rotate_in_place                                                                 
                                                                                   
  - direction에 따라 Twist.angular.z 부호를 정하고, 주어진 각도를 원하는 각속도    
    (ang_speed, default 1 rad/s)로 나눠 소요 시간을 계산.                          
  - 20Hz 루프로 해당 시간 동안 명령을 publish한 뒤 0 속도로 정지.                  
  - direction "turn"/"turn1" 등 특수 명령도 처리.                                  
                                                                                   
  step
                                                                                   
  - 모드가 EXECUTE_ACTION이고 pending_actions가 남아 있을 때만 실행.               
  - 먼저 0 Twist를 publish하고 0.15초 기다려 로봇을 안정화.                        
  - 하나의 (direction, deg)를 팝:                                                  
      - "capture" → _capture_image()                                               
      - "yolo_capture" → _capture_yolo_image()                                     
      - 그 외 → _rotate_in_place(direction, deg)                                   
  - 큐가 비면 mode를 "LANE_FOLLOW"로 되돌려 차선 추종 루틴으로 복귀.               
                                                                                   
  _process_qr_codes                                                                
                                                                                   
  - pyzbar로 QR을 디코드하고, 새 텍스트가 감지되면 1초 간격으로만 ROS 로그에 출력. 
    QR이 없거나 오류가 나면 조용히 무시.                                           
                                                                                   
  전체적으로 observe_and_maybe_trigger()가 “해야 할 일(pending_actions)”을 쌓고    
  step()이 그 일을 하나씩 실행합니다. 회전 직전에 로봇을 멈추고, 회전 후에는 자동으
  로 다시 LANE_FOLLOW로 돌아가므로 차선 주행 루프에 자연스럽게 녹아 있습니다.


      def step(self):

# yolo_image.py
  - ArUco 트리거가 저장한 yolo_images 폴더를 감시하면서, 새 이미지가 생길   
    때마다 미리 지정한 YOLO 모델로 추론하고 결과를 터미널에 출력하는 독립 스크립트입니다.

  - 시작 시 torch가 있으면 GPU(device=0)를, 없으면 CPU를 사용하도록 설정하고, config/best_rokaf.pt 가중치를 불러옵니다.
  - IMAGE_DIR는 기본적으로 ~/catkin_ws/src/ROKAF_Autonomous_Car_2025/yolo_images이며, 폴더가 없다면 에러를 내고 종료합니다.
  - 메인 루프에서는 1초마다(SLEEP_SEC = 1.0) 디렉터리의 JPG/PNG 파일 목록을 읽어서,
  - 이미지가 없으면 1초 쉬었다가 다시 확인합니다.

  즉, lane_follower-PID.py/ArUcoTrigger가 캡처를 쌓아주고, yolo_image.py가 그 폴더를 실시간으로 
  감시하며 “캡처 → 즉시 추론 → 로그 출력” 흐름을 만들어 줍니다. 원하는 클래스별 빈도를 한눈에 모니터링하고 데이터 품질을 바로 확인할 수 있는 도구입니다.

    - MODEL = YOLO('config/best_rokaf.pt'): 로컬 가중치 파일을 로드해 모델 인스턴스를 생성합니다. 
    CONFIDENCE_THRESHOLD = 0.3은 필터링 기준입니다.                                             
  - IMAGE_DIR = os.path.expanduser("~/catkin_ws/src/ROKAF_Autonomous_Car_2025/yolo_images"):    
    ArUco 트리거가 이미지를 저장하는 폴더를 지정합니다. 필요하면 이 경로만 바꾸면 됩니다.       
  - SLEEP_SEC = 1.0: 새 파일 감시 루프의 슬립 시간(초)입니다.                                   
  - def main():: 전체 감시/추론 로직을 담는 함수입니다.                                         
      - class_counts = defaultdict(int): 클래별 누적 감지 수를 담는 dict.
      - processed_files = set(): 이미 처리한 파일명을 저장할 집합.                              
      - 폴더 존재 확인: if not os.path.isdir(IMAGE_DIR): raise RuntimeError(...).               
      - 초기 안내 출력: 감시 폴더와 중단 방법을 알립니다.                                       
      - while True: 무한 루프 시작.                                                             
          - filenames = [...]: 디렉터리 내 확장자가 jpg/jpeg/png인 파일을 모두 모으고 정렬합니다            (시간 순서 보장 목적).                                                              
          - new_files = [f for f in filenames if f not in processed_files]: 아직 처리하지 않은  
            파일만 골라냅니다.                                                                  
          - if new_files:가 참이면 새 파일마다 반복:                                            
              - full_path = os.path.join(...)로 절대경로 작성.                                  
              - img = cv2.imread(full_path)로 이미지 로드. 실패 시 경고 출력 후 해당 파일을     
                processed로 표시하고 건너뜁니다.                                                
              - results = MODEL(img, verbose=False, device=INFERENCE_DEVICE): YOLO 추론을 실행합                니다.                                                                           
              - per_image_counts = defaultdict(int): 이번 이미지에서 클래스별 카운트를 누적     
                할 dict.                                                                        
              - for r in results:: Ultralytics 결과 객체마다 반복.                              
                  - if r.boxes is None: continue: 탐지 박스가 없으면 넘어갑니다.                
                  - 박스 루프: 각 box에서 confidence, class id, label을 추출.                   
                  - if conf < CONFIDENCE_THRESHOLD: continue: 낮은 신뢰도는 건너뜁니다.         
                  - class_counts[label] += 1, per_image_counts[label] += 1: 누적/이미지별 카운트                    를 각각 증가시킵니다.                                                       
              - processed_files.add(fname)로 중복 처리 방지.                                    
              - 결과 출력:                                                                      
                  - "=== Image processed: fname ===" 헤더                                       
                  - per-image 카운트가 있으면 label: count 식으로 출력, 없으면 “No objects      
                    detected…” 메시지.                                                          
                  - 누적 카운트도 정렬된 순서로 label: count를 보여주고 구분선을 출력합니다.    
          - 새 파일이 없으면 루프 마지막에서 time.sleep(SLEEP_SEC)로 1초 대기 후 다시 폴더를 스 
            캔합니다.                                                                           
  - 모듈이 직접 실행될 때(if __name__ == "__main__":) main()을 호출해 감시 루프를 시작합니다.   
 