import os
import time
from collections import defaultdict

import cv2
from ultralytics import YOLO

try:
    import torch
except ImportError:
    print("Warning: torch module not found. Install torch for optimal performance.")
    torch = None

# Choose inference device
if torch and torch.cuda.is_available():
    INFERENCE_DEVICE = 0
    print(f"YOLO will use GPU (Device: {INFERENCE_DEVICE}).")
else:
    INFERENCE_DEVICE = 'cpu'
    print("YOLO will use CPU.")

# Load YOLO model
MODEL = YOLO('config/best_rokaf.pt')
CONFIDENCE_THRESHOLD = 0.3 # YOLO 모델이 예측한 박스의 신뢰도(confidence)가 이 값보다 낮으면 그 박스를 무시하겠다는 기준

# ArUcoTrigger에서 YOLO_capture로 저장하는 디렉토리
# (원하면 여기 경로만 바꿔주면 됨)
IMAGE_DIR = os.path.expanduser(
    "~/catkin_ws/src/ROKAF_Autonomous_Car_2025/yolo_images"
)

# 새 파일 체크 주기(초)
SLEEP_SEC = 1.0


def main():
    # 클래스별 누적 카운트: {"car": 3, "tank": 1, ...}
    class_counts = defaultdict(int)

    # 이미 처리한(추론을 돌린) 파일 이름을 저장해두는 집합: 매번 디렉토리 스캔할 때 새 파일만 골라내기 위함.
    processed_files = set()

    if not os.path.isdir(IMAGE_DIR):
        raise RuntimeError("Image directory does not exist: {}".format(IMAGE_DIR))

    print("[YOLO] Watching directory:", IMAGE_DIR)
    print("[YOLO] Press Ctrl+C to stop.\n")

    while True:
        # 디렉토리 안의 이미지 파일 리스트: 확장자가 jpg/jpeg/png인 파일을 모두 모으고 정렬(시간 순서 보장 목적) -> 이미지가 만들어진 순서대로 로그가 찍히고, 누적 카운트도 시간 흐름과 일관되게 올라가서 모니터링이 수월
        filenames = [
            f for f in os.listdir(IMAGE_DIR)
            if f.lower().endswith((".jpg", ".jpeg", ".png"))
        ]
        filenames.sort()

        # 아직 처리하지 않은 새 파일만 선택
        new_files = [f for f in filenames if f not in processed_files]

        if new_files:
            for fname in new_files:
                full_path = os.path.join(IMAGE_DIR, fname)
                img = cv2.imread(full_path)

                if img is None: # 깨진 파일에 대해 무한 경고 반복되는 거 방지하기 위함
                    print("[WARN] Failed to read image:", full_path)
                    processed_files.add(fname)
                    continue

                # YOLO inference(Yolo 모델로 추론 수행): 처리하지 않은 새 파일에 대해서만 YOLO 추론 수행
                results = MODEL(img, verbose=False, device=INFERENCE_DEVICE)

                # 이번 이미지에서의 클래스별 카운트
                per_image_counts = defaultdict(int)

                for r in results: # UltraLytics YOLO의 결과 각 요소는 한 장의 이미지에 대한 예측 담고 있음 -> 이를 순회
                    if r.boxes is None:
                        continue

                    for box in r.boxes: # 감지된 각 bounding box에 대해 반복
                        conf = float(box.conf[0]) # 해당 박스의 신뢰도 점수(confidence score)
                        if conf < CONFIDENCE_THRESHOLD:
                            continue # 신뢰도가 기준치보다 낮으면 무시

                        class_id = int(box.cls[0])
                        label = MODEL.names[class_id] # 클래스 이름(예: "enem", "alli") 가져옴.

                        # 누적 카운트
                        class_counts[label] += 1 # 전체 누적 수 증가
                        per_image_counts[label] += 1 # 이번 이미지에서의 수 증가

                processed_files.add(fname) # 이 파일은 처리 완료했으므로 집합에 추가

                # 결과 출력: 
                print("=== Image processed:", fname, "===")
                if per_image_counts:
                    print("  [per-image counts]")
                    for label, c in sorted(per_image_counts.items()):
                        print("   - {}: {}".format(label, c))
                else:
                    print("  No objects detected over conf {:.2f}".format(CONFIDENCE_THRESHOLD))

                print("\n  [cumulative counts so far]")
                for label, c in sorted(class_counts.items()):
                    print("   - {}: {}".format(label, c))
                print("================================\n")

        # 새 이미지가 없으면 잠깐 쉼
        time.sleep(SLEEP_SEC)


if __name__ == "__main__":
    main()
