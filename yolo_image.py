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
CONFIDENCE_THRESHOLD = 0.3

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

    # 이미 처리한 파일 이름을 저장해두는 집합
    processed_files = set()

    if not os.path.isdir(IMAGE_DIR):
        raise RuntimeError("Image directory does not exist: {}".format(IMAGE_DIR))

    print("[YOLO] Watching directory:", IMAGE_DIR)
    print("[YOLO] Press Ctrl+C to stop.\n")

    while True:
        # 디렉토리 안의 이미지 파일 리스트
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

                if img is None:
                    print("[WARN] Failed to read image:", full_path)
                    processed_files.add(fname)
                    continue

                # YOLO inference
                results = MODEL(img, verbose=False, device=INFERENCE_DEVICE)

                # 이번 이미지에서의 클래스별 카운트
                per_image_counts = defaultdict(int)

                for r in results:
                    if r.boxes is None:
                        continue

                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf < CONFIDENCE_THRESHOLD:
                            continue

                        class_id = int(box.cls[0])
                        label = MODEL.names[class_id]

                        # 누적 카운트
                        class_counts[label] += 1
                        per_image_counts[label] += 1

                processed_files.add(fname)

                # 결과 출력
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
