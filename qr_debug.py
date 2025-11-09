#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
단일 이미지로 pyzbar QR 디코딩을 검증하는 스크립트.
사용법: python2 qr_debug.py --image /path/to/qr.png
"""

import argparse
import sys

import cv2
from pyzbar import pyzbar


def decode_qr(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise IOError("이미지를 불러올 수 없습니다: {}".format(image_path))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = pyzbar.decode(gray)
    if not results:
        print("QR 코드가 감지되지 않았습니다.")
        return

    print("QR 후보 개수:", len(results))
    for idx, obj in enumerate(results, 1):
        payload = obj.data.decode("utf-8", errors="ignore") if obj.data else ""
        print("[{}] type={}, data={!r}".format(idx, obj.type, payload))


def main():
    parser = argparse.ArgumentParser(description="pyzbar QR 디코딩 테스트")
    parser.add_argument("--image", required=True, help="QR 이미지 경로")
    args = parser.parse_args()

    try:
        decode_qr(args.image)
    except Exception as exc:
        print("디코딩 중 오류:", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
