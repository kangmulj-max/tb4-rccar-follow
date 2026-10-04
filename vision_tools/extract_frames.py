#!/usr/bin/env python3
"""영상 → 프레임 이미지 추출 (ffmpeg 없이 OpenCV 만 사용).

예) python extract_frames.py S1.mp4 frames/S1 --fps 0.5
    → frames/S1/S1_0001.jpg, S1_0002.jpg ... (2 초에 1 장)
"""
import argparse
import os

import cv2


def imwrite(path, img):
    """한글 경로에서도 저장되도록 (cv2.imwrite 는 Windows 한글 경로에서 조용히 실패함)."""
    ok, buf = cv2.imencode(os.path.splitext(path)[1] or '.jpg', img)
    if ok: buf.tofile(path)
    return ok


def main():
    ap = argparse.ArgumentParser(description='영상에서 일정 간격으로 프레임 저장')
    ap.add_argument('video', help='입력 영상 파일 (mp4, avi ...)')
    ap.add_argument('out', help='출력 폴더')
    ap.add_argument('--fps', type=float, default=0.5, help='초당 저장 장수 (기본 0.5 = 2 초에 1 장)')
    ap.add_argument('--prefix', default=None, help='파일 이름 앞부분 (기본: 영상 파일 이름)')
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f'영상을 열 수 없습니다: {a.video}')
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, round(src_fps / a.fps))
    prefix = a.prefix or os.path.splitext(os.path.basename(a.video))[0]
    os.makedirs(a.out, exist_ok=True)

    i = n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % step == 0:
            n += 1
            imwrite(os.path.join(a.out, f'{prefix}_{n:04d}.jpg'), frame)
        i += 1
    print(f'완료: {n} 장 저장 → {a.out}  (원본 {src_fps:.1f} fps, {i} 프레임, {step} 프레임마다 1 장)')


if __name__ == '__main__':
    main()
