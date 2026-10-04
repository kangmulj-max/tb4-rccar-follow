#!/usr/bin/env python3
"""기존 모델에서 이어서 학습(파인튜닝). GPU 가 없으면 CPU 로 돌지만 매우 느림 → Colab 권장.

예) python train.py --model my_best.pt --data dataset/data.yaml --epochs 30
결과: runs/detect/<name>/weights/best.pt  → judge.py 의 --new 로 넣어 판정
주의: Roboflow 에서 내보낼 때 증강(augmentation)은 끄세요. YOLO 가 학습 중에 따로 증강합니다.
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser(description='YOLO 파인튜닝')
    ap.add_argument('--model', required=True, help='시작 가중치 (기존 best.pt)')
    ap.add_argument('--data', required=True, help='make_split.py 가 만든 data.yaml')
    ap.add_argument('--epochs', type=int, default=30)
    ap.add_argument('--imgsz', type=int, default=640, help='지금 웹캠 파이프라인(640 자르기)과 같게 유지')
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--name', default='finetune')
    a = ap.parse_args()
    YOLO(a.model).train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, patience=10,
                        name=a.name, exist_ok=True, degrees=0.0)


if __name__ == '__main__':
    main()
