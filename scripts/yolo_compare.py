#!/usr/bin/env python3
"""
YOLO 모델 비교 — MODELS 환경변수 (기본 "11 12", 예: MODELS="8 11 12") (실물 데이터셋 amr_dataset.zip valid/test + 추론 속도).
  source ~/yolo_venv/bin/activate; python yolo_compare.py
"""
import os, time, zipfile
import numpy as np
from ultralytics import YOLO

ROOT = os.path.expanduser('~/tb4_follow_sim'); DS = f'{ROOT}/dataset'
if not os.path.isdir(DS):
    zipfile.ZipFile(next(p for p in (os.environ.get('TB4_DATASET_ZIP', ''), f'{ROOT}/data/amr_dataset.zip', '/mnt/c/Users/kangm/Downloads/amr_dataset.zip') if p and os.path.exists(p))).extractall(DS)
yaml = f'{DS}/data_abs.yaml'
open(yaml, 'w').write(f"path: {DS}\ntrain: train/images\nval: valid/images\ntest: test/images\nnc: 2\nnames: ['car', 'dummy']\n")

img = np.random.randint(0, 255, (704, 704, 3), np.uint8)
for v in os.environ.get('MODELS', '11 12').split():
    m = YOLO(f'{ROOT}/weights/yolo{v}n_amr_best.pt')
    info = m.info(verbose=False)
    print(f'\n##### yolo{v}n  (layers, params, gradients, GFLOPs) = {info}')
    for split in ('val', 'test'):
        r = m.val(data=yaml, split=split, imgsz=640, conf=0.001, iou=0.6, verbose=False, plots=False,
                  project=f'{ROOT}/logs/yolo_val', name=f'y{v}_{split}', exist_ok=True)
        cls = {m.names[i]: (round(float(r.box.p[k]), 3), round(float(r.box.r[k]), 3), round(float(r.box.ap50[k]), 3))
               for k, i in enumerate(r.box.ap_class_index)}
        print(f'  [{split}] mAP50={r.box.map50:.3f} mAP50-95={r.box.map:.3f}  per-class (P, R, AP50): {cls}')
    for dev in ('cuda:0', 'cpu'):
        for _ in range(5): m.predict(img, device=dev, verbose=False)
        t = time.time(); n = 30 if dev != 'cpu' else 15
        for _ in range(n): m.predict(img, device=dev, verbose=False)
        print(f'  latency {dev}: {1000 * (time.time() - t) / n:.1f} ms/frame (704x704 → imgsz 640)')
    for _ in range(3): m.track(img, persist=True, verbose=False, tracker='bytetrack.yaml', device='cpu')
