#!/usr/bin/env python3
"""
현재 시뮬 카메라 1프레임에 yolo11n / yolo12n 을 돌려 RC카 감지 여부 확인 (로봇 구동 없음).
  source ~/yolo_venv/bin/activate; python detect_test.py [conf=0.4]
결과 이미지: ~/tb4_follow_sim/logs/detect_y11.jpg, detect_y12.jpg
"""
import os, sys, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)   # best-effort 구독: reliable·best-effort 발행자 모두와 연결
from sensor_msgs.msg import Image
from ultralytics import YOLO

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 이 저장소 폴더 (어디에 풀어도 동작)
os.makedirs(f'{ROOT}/logs', exist_ok=True)
CONF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4


def grab():
    rclpy.init(); n = Node('detect_test'); box = []
    n.create_subscription(Image, '/robot1/oakd/rgb/image_raw', lambda m: box.append(m), SENSOR_QOS)
    t = time.time()
    while not box and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node(); rclpy.shutdown()
    if not box: sys.exit('no image on /robot1/oakd/rgb/image_raw')
    m = box[-1]
    return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()


img = grab()
for v in ('11', '12'):
    model = YOLO(f'{ROOT}/weights/yolo{v}n_amr_best.pt')
    r = model.predict(img, conf=0.05, verbose=False)[0]
    dets = [(model.names[int(b.cls[0])], float(b.conf[0]), [int(x) for x in b.xyxy[0]]) for b in r.boxes]
    dets.sort(key=lambda d: -d[1])
    print(f'yolo{v}n: ' + (', '.join(f'{c} {p:.2f} {xy}' for c, p, xy in dets[:5]) or 'no detection'))
    ok = [d for d in dets if d[0] == 'car' and d[1] >= CONF]
    print(f'   → car@conf>={CONF}: {"YES" if ok else "NO"}')
    cv2.imwrite(f'{ROOT}/logs/detect_y{v}.jpg', r.plot())
