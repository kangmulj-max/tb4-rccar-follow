#!/usr/bin/env python3
"""웹캠 YOLO RC카 박스 → 맵 좌표 발행 (LOP/LPP 입력용).
  python3 webcam_car_localizer.py --ros-args -p image_topic:=/sim/webcam
  실기 두 PC (AMR PC 에서): -p image_topic:=/webcam/image_raw/compressed [-p calib:=calib/webcam_calib_marker.yaml -p frame_id:=floor]
  웹캠 PC 는 webcam_rec.py --publish (2 Hz 미리보기) → 실시간 위치도 2 Hz. 정밀 분석은 webcam_track_offline.py
출력: /webcam/car_pose (geometry_msgs/PoseStamped, frame map). yaw 는 앞뒤 구분 불가(0~π).
위치는 박스 아래 변 중앙이 아니라 알려진 RC카 치수로 3D 박스를 맞춰 구한 바닥 중심
(Mousavian et al. 2017). 아래 변 중앙만 쓰면 비스듬한 웹캠에서 약 14 cm 앞쪽으로 치우침 (시뮬 비교).
"""
import math, os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import webcam_geom as G
from webcam_calib import load_calib, YAML, ROOT


class Localizer(Node):
    def __init__(self):
        super().__init__('webcam_car_localizer')
        P = self.declare_parameter
        P('image_topic', '/sim/webcam'); P('model', os.path.join(ROOT, 'weights', 'webcam_yolo26s_best.pt'))
        P('calib', YAML); P('conf', 0.4); P('car_dims', list(G.CAR)); P('arena', [-4.0, 4.0, -3.0, 3.0])
        g = lambda n: self.get_parameter(n).value
        from ultralytics import YOLO
        self.yolo = YOLO(g('model')); self.cam = load_calib(g('calib'))
        self.conf, self.dims, self.arena = g('conf'), tuple(g('car_dims')), g('arena')
        self.pub = self.create_publisher(PoseStamped, '/webcam/car_pose', 10)
        P('frame_id', 'map')   # 마커 방식 보정이면 floor
        self.frame_id = g('frame_id'); t = g('image_topic')
        self.create_subscription(CompressedImage if t.endswith('/compressed') else Image, t, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        img = G.to_model_view(G.img_from_msg(m))                          # 640 정사각 (학습 데이터와 같은 화면)
        r = self.yolo.predict(img, conf=self.conf, verbose=False)[0]
        cars = [b for b in r.boxes if self.yolo.names[int(b.cls)] == 'car']
        if not cars: return
        bb = max(cars, key=lambda b: float(b.conf)).xyxy[0].tolist()
        x, y, yaw, fit_px = G.localize_car(self.cam, bb, self.dims)
        x0, x1, y0, y1 = self.arena
        if not (x0 <= x <= x1 and y0 <= y <= y1) or fit_px > 15:      # 지평선 근처 폭주·잘못 맞춘 박스 버림
            self.get_logger().warn(f'버림: ({x:.2f},{y:.2f}) fit {fit_px:.1f}px', throttle_duration_sec=2); return
        p = PoseStamped(); p.header.stamp = m.header.stamp; p.header.frame_id = self.frame_id
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub.publish(p)


def main():
    rclpy.init(); n = Localizer()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == '__main__':
    main()
