#!/usr/bin/env python3
"""
기하 레이캐스트 RPLIDAR (gz gpu_lidar 대체).
WSL(d3d12)에서 gz gpu_lidar 가 ogre1 로는 전 방향 range_min 만, ogre2 로는 카메라가 ~2 Hz 로 떨어져
월드 SDF 충돌 박스 + ground-truth 포즈로 2D 레이캐스트해 실기와 같은 형식의 /robot1/scan 을 만든다.

실기 사양(스냅샷/URDF): 640 샘플, -π..π, 0.164~12 m, frame rplidar_link, base 대비 +90° 회전 장착.
라이다 스캔 평면 높이(z_l)를 지나는 물체만 보인다 → 낮은 물체(RC카 0.085 m 실측, dummy 0.1 m)는 실기처럼 안 보임.
"""
import math, os, xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# 10/4 실물 bag(slam_raw_run1): 720 샘플(0.5°), 0.15~12 m, 7.63 Hz. (10/2 스냅샷 기준 640 샘플·0.164 m 에서 변경)
N, AMIN, RMIN, RMAX = 720, -math.pi, 0.15, 12.0
INC = 2 * math.pi / N


def pose6(txt):
    v = [float(x) for x in (txt or '0 0 0 0 0 0').split()]
    return v + [0.0] * (6 - len(v))


def yaw_q(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_world(path):
    """모델별 충돌 박스 [(cx, cy, yaw, sx, sy, zmin, zmax) in model frame], 모델 pose, static 여부"""
    models = {}
    for m in ET.parse(path).getroot().iter('model'):
        mp = pose6(m.findtext('pose')); boxes = []
        for c in m.iter('collision'):
            b = c.find('geometry/box/size')
            if b is None: continue
            sx, sy, sz = (float(x) for x in b.text.split()); cp = pose6(c.findtext('pose'))
            boxes.append((cp[0], cp[1], cp[5], sx, sy, cp[2] - sz / 2, cp[2] + sz / 2))
        if boxes: models[m.get('name')] = {'pose': (mp[0], mp[1], mp[5]), 'boxes': boxes,
                                           'static': (m.findtext('static') or 'false').strip() == 'true'}
    return models


class SimLidar(Node):
    def __init__(self):
        super().__init__('sim_rplidar')
        world = self.declare_parameter('world', os.path.expanduser('~/tb4_follow_sim/worlds/follow_arena.sdf')).value
        self.rate = self.declare_parameter('rate', 7.6).value           # 10/4 실물 bag 7.63 Hz
        self.noise = self.declare_parameter('noise', 0.0).value          # 일정 잡음 [m] (이전 기본 0.01)
        self.noise_k = self.declare_parameter('noise_k', 0.0025).value    # 거리 비례 잡음 σ = k·r. 10/4 실물 bag 정지 구간: 중앙값 0.7 / 2.2 / 7.2 mm (≤1 / 1-2 / 2-3.5 m)
        self.models = load_world(world)
        self.lidar = None                                               # (x, y, yaw, z) world
        self.pub = self.create_publisher(LaserScan, '/robot1/scan', 5)
        self.create_subscription(TFMessage, '/robot1/_internal/sim_ground_truth_pose', self.cb_robot, 5)
        for name, m in self.models.items():
            if not m['static']:
                self.create_subscription(TFMessage, f'/{name}/pose', lambda msg, n=name: self.cb_model(n, msg), 5)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'sim rplidar: {len(self.models)} models ' +
                               ', '.join(f'{k}{"" if v["static"] else "*"}' for k, v in self.models.items()))

    def cb_model(self, name, msg):
        for t in msg.transforms:
            if t.child_frame_id in (name, '') or len(msg.transforms) == 1:
                tr = t.transform.translation; self.models[name]['pose'] = (tr.x, tr.y, yaw_q(t.transform.rotation))

    def cb_robot(self, msg):
        model = link = None
        for t in msg.transforms:
            if t.child_frame_id == 'robot1/turtlebot4': model = t.transform
            elif t.child_frame_id == 'robot1/turtlebot4/rplidar_link': link = t.transform
        if model is None or link is None: return
        my = yaw_q(model.rotation); c, s = math.cos(my), math.sin(my)
        lx, ly = link.translation.x, link.translation.y
        self.lidar = (model.translation.x + c * lx - s * ly, model.translation.y + s * lx + c * ly,
                      my + yaw_q(link.rotation), model.translation.z + link.translation.z)

    def tick(self):
        if self.lidar is None: return
        ox, oy, oyaw, oz = self.lidar
        ang = AMIN + INC * np.arange(N) + oyaw; dx, dy = np.cos(ang), np.sin(ang)
        r = np.full(N, np.inf)
        for name, m in self.models.items():
            if name == 'floor': continue
            mx, my, myaw = m['pose']; cm, sm = math.cos(myaw), math.sin(myaw)
            for bx, by, byaw, sx, sy, z0, z1 in m['boxes']:
                if not (z0 <= oz <= z1): continue                         # 스캔 평면을 지나지 않으면 안 보임
                cx, cy = mx + cm * bx - sm * by, my + sm * bx + cm * by; yaw = myaw + byaw
                cb, sb = math.cos(-yaw), math.sin(-yaw)
                px, py = cb * (ox - cx) - sb * (oy - cy), sb * (ox - cx) + cb * (oy - cy)
                ddx, ddy = cb * dx - sb * dy, sb * dx + cb * dy
                with np.errstate(divide='ignore', invalid='ignore'):
                    t1x, t2x = (-sx / 2 - px) / ddx, (sx / 2 - px) / ddx
                    t1y, t2y = (-sy / 2 - py) / ddy, (sy / 2 - py) / ddy
                tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
                tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
                hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
                r = np.where(hit, np.minimum(r, tmin), r)
        sd = self.noise + self.noise_k * np.where(np.isfinite(r), r, 0.0)
        r = r + np.random.normal(0.0, 1.0, N) * sd
        r = np.where((r < RMIN) | (r > RMAX) | ~np.isfinite(r), np.inf, r)
        msg = LaserScan(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'rplidar_link'
        msg.angle_min, msg.angle_max, msg.angle_increment = AMIN, AMIN + INC * (N - 1), INC
        msg.scan_time = 1.0 / self.rate; msg.range_min, msg.range_max = RMIN, RMAX
        msg.ranges = r.astype(np.float32).tolist(); self.pub.publish(msg)


def main():
    rclpy.init(); n = SimLidar()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
