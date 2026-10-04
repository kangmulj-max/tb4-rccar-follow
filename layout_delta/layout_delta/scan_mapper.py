"""scan_mapper node: builds this round's occupancy grid on the fixed reference map frame.

Subscribes : scan (sensor_msgs/LaserScan)          — relative name, use a namespace
TF         : map -> <scan frame> at scan time (from AMCL / localization on the reference map)
Services   : ~/save (std_srvs/Trigger)              — write round map (pgm+yaml), coverage stats
Params     : ref_map (yaml), out_dir, map_frame='map', max_range=6.0, min_scans_per_save=20,
             tf_timeout=0.1, max_tf_age=0.3 (fallback to latest TF), stride=1 (use every n-th scan),
             only_when_still=True, still_settle_s=0.5 (stop-and-observe; subscribes odom),
             max_pose_std_m=0.25 (skip scans while AMCL covariance is large; subscribes amcl_pose)
On shutdown the grid is saved once more (no data loss when the mission is interrupted).
"""
import json
import math
import os
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .grid import MapGrid, ScanAccumulator


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ScanMapper(Node):
    def __init__(self):
        super().__init__('scan_mapper')
        P = self.declare_parameter
        self.ref = MapGrid.load(P('ref_map', '').value)
        self.out_dir = P('out_dir', 'round').value
        self.map_frame = P('map_frame', 'map').value
        self.min_scans = P('min_scans_per_save', 20).value
        self.tf_timeout = P('tf_timeout', 0.1).value
        self.stride = max(1, P('stride', 1).value)
        self.max_tf_age = P('max_tf_age', 0.3).value     # fallback: latest TF allowed if within this age of the scan
        # stop-and-observe: integrate only scans taken while the robot stands still (pose error and scan/pose
        # time skew are smallest then). Quantitative round comparison uses these scans only.
        self.only_still = P('only_when_still', True).value
        self.still_settle = P('still_settle_s', 0.5).value
        self.still_since = None; self.odom_t = 0.0; self.n_moving = 0
        self.create_subscription(Odometry, 'odom', self.cb_odom, qos_profile_sensor_data)
        # localization quality gate: a lost robot must not write into the round map (0 = off)
        self.max_pose_std = P('max_pose_std_m', 0.25).value
        self.pose_std = 0.0; self.n_badpose = 0
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.cb_pose, 10)
        self.acc = ScanAccumulator(self.ref, max_range=P('max_range', 6.0).value)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.n_in = 0; self.n_tf_fail = 0; self.n_tf_latest = 0; self.t0 = time.time()
        self.create_subscription(LaserScan, 'scan', self.cb, qos_profile_sensor_data)
        self.create_service(Trigger, '~/save', self.srv_save)
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f'scan_mapper: ref {self.ref.shape} @ {self.ref.resolution} m → {self.out_dir}')

    def cb_odom(self, m: Odometry):
        v = m.twist.twist
        still = abs(v.linear.x) < 0.01 and abs(v.angular.z) < 0.02
        now = time.time(); self.odom_t = now
        self.still_since = (self.still_since or now) if still else None

    def cb_pose(self, m: PoseWithCovarianceStamped):
        c = m.pose.covariance
        self.pose_std = math.sqrt(max(0.0, c[0] + c[7]))

    def is_still(self):
        now = time.time()
        return self.still_since is not None and now - self.odom_t < 0.5 and now - self.still_since >= self.still_settle

    def cb(self, m: LaserScan):
        self.n_in += 1
        if self.n_in % self.stride:
            return
        if self.only_still and not self.is_still():
            self.n_moving += 1
            return
        if self.max_pose_std > 0 and self.pose_std > self.max_pose_std:
            self.n_badpose += 1
            return
        try:
            t = self.tf.lookup_transform(self.map_frame, m.header.frame_id, m.header.stamp, Duration(seconds=self.tf_timeout))
        except Exception:
            t = None
        if t is None:   # scan newer than the last odom/AMCL TF → use the latest TF if it is close enough in time
            try:
                t = self.tf.lookup_transform(self.map_frame, m.header.frame_id, rclpy.time.Time())
                age = (rclpy.time.Time.from_msg(m.header.stamp) - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
                if abs(age) > self.max_tf_age:
                    t = None
                else:
                    self.n_tf_latest += 1
            except Exception:
                t = None
        if t is None:
            self.n_tf_fail += 1
            if self.n_tf_fail in (1, 50, 500):
                self.get_logger().warn(f'TF {self.map_frame}->{m.header.frame_id} unavailable ({self.n_tf_fail} scans skipped) — is localization running?')
            return
        tr = t.transform.translation
        self.acc.add_scan(tr.x, tr.y, _yaw(t.transform.rotation), m.ranges, m.angle_min, m.angle_increment, m.range_min)

    def save(self, tag='map'):
        g = self.acc.grid()
        path = os.path.join(self.out_dir, f'{tag}.yaml')
        g.save(path)
        observed = int((self.acc.hits > 0).sum()); known_ref = int((self.ref.data != 1).sum())
        stats = dict(scans_used=self.acc.n_scans, scans_in=self.n_in, tf_skipped=self.n_tf_fail, tf_latest_fallback=self.n_tf_latest, skipped_moving=self.n_moving, skipped_bad_pose=self.n_badpose,
                     observed_cells=observed, coverage_of_ref_known=round(observed / max(1, known_ref), 3),
                     elapsed_s=round(time.time() - self.t0, 1))
        with open(os.path.join(self.out_dir, f'{tag}_stats.json'), 'w', encoding='utf-8') as f:
            json.dump(stats, f, indent=1)
        return path, stats

    def srv_save(self, req, res):
        if self.acc.n_scans < self.min_scans:
            res.success = False; res.message = f'only {self.acc.n_scans} scans integrated'
            return res
        path, stats = self.save()
        res.success = True; res.message = f'{path} {json.dumps(stats)}'
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    n = ScanMapper()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if n.acc.n_scans:
            n.save('map_on_exit')
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
