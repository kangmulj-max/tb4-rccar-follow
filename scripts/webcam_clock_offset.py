#!/usr/bin/env python3
"""웹캠 PC 시계 ↔ AMR PC 시계 차이를 bag 만으로 추정 (수동 동기화, Olson 2010 IROS 'A Passive Solution to the Sensor Synchronization Problem').
  python3 webcam_clock_offset.py ~/tb4_real_logs/<시도ID>/bag [--topic /webcam/image_raw/compressed] [--drift-ppm 50]
필요: record_real.sh 를 WEBCAM=1 로 실행 (웹캠 PC 는 webcam_rec.py --publish).
  p_i = 웹캠 PC 가 프레임을 찍은 시각 (header.stamp, 웹캠 PC 시계)
  q_i = AMR PC 가 받은 시각 (bag 기록 시각, AMR PC 시계)
  p_i - q_i = A - e_i  (A = 두 시계 차이, e_i ≥ 0 = 전송 지연) → A ≥ max(p_i - q_i)
  지연이 가장 짧았던 메시지가 A 를 가장 잘 알려 줌. 시계가 천천히 어긋나는 것(drift)은 |ΔA| ≤ α·Δt 로 허용 (Olson Alg. 1, 앞뒤 두 번 훑기).
출력: A (s) — 웹캠 frames.csv 의 t_wall 에서 A 를 빼면 AMR PC 시계 (bag 과 같은 시계).
  같이 출력하는 '단순 평균 지연' 은 동기화를 안 했을 때의 오차 크기(참고).
"""
import argparse, sys
import numpy as np


def passive_offset(p, q, alpha):
    """Olson 2010 Algorithm 1 (양방향). p, q: 같은 길이 배열 (s). 반환: 메시지별 A_i."""
    n = len(p); A = np.empty(n); f = lambda d: alpha * abs(d)
    bp = bq = None
    for i in range(n):                                            # 앞으로
        if bp is None or p[i] - q[i] >= bp - bq - f(p[i] - bp): bp, bq = p[i], q[i]; A[i] = p[i] - q[i]
        else: A[i] = bp - bq - f(p[i] - bp)
    bp = bq = None
    for i in range(n - 1, -1, -1):                                # 뒤로
        if bp is None or p[i] - q[i] >= bp - bq - f(p[i] - bp): bp, bq = p[i], q[i]; A[i] = max(A[i], p[i] - q[i])
        else: A[i] = max(A[i], bp - bq - f(p[i] - bp))
    return A


def read_bag(path, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    P, Q = [], []
    while r.has_next():
        _, data, t_ns = r.read_next()
        m = deserialize_message(data, CompressedImage)
        P.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); Q.append(t_ns * 1e-9)
    return np.array(P), np.array(Q)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('--topic', default='/webcam/image_raw/compressed')
    ap.add_argument('--drift-ppm', type=float, default=50.0, help='두 시계가 어긋나는 최대 속도 (일반 PC 수정 발진기 수십 ppm)')
    a = ap.parse_args()
    p, q = read_bag(a.bag, a.topic)
    if len(p) < 5: sys.exit(f'{a.topic} 메시지 {len(p)} 개 — WEBCAM=1 로 기록했는지 확인')
    A = passive_offset(p, q, a.drift_ppm * 1e-6)
    lat = q - (p - A)                                              # 추정 전송 지연
    print(f'메시지 {len(p)} 개, {q[-1] - q[0]:.0f} s')
    print(f'시계 차이 A (웹캠 PC − AMR PC): 중앙값 {np.median(A) * 1000:+.1f} ms  (시도 중 변화 {np.ptp(A) * 1000:.1f} ms)')
    print(f'추정 전송 지연: 중앙값 {np.median(lat) * 1000:.0f} ms, 최대 {lat.max() * 1000:.0f} ms')
    print(f'참고 — 동기화 안 하고 받은 시각을 쓰면 오차: 평균 {np.mean(q - p) * 1000:+.0f} ms')
    print('사용: AMR 시계 = 웹캠 frames.csv 의 t_wall − A')
