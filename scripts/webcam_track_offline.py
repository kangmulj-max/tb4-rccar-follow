#!/usr/bin/env python3
"""웹캠 녹화(webcam_rec.py) → 프레임별 RC카·로봇 위치, 간격, '로봇 카메라 시야 안' 판정. 시험 뒤 오프라인 (ROS 불필요).
  python3 webcam_track_offline.py ~/webcam_logs/<시도ID> --calib calib/webcam_calib_marker.yaml
입력: <시도>/webcam_*.mp4 + frames.csv (시각), 보정 yaml (마커 방식 또는 마커 없는 방식 — 같은 형식)
  RC카: 웹캠 YOLO(car) 박스 + 알려진 치수로 3D 박스 맞춤 (webcam_geom.localize_car)
  로봇: 윗판 마커(ID 0)가 있으면 그 중심·윗변 방향 → 위치·정면 방향.
        웹캠 모델에는 로봇 클래스가 없으므로 마커가 없으면 로봇 칸은 비고, 마커 없는 방식에서는
        AMR PC bag 의 /robot1/amcl_pose (맵 좌표) 를 시각으로 맞춰 붙여야 함 (보정도 맵 좌표로 했을 때만).
출력: <시도>/webcam_track.csv, webcam_track_summary.json
"""
import argparse, csv, glob, json, math, os, sys
import numpy as np, cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import webcam_geom as G
from webcam_calib import load_calib, ROOT

ap = argparse.ArgumentParser()
ap.add_argument('trial_dir')
ap.add_argument('--calib', default=os.path.join(ROOT, 'calib', 'webcam_calib_marker.yaml'))
ap.add_argument('--model', default=os.path.join(ROOT, 'weights', 'webcam_yolo26s_best.pt'))
ap.add_argument('--conf', type=float, default=0.4)
ap.add_argument('--stride', type=int, default=3, help='N 프레임마다 1 장 처리 (15 fps / 3 = 5 Hz)')
ap.add_argument('--car-dims', type=float, nargs=3, default=list(G.CAR), metavar=('L', 'W', 'H'), help='RC카 실측 치수 (m)')
ap.add_argument('--robot-id', type=int, default=0); ap.add_argument('--robot-marker-z', type=float, default=G.ROBOT['h'], help='윗판 마커 높이 (m)')
ap.add_argument('--robot-r', type=float, default=G.ROBOT['r'])
ap.add_argument('--fov-deg', type=float, default=42.0, help='로봇 카메라 수평 화각 (robot1 실측 704x704 → 약 42°)')
ap.add_argument('--max-fit-px', type=float, default=15.0)
a = ap.parse_args()

cam = load_calib(a.calib)
frames = {int(r['frame']): r for r in csv.DictReader(open(os.path.join(a.trial_dir, 'frames.csv'), encoding='utf-8'))}
vids = sorted(glob.glob(os.path.join(a.trial_dir, 'webcam_*.mp4')))
if not vids: sys.exit('webcam_*.mp4 없음')
from ultralytics import YOLO
yolo = YOLO(a.model)
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
half = math.radians(a.fov_deg) / 2

rows = []; idx = 0
for vp in vids:                                                     # 세그먼트를 이어서 = frames.csv 의 frame 번호 순서
    cap = cv2.VideoCapture(vp)
    while True:
        ok, img = cap.read()
        if not ok: break
        i = idx; idx += 1
        if i % a.stride: continue
        Hh, Ww = img.shape[:2]; f = frames.get(i, {})
        r = dict(frame=i, t_wall=f.get('t_wall', ''), car_x='', car_y='', car_yaw='', car_conf='', car_fit_px='',
                 rob_x='', rob_y='', rob_heading='', gap_center_m='', gap_front_m='', car_bearing_deg='', car_in_robot_fov='')
        # 로봇: 원본 해상도에서 마커 검출 → 640 화면 좌표 → 마커 높이 평면으로 역투영
        cs, ids, _ = det.detectMarkers(img)
        if ids is not None and a.robot_id in ids.flatten():
            cr = cs[list(ids.flatten()).index(a.robot_id)][0]
            P = G.backproject(cam, G.full_to_model(cr, Ww, Hh), z=a.robot_marker_z)     # TL, TR, BR, BL
            ctr = P.mean(0); up = (P[0] + P[1]) / 2 - (P[2] + P[3]) / 2                  # 마커 윗변 = 로봇 정면
            r.update(rob_x=round(ctr[0], 3), rob_y=round(ctr[1], 3), rob_heading=round(math.atan2(up[1], up[0]), 3))
        # RC카: 640 화면에서 YOLO → 3D 박스 맞춤
        res = yolo.predict(G.to_model_view(img), conf=a.conf, verbose=False)[0]
        cars = [b for b in res.boxes if yolo.names[int(b.cls)] == 'car']
        if cars:
            b = max(cars, key=lambda b: float(b.conf))
            x, y, yaw, fit = G.localize_car(cam, b.xyxy[0].tolist(), tuple(a.car_dims))
            if fit <= a.max_fit_px:
                r.update(car_x=round(x, 3), car_y=round(y, 3), car_yaw=round(yaw, 3), car_conf=round(float(b.conf), 3), car_fit_px=round(fit, 1))
        if r['car_x'] != '' and r['rob_x'] != '':
            dx, dy = r['car_x'] - r['rob_x'], r['car_y'] - r['rob_y']; d = math.hypot(dx, dy)
            brg = (math.atan2(dy, dx) - r['rob_heading'] + math.pi) % (2 * math.pi) - math.pi
            r.update(gap_center_m=round(d, 3), gap_front_m=round(d - a.robot_r - a.car_dims[0] / 2, 3),   # 범퍼 앞 ↔ 차 뒤 (근사)
                     car_bearing_deg=round(math.degrees(brg), 1), car_in_robot_fov=int(abs(brg) <= half))
        rows.append(r)
    cap.release()

out = os.path.join(a.trial_dir, 'webcam_track.csv')
with open(out, 'w', newline='', encoding='utf-8') as fo:
    w = csv.DictWriter(fo, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
both = [r for r in rows if r['gap_center_m'] != '']
gf = np.array([r['gap_front_m'] for r in both]) if both else np.array([])
S = dict(frames_processed=len(rows), stride=a.stride, calib=os.path.basename(a.calib),
         car_seen_pct=round(100 * np.mean([r['car_x'] != '' for r in rows]), 1),
         robot_seen_pct=round(100 * np.mean([r['rob_x'] != '' for r in rows]), 1),
         both_seen=len(both),
         gap_front_m=dict(median=round(float(np.median(gf)), 3), min=round(float(gf.min()), 3), max=round(float(gf.max()), 3)) if both else None,
         car_in_robot_fov_pct=round(100 * np.mean([r['car_in_robot_fov'] for r in both]), 1) if both else None,
         note='웹캠 시야 안 구간만. 간격은 로봇 중심·RC카 중심에서 반지름·반길이를 뺀 근사. 시각은 웹캠 PC 시계 (t_wall).')
json.dump(S, open(os.path.join(a.trial_dir, 'webcam_track_summary.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print(json.dumps(S, ensure_ascii=False, indent=1)); print(f'저장: {out}')
