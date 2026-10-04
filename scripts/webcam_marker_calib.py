#!/usr/bin/env python3
"""마커 방식 웹캠 보정 — 바닥 마커(ID 10~13) 4 장 + 줄자. ROS·로봇·AMCL 불필요, 웹캠 PC 에서 바로 실행.
마커 없는 방식(webcam_calib.py: 로봇 박스 + AMCL)과 같은 카메라 모델·같은 yaml 을 만든다 → webcam_car_localizer.py,
webcam_track_offline.py 를 그대로 쓸 수 있음. 차이: 좌표계가 SLAM 맵이 아니라 '바닥 좌표'(줄자로 정한 것).

  python3 webcam_marker_calib.py ~/webcam_logs/<시도ID>/webcam_000.mp4 calib/floor_markers.csv
  python3 webcam_marker_calib.py 사진.jpg calib/floor_markers.csv --out calib/webcam_calib_marker.yaml

floor_markers.csv (줄자로 잰 값, m / 도):
  id,x,y,size,yaw_deg
  10,0.00,0.00,0.10,0      ← 마커 중심 좌표, size = 검은 테두리 바깥 한 변(인쇄 후 실측), yaw = 마커 윗변이 가리키는 방향
  ...                         (바닥 x 축 기준 반시계, 4 장 모두 같은 방향으로 붙였으면 같은 값)
좌표 정하는 법: 원점 = ID 10 중심, x 축 = ID 10 → ID 11 방향, y 축 = 그 왼쪽(위에서 볼 때 반시계 90°).
영상이면 앞쪽 --frames 장에서 모서리를 평균 (사람이 지나가 가린 프레임은 자동 제외).
검증: 재투영 오차(px) + 한 장씩 빼고 보정해 뺀 마커 중심을 맞히는 오차(cm) — 이 값이 실제 위치 오차의 근사.
"""
import argparse, math, os, sys
import numpy as np, cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import webcam_geom as G
from webcam_calib import KEYS, ROOT

ap = argparse.ArgumentParser()
ap.add_argument('src', help='웹캠 영상(mp4) 또는 사진')
ap.add_argument('markers', help='floor_markers.csv')
ap.add_argument('--out', default=os.path.join(ROOT, 'calib', 'webcam_calib_marker.yaml'))
ap.add_argument('--frames', type=int, default=30, help='영상에서 쓸 프레임 수 (앞에서부터)')
ap.add_argument('--check', default='', help='검증 그림 저장 경로 (기본: out 옆 .jpg)')
a = ap.parse_args()

M = {}
for line in open(a.markers, encoding='utf-8'):
    t = [s.strip() for s in line.split(',')]
    if len(t) >= 5 and t[0].isdigit():
        M[int(t[0])] = (float(t[1]), float(t[2]), float(t[3]), math.radians(float(t[4])))
if len(M) < 3: sys.exit(f'{a.markers}: 마커 좌표가 3 개 미만')

det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
cap = cv2.VideoCapture(a.src); seen = {k: [] for k in M}; first = None; W = H = None; n = 0
while n < a.frames:
    ok, img = cap.read()
    if not ok: break
    n += 1; H, W = img.shape[:2]; first = img if first is None else first
    cs, ids, _ = det.detectMarkers(img)            # 원본 해상도에서 검출 (작은 마커에 유리)
    for c, i in zip(cs or [], (ids.flatten() if ids is not None else [])):
        if int(i) in M: seen[int(i)].append(c[0])
if first is None: sys.exit(f'영상을 못 읽음: {a.src}')
found = {k: np.median(v, axis=0) for k, v in seen.items() if len(v) >= max(1, n // 3)}   # 1/3 이상 보인 마커만
print(f'{n} 프레임, 원본 {W}x{H} → 640 화면 (가운데 {min(W, H)} 정사각). 찾은 마커: '
      + ', '.join(f'ID {k} ({len(seen[k])}/{n})' for k in sorted(M)))
if len(found) < 3: sys.exit('마커가 3 장 이상 보여야 함 — 웹캠 화면에서 4 장이 모두 보이게 옮길 것 (크기 기준은 인쇄 시트 3~6 쪽)')

ids = sorted(found)
P = np.vstack([G.marker_corners_floor(*M[k]) for k in ids])
uv = np.vstack([G.full_to_model(found[k], W, H) for k in ids])
c, rms = G.calibrate_from_points(P, uv)

# 한 장씩 빼고 보정 → 뺀 마커 중심 예측 오차 (마커 4 장이면 3 장으로 보정하는 셈이라 보수적인 값)
loo = []
if len(ids) >= 4:
    for j, k in enumerate(ids):
        keep = [i for i in range(len(ids)) if i != j]
        ck, _ = G.calibrate_from_points(np.vstack([P[4 * i:4 * i + 4] for i in keep]), np.vstack([uv[4 * i:4 * i + 4] for i in keep]), c0=c)
        pred = G.backproject(ck, uv[4 * j:4 * j + 4]).mean(0)
        loo.append((k, 100 * math.dist(pred, M[k][:2])))
side = [100 * abs(np.linalg.norm(np.diff(G.backproject(c, uv[4 * i:4 * i + 4])[[0, 1]], axis=0)) - M[k][2]) for i, k in enumerate(ids)]

lines = ['# webcam_marker_calib.py — 바닥 좌표계 카메라 (마커 방식, 주점=640 화면 중심, 방사 왜곡 k1)']
lines += [f'{kk}: {v:.6f}' for kk, v in zip(KEYS, c)]
lines += [f'image_size: [{G.IMG_W}, {G.IMG_H}]', f'source_size: [{W}, {H}]', 'frame: floor', 'method: marker',
          f'markers: {ids}', f'reproj_rms_px: {rms:.2f}', f'marker_side_err_cm_max: {max(side):.1f}']
if loo: lines += [f'loo_err_cm_max: {max(e for _, e in loo):.1f}', '# loo: ' + ', '.join(f'ID {k} {e:.1f} cm' for k, e in loo)]
os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
open(a.out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
print('\n'.join(lines[1:])); print(f'저장: {a.out}')
print(f'카메라: 높이 {c[2]:.2f} m, 숙임 {math.degrees(c[4]):.1f}°, f {c[6]:.0f} px (640 화면 기준)')

# 검증 그림: 640 화면 + 바닥 0.5 m 격자 + 마커 재투영
v = G.to_model_view(first)
xs = [m[0] for m in M.values()]; ys = [m[1] for m in M.values()]
for gx in np.arange(math.floor(min(xs)) - 1, max(xs) + 1.01, 0.5):
    pts = G.project(c, np.c_[np.full(41, gx), np.linspace(min(ys) - 1, max(ys) + 1, 41), np.zeros(41)])
    cv2.polylines(v, [pts.astype(np.int32)], False, (255, 200, 0), 1)
for gy in np.arange(math.floor(min(ys)) - 1, max(ys) + 1.01, 0.5):
    pts = G.project(c, np.c_[np.linspace(min(xs) - 1, max(xs) + 1, 41), np.full(41, gy), np.zeros(41)])
    cv2.polylines(v, [pts.astype(np.int32)], False, (255, 200, 0), 1)
for i, k in enumerate(ids):
    cv2.polylines(v, [G.project(c, np.c_[P[4 * i:4 * i + 4], np.zeros(4)]).astype(np.int32)], True, (0, 0, 255), 2)
    cv2.putText(v, f'ID {k}', tuple(int(t) for t in uv[4 * i] + [0, -6]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
chk = a.check or os.path.splitext(a.out)[0] + '_check.jpg'; cv2.imwrite(chk, v)
print(f'검증 그림: {chk}  (빨간 사각형이 마커와 겹치고, 0.5 m 격자가 바닥 테이프와 맞으면 OK)')
if rms > 3 or (loo and max(e for _, e in loo) > 10):
    print('!! 오차가 큼 → floor_markers.csv 의 좌표·size·yaw(윗변 방향)를 다시 확인')
