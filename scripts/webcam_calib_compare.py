"""붙여 주신 답변의 방법 vs 모델 기반 방법 — 같은 조건 몬테카를로 비교.
  A  (답변 1안 그대로): 로봇 '바닥 접지점' 클릭 → 실제로는 보이지 않으므로 사람이 박스 아래 변 중앙을 클릭한다고 가정
                      → cv2.findHomography(RANSAC), 왜곡 보정 없음 / RC카는 박스 아래 변 중앙을 H 로 변환
  A+ (A + 체커보드 왜곡 보정): 왜곡 계수를 정확히 안다고 가정하고 같은 절차
  B  (모델 기반): 로봇 원기둥 모델 + AMCL (x,y) 로 카메라 파라미터(k1 포함) 추정 → RC카는 알려진 치수로 3D 박스 맞춤
  B0 (B 의 카메라 + 박스 아래 변 중앙 역투영만): 3D 박스 맞춤의 효과 분리용
"""
import math, sys
import numpy as np, cv2
sys.path.insert(0, __file__.rsplit('\\', 1)[0] if '\\' in __file__ else '.')
from webcam_geom import *

rng = np.random.default_rng(1)
# 참값 카메라: 앞서 데이터셋으로 추정한 웹캠 (높이 1.29 m, 48.1°, f 750) + 약한 술통 왜곡, 롤 0.9°
TRUE = np.array([-0.5, 0.0, 1.29, 0.0, math.radians(48.1), math.radians(0.9), 750.0, -0.12])
SIG_AMCL, SIG_PX, N_CAL, TRIALS = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05, 2.0, int(sys.argv[1]) if len(sys.argv) > 1 else 10, 20
ARENA = 4.5   # 이 범위 밖 추정은 무효로 처리 (지평선 근처 역투영 폭주 방지)


def visible_pts(n, margin=25):
    out = []
    while len(out) < n:
        x, y = rng.uniform(0.0, 3.0), rng.uniform(-1.6, 1.6)
        bb = robot_bbox(TRUE, x, y)
        if bb[0] > margin and bb[1] > margin and bb[2] < IMG_W - margin and bb[3] < IMG_H - margin:
            out.append((x, y))
    return out


def noisy(bb): return bb + rng.normal(0, SIG_PX, 4)


def undist_px(c, uv):
    xn, yn = undistort_norm(c, np.asarray(uv, float))
    return np.c_[c[6] * xn + IMG_W / 2, c[6] * yn + IMG_H / 2]


res = {k: [] for k in ('A', 'A+', 'B0', 'B')}
cam_err = []
for t in range(TRIALS):
    cal = visible_pts(N_CAL)
    obs = [(x + rng.normal(0, SIG_AMCL), y + rng.normal(0, SIG_AMCL), noisy(robot_bbox(TRUE, x, y))) for x, y in cal]
    # A / A+: 박스 아래 변 중앙 ↔ AMCL (x,y)
    px = np.array([[(b[0] + b[2]) / 2, b[3]] for _, _, b in obs]); mp = np.array([[x, y] for x, y, _ in obs])
    HA, _ = cv2.findHomography(px, mp, cv2.RANSAC, 0.1)
    HAp, _ = cv2.findHomography(undist_px(TRUE, px), mp, cv2.RANSAC, 0.1)
    # B: 대략값(높이 1.5, 숙임 40°, f 600, k1 0)에서 시작
    c0 = np.array([-0.8, 0.3, 1.5, 0.15, math.radians(40), 0.0, 600.0, 0.0])
    cB, rms = calibrate_from_robot(obs)          # 초기값은 호모그래피 분해
    cam_err.append((abs(cB[2] - TRUE[2]), abs(math.degrees(cB[4] - TRUE[4])), abs(cB[6] - TRUE[6]), rms))
    # 평가: RC카 30 대 (임의 위치·방향)
    for x, y in visible_pts(30, margin=10):
        yaw = rng.uniform(0, math.pi); bb = noisy(car_bbox(TRUE, x, y, yaw))
        bc = np.array([[(bb[0] + bb[2]) / 2, bb[3]]], np.float64)
        pa = cv2.perspectiveTransform(bc[None], HA)[0, 0]
        pap = cv2.perspectiveTransform(undist_px(TRUE, bc)[None], HAp)[0, 0]
        pb0 = backproject(cB, bc)[0]
        xb, yb, _, _ = localize_car(cB, bb)
        for k, p in (('A', pa), ('A+', pap), ('B0', pb0), ('B', (xb, yb))):
            ok = abs(p[0]) < ARENA and abs(p[1]) < ARENA
            res[k].append(math.hypot(p[0] - x, p[1] - y) if ok else float('inf'))

print(f'조건: 보정점 {N_CAL} 개, AMCL σ {SIG_AMCL} m, 박스 σ {SIG_PX} px, 시행 {TRIALS} 회 × RC카 30 대')
ce = np.array(cam_err)
print(f'B 카메라 추정 오차 중앙값: 높이 {np.median(ce[:,0])*100:.1f} cm, 숙임 {np.median(ce[:,1]):.2f}°, f {np.median(ce[:,2]):.0f} px, 박스 재투영 rms {np.median(ce[:,3]):.1f} px')
for k, v in res.items():
    v = np.array(v) * 100; bad = np.mean(~(v < 50)) * 100
    print(f'{k:3s}  중앙값 {np.median(v):5.1f} cm   90% {np.percentile(v,90):6.1f} cm   실패(>50cm·무효) {bad:4.1f}%')
