"""고정 웹캠 ↔ SLAM 맵(바닥 평면) 기하. ROS 의존 없음 (오프라인 시험·ROS 노드 공용).

카메라 모델: 핀홀 + 1차 방사 왜곡 k1 (Zhang 2000), 파라미터 c = [x, y, z, yaw, pitch, roll, f, k1]
  - (x, y, z): 맵 좌표의 카메라 위치, yaw: 맵 x 축 기준 수평 방향, pitch: 아래로 숙인 각(+), roll
  - 주점은 영상 중심 (W/2, H/2) 으로 고정 (640 정사각 center-crop 데이터셋과 동일)
물체 모델 (2D 박스 = 3D 형상 투영의 외접 사각형, Mousavian et al. 2017 의 제약):
  - 로봇: 원기둥 (반지름 r, 높이 h)      - RC카: 직육면체 (L, W, H) + yaw
"""
import math
import numpy as np
from scipy.optimize import least_squares

IMG_W = IMG_H = 640
ROBOT = dict(r=0.170, h=0.355)           # TurtleBot4 10/4 실측 (가로 약 34 cm, 높이 35.5 cm, measurements/field_env.sh) 를 원기둥으로 근사
CAR = (0.25, 0.08, 0.085)                # RC카 (L, W, H) 10/4 실측 (밑면 25 × 8 cm, 높이 8.5 cm, measurements/field_env.sh)


def rot(yaw, pitch, roll):
    """맵 → 카메라 회전. 카메라 축: x 오른쪽, y 아래, z 광축."""
    base = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], float)      # 수평으로 +x 를 볼 때
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])             # 맵 → 카메라 수평 방향
    cp, sp = math.cos(pitch), math.sin(pitch)
    Rp = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    cr, sr = math.cos(roll), math.sin(roll)
    Rr = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Rr @ Rp @ base @ Rz


def project(c, P):
    """맵 좌표 점들 (N,3) → 픽셀 (N,2), 왜곡 포함."""
    x, y, z, yaw, pitch, roll, f, k1 = c
    Pc = (rot(yaw, pitch, roll) @ (np.asarray(P, float) - [x, y, z]).T).T
    xn, yn = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
    d = 1 + k1 * (xn ** 2 + yn ** 2)
    return np.c_[f * xn * d + IMG_W / 2, f * yn * d + IMG_H / 2]


def undistort_norm(c, uv):
    """픽셀 → 왜곡 제거한 정규화 좌표 (고정점 반복)."""
    f, k1 = c[6], c[7]
    xd, yd = (uv[..., 0] - IMG_W / 2) / f, (uv[..., 1] - IMG_H / 2) / f
    xn, yn = xd.copy(), yd.copy()
    for _ in range(8):
        d = 1 + k1 * (xn ** 2 + yn ** 2); xn, yn = xd / d, yd / d
    return xn, yn


def backproject(c, uv, z=0.0):
    """픽셀 → 높이 z 평면 위 맵 좌표 (N,2)."""
    uv = np.atleast_2d(np.asarray(uv, float))
    xn, yn = undistort_norm(c, uv)
    R = rot(*c[3:6]); d = (R.T @ np.vstack([xn, yn, np.ones_like(xn)])).T
    t = (z - c[2]) / d[:, 2]
    return np.c_[c[0] + t * d[:, 0], c[1] + t * d[:, 1]]


def floor_homography(c):
    """왜곡이 없을 때의 바닥(z=0) → 픽셀 H (Zhang 2000: H ∝ K [r1 r2 t])."""
    K = np.array([[c[6], 0, IMG_W / 2], [0, c[6], IMG_H / 2], [0, 0, 1]])
    R = rot(*c[3:6]); t = -R @ np.array(c[:3])
    H = K @ np.c_[R[:, 0], R[:, 1], t]
    return H / H[2, 2]


def _bbox(uv):
    return np.r_[uv.min(0), uv.max(0)]


def cylinder_pts(x, y, r=ROBOT['r'], h=ROBOT['h'], n=24):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.c_[x + r * np.cos(a), y + r * np.sin(a)]
    return np.r_[np.c_[ring, np.zeros(n)], np.c_[ring, np.full(n, h)]]


def box_pts(x, y, yaw, dims=CAR):
    L, W, H = dims; c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[x + a * c - b * s, y + a * s + b * c, zz]
                     for a in (-L / 2, L / 2) for b in (-W / 2, W / 2) for zz in (0, H)])


def robot_bbox(c, x, y): return _bbox(project(c, cylinder_pts(x, y)))
def car_bbox(c, x, y, yaw, dims=CAR): return _bbox(project(c, box_pts(x, y, yaw, dims)))


# ---------------------------------------------------------------- 초기값: 바닥 호모그래피 분해 (Zhang 2000)
def init_from_homography(H):
    """맵(z=0) → 픽셀 H 에서 f, 자세, 위치 근사. 정사각 화소·주점=중심 가정.
    r1·r2 = 0, |r1| = |r2| 두 제약으로 f² 를 구하고 [r1 r2 t] 를 복원."""
    H = H / np.linalg.norm(H[:, 0])
    p = H[0] - IMG_W / 2 * H[2]; q = H[1] - IMG_H / 2 * H[2]; c = H[2]
    cand = []
    if abs(c[0] * c[1]) > 1e-12: cand.append(-(p[0] * p[1] + q[0] * q[1]) / (c[0] * c[1]))
    if abs(c[1] ** 2 - c[0] ** 2) > 1e-12:
        cand.append((p[0] ** 2 + q[0] ** 2 - p[1] ** 2 - q[1] ** 2) / (c[1] ** 2 - c[0] ** 2))
    f2 = [v for v in cand if v > 0]
    f = math.sqrt(np.mean(f2)) if f2 else 700.0
    K = np.array([[f, 0, IMG_W / 2], [0, f, IMG_H / 2], [0, 0, 1]])
    A = np.linalg.inv(K) @ H
    lam = 1 / np.linalg.norm(A[:, 0])
    if (lam * A[:, 2])[2] < 0: lam = -lam                      # 맵 원점이 카메라 앞에 오도록
    r1, r2, t = lam * A[:, 0], lam * A[:, 1], lam * A[:, 2]
    R = np.c_[r1, r2, np.cross(r1, r2)]
    U, _, Vt = np.linalg.svd(R); R = U @ Vt                       # 가장 가까운 회전행렬
    C = -R.T @ t
    d = R.T @ [0, 0, 1]; yaw = math.atan2(d[1], d[0]); pitch = math.asin(np.clip(-d[2], -1, 1))
    R0 = rot(yaw, pitch, 0.0); Rr = R @ R0.T; roll = math.atan2(Rr[1, 0], Rr[0, 0])
    return np.array([C[0], C[1], C[2], yaw, pitch, roll, f, 0.0])


# ---------------------------------------------------------------- 보정: 로봇(AMCL 자세)을 움직이는 기준물로
def calibrate_from_robot(samples, c0=None, fit_k1=True, k1_sigma=None):
    """samples: [(x_map, y_map, bbox[u1,v1,u2,v2])] → 카메라 파라미터.
    박스 4 변 재투영 오차를 최소화 (기하 오차, Hartley & Zisserman 2004). AMCL 오차·검출 이상치에
    강하도록 soft-L1 손실. c0 가 없으면 박스 아래 변 중앙 ↔ (x,y) 호모그래피를 분해해 초기값 (Zhang 2000)."""
    if c0 is None:
        import cv2
        px = np.array([[(b[0] + b[2]) / 2, b[3]] for _, _, b in samples], np.float64)
        mp = np.array([[x, y] for x, y, _ in samples], np.float64)
        Hm2p, _ = cv2.findHomography(mp, px, 0)
        c0 = init_from_homography(Hm2p)
    def res(p):
        c = p if fit_k1 else np.r_[p, 0.0]
        r = np.concatenate([robot_bbox(c, x, y) - bb for x, y, bb in samples])
        # k1 은 f 와 상관이 커서 보정점만으로는 잘 안 정해짐 → 약한 사전값(0 중심)으로 안정화 (선택)
        return np.r_[r, 10.0 * c[7] / k1_sigma] if (fit_k1 and k1_sigma) else r
    p0 = np.asarray(c0, float) if fit_k1 else np.asarray(c0[:7], float)
    lo = [-50, -50, 0.2, -2 * math.pi, 0.0, -0.5, 100] + ([-0.6] if fit_k1 else [])
    hi = [50, 50, 10, 2 * math.pi, 1.55, 0.5, 5000] + ([0.6] if fit_k1 else [])
    r = least_squares(res, np.clip(p0, lo, hi), bounds=(lo, hi), loss='soft_l1', f_scale=3.0, x_scale='jac')
    c = r.x if fit_k1 else np.r_[r.x, 0.0]
    return c, float(np.sqrt(np.mean(r.fun ** 2)))


# ---------------------------------------------------------------- RC카 위치: 박스 + 알려진 치수 → 바닥 중심
def localize_robot(c, bbox):
    """로봇 박스 → 바닥 중심 (x, y). 보정 검증(교차검증)용."""
    bb = np.asarray(bbox, float)
    x0, y0 = backproject(c, [[(bb[0] + bb[2]) / 2, bb[3]]])[0]
    r = least_squares(lambda p: robot_bbox(c, p[0], p[1]) - bb, [x0, y0])
    return r.x


def localize_car(c, bbox, dims=CAR):
    """2D 박스를 가장 잘 설명하는 (x, y, yaw) — 박스 4 변 = 3D 직육면체 투영의 외접 (Mousavian 2017).
    초기값: 박스 아래 변 중앙을 바닥으로 역투영. yaw 는 앞뒤 대칭이라 0~π 다중 시작."""
    bb = np.asarray(bbox, float)
    x0, y0 = backproject(c, [[(bb[0] + bb[2]) / 2, bb[3]]])[0]
    best = None
    for yaw0 in np.linspace(0, math.pi, 4, endpoint=False):
        r = least_squares(lambda p: car_bbox(c, p[0], p[1], p[2], dims) - bb, [x0, y0, yaw0])
        if best is None or r.cost < best.cost: best = r
    x, y, yaw = best.x
    return x, y, yaw % math.pi, float(np.sqrt(np.mean(best.fun ** 2)))


# ---------------------------------------------------------------- 원본 웹캠 영상 ↔ 모델 화면 (640 정사각)
def crop_params(w, h):
    """웹캠 원본 (w, h) → 640 정사각 화면: 가운데 정사각 잘라 640 으로 축소 (Roboflow 'Fill (with center crop)', 학습 데이터와 같은 전처리)."""
    side = min(w, h)
    return (w - side) / 2.0, (h - side) / 2.0, IMG_W / side


def to_model_view(img):
    import cv2
    h, w = img.shape[:2]; x0, y0, _ = crop_params(w, h); side = min(w, h)
    return cv2.resize(img[int(y0):int(y0) + side, int(x0):int(x0) + side], (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)


def full_to_model(uv, w, h):
    """원본 픽셀 → 640 화면 좌표 (잘린 바깥은 0~640 밖 값 — 카메라 모델은 그대로 성립)."""
    x0, y0, s = crop_params(w, h); uv = np.asarray(uv, float)
    return np.c_[(uv[..., 0] - x0) * s, (uv[..., 1] - y0) * s] if uv.ndim == 2 else np.array([(uv[0] - x0) * s, (uv[1] - y0) * s])


# ---------------------------------------------------------------- 보정 (마커 방식): 바닥 마커 모서리 ↔ 줄자 좌표
def marker_corners_floor(x, y, size, yaw):
    """바닥에 누운 ArUco 마커의 네 모서리 (OpenCV 순서: 왼위, 오위, 오아래, 왼아래).
    yaw = 마커 윗변(인쇄물 위쪽)이 가리키는 바닥 방향 (rad, 바닥 x 축 기준). 위에서 보면 마커 오른쪽 = 윗방향의 시계 방향."""
    up = np.array([math.cos(yaw), math.sin(yaw)]); right = np.array([math.sin(yaw), -math.cos(yaw)]); h = size / 2
    return np.array([[x, y] + h * (su * up + sr * right) for su, sr in ((1, -1), (1, 1), (-1, 1), (-1, -1))])


def calibrate_from_points(P, uv, c0=None, k1_sigma=0.1):
    """바닥 점 P (N,2 또는 N,3) ↔ 640 화면 픽셀 uv (N,2) → 카메라 8 파라미터 (calibrate_from_robot 과 같은 모델·출력).
    초기값은 바닥 호모그래피 분해 (Zhang 2000), 이후 재투영 오차 최소화 (k1 은 약한 사전값)."""
    import cv2
    P = np.asarray(P, float); P3 = np.c_[P[:, :2], P[:, 2] if P.shape[1] > 2 else np.zeros(len(P))]; uv = np.asarray(uv, float)
    if c0 is None:
        Hm2p, _ = cv2.findHomography(P3[:, :2], uv, 0)
        c0 = init_from_homography(Hm2p)
    lo = [-50, -50, 0.2, -2 * math.pi, 0.0, -0.5, 100, -0.6]; hi = [50, 50, 10, 2 * math.pi, 1.55, 0.5, 5000, 0.6]
    res = lambda c: np.r_[(project(c, P3) - uv).ravel(), 10.0 * c[7] / k1_sigma]
    r = least_squares(res, np.clip(np.asarray(c0, float), lo, hi), bounds=(lo, hi), loss='soft_l1', f_scale=2.0, x_scale='jac')
    rms = float(np.sqrt(np.mean((project(r.x, P3) - uv) ** 2) * 2))
    return r.x, rms


def img_from_msg(m):
    """sensor_msgs Image(bgr8/rgb8) 또는 CompressedImage → BGR 배열. 두 PC 구성에서는 웹캠 PC 가 압축 영상을 보냄."""
    import cv2
    if hasattr(m, 'format'):                                       # CompressedImage
        return cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
    return img[:, :, ::-1] if m.encoding.lower().startswith('rgb') else img
