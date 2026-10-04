#!/usr/bin/env python3
"""웹캠 녹화 (웹캠이 연결된 별도 PC 에서 실행). ROS 없이도 동작, ROS 가 있으면 저속 미리보기 발행도 가능.
  python3 webcam_rec.py --trial 261004-1430                     # 기본: 이 PC 에 녹화만 (로봇 Wi-Fi 부하 없음)
  python3 webcam_rec.py --trial 261004-1430 --show              # 화면 보면서 (q 로 종료)
  python3 webcam_rec.py --trial 261004-1430 --publish           # + ROS 토픽 /webcam/image_raw/compressed (2 Hz)
  python3 webcam_rec.py --cam test --seconds 5                  # 웹캠 없이 동작 확인 (합성 영상)
출력: <out>/<trial>/webcam_000.mp4 … (seg_min 분마다 새 파일), frames.csv, meta.json
  frames.csv: frame, seg, t_wall (이 PC 의 UTC 시각 s), t_mono, aruco_ids
시각 맞추기: (1) 두 PC 시계를 NTP 로 맞춤 (README) (2) 시작할 때 표시판 마커(ID 3)를 웹캠과 로봇 카메라에 같이 보여 줌
  → frames.csv 의 aruco_ids 와 로봇 bag 에서 ID 3 이 처음 보인 시각으로 시계 차이를 다시 확인.
"""
import argparse, csv, json, os, platform, signal, socket, sys, time
import cv2, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--trial', default=time.strftime('%y%m%d-%H%M'), help='시도 ID (record_real.sh 와 같게)')
ap.add_argument('--cam', default='0', help='웹캠 번호, 영상 파일 경로, 또는 test')
ap.add_argument('--width', type=int, default=1280); ap.add_argument('--height', type=int, default=720)
ap.add_argument('--fps', type=float, default=15)
ap.add_argument('--out', default=os.path.expanduser('~/webcam_logs'))
ap.add_argument('--seg-min', type=float, default=10, help='파일을 나누는 간격 (분) — 중간에 꺼져도 앞부분은 남음')
ap.add_argument('--no-aruco', action='store_true', help='프레임별 ArUco ID 기록 끄기 (CPU 가 부족할 때)')
ap.add_argument('--show', action='store_true')
ap.add_argument('--seconds', type=float, default=0, help='0 = Ctrl+C 까지')
ap.add_argument('--publish', action='store_true', help='ROS 2 로 압축 영상 발행 (이 PC 에 ROS 2 와 같은 네트워크 설정 필요)')
ap.add_argument('--topic', default='/webcam/image_raw/compressed')
ap.add_argument('--pub-hz', type=float, default=2.0, help='발행 주기 — 로봇과 같은 Wi-Fi 면 낮게 유지')
ap.add_argument('--pub-width', type=int, default=640)
a = ap.parse_args()

OUT = os.path.join(a.out, a.trial); os.makedirs(OUT, exist_ok=True)
if os.path.exists(os.path.join(OUT, 'frames.csv')): sys.exit(f'이미 있는 시도 ID: {OUT}')

# --- 입력
if a.cam == 'test':
    D0 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    mk = cv2.aruco.generateImageMarker(D0, 3, 160)
    def grab(i):
        img = np.full((a.height, a.width, 3), 200, np.uint8)
        x = 100 + (i * 7) % (a.width - 400)
        cv2.rectangle(img, (x, 300), (x + 120, 360), (40, 40, 40), -1)            # 움직이는 '차'
        if (i // 15) % 4 == 0: img[60:220, 60:220] = cv2.cvtColor(np.pad(mk, 0), cv2.COLOR_GRAY2BGR)  # 표시판 깜빡임
        time.sleep(1 / a.fps); return True, img
    src = 'synthetic'
else:
    cam = int(a.cam) if a.cam.isdigit() else a.cam
    cap = cv2.VideoCapture(cam, cv2.CAP_DSHOW) if (isinstance(cam, int) and os.name == 'nt') else cv2.VideoCapture(cam)
    if not cap.isOpened(): sys.exit(f'웹캠을 열 수 없음: {a.cam} (다른 번호 --cam 1, 또는 다른 프로그램이 쓰는 중인지 확인)')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))               # USB 대역폭: 720p 30 fps 가능하게
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height); cap.set(cv2.CAP_PROP_FPS, a.fps)
    if isinstance(cam, int): grab = lambda i: cap.read()
    else:                                                       # 영상 파일: 녹화 속도대로 재생 (발행 시험용)
        def grab(i): time.sleep(1 / a.fps); return cap.read()
    src = str(cam)
ok, f0 = grab(0)
if not ok: sys.exit('첫 프레임을 못 읽음')
H, W = f0.shape[:2]

det = None if a.no_aruco else cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())

# --- 선택: ROS 발행 (rclpy 가 없으면 녹화만)
pub = None
if a.publish:
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import CompressedImage
        rclpy.init(); node = rclpy.create_node('webcam_rec')
        pub = node.create_publisher(CompressedImage, a.topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        print(f'[발행] {a.topic} {a.pub_hz} Hz, 폭 {a.pub_width} px, ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}')
    except Exception as e:
        print(f'[경고] ROS 발행 불가 ({e}) → 녹화만 계속')

meta = dict(trial=a.trial, host=socket.gethostname(), os=platform.platform(), source=src, size=[W, H], fps_req=a.fps,
            start_wall=time.time(), start_iso=time.strftime('%Y-%m-%dT%H:%M:%S%z'), aruco=not a.no_aruco,
            publish=bool(pub), topic=a.topic if pub else None,
            note='t_wall 은 이 PC 시계. 로봇 bag 과 맞추려면 두 PC 시계를 NTP 로 맞추고 ID 3 표시판으로 확인')
json.dump(meta, open(os.path.join(OUT, 'meta.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)

fourcc = cv2.VideoWriter_fourcc(*'mp4v'); seg = -1; vw = None
fc = open(os.path.join(OUT, 'frames.csv'), 'w', newline=''); cw = csv.writer(fc)
cw.writerow(['frame', 'seg', 't_wall', 't_mono', 'aruco_ids'])
stop = False
def _stop(*_):
    global stop; stop = True
signal.signal(signal.SIGINT, _stop); signal.signal(signal.SIGTERM, _stop)

print(f'● 웹캠 녹화: {a.trial} → {OUT}  ({W}x{H}, 요청 {a.fps:g} fps). Ctrl+C 로 종료')
t0 = time.monotonic(); i = 0; last_pub = 0.0; img = f0; ok = True; n_aruco = 0
while not stop and ok:
    tw, tm = time.time(), time.monotonic()
    s = int((tm - t0) // (a.seg_min * 60))
    if s != seg:
        if vw: vw.release()
        seg = s; vw = cv2.VideoWriter(os.path.join(OUT, f'webcam_{seg:03d}.mp4'), fourcc, a.fps, (W, H))
    vw.write(img)
    ids = ''
    if det is not None:
        _, idv, _ = det.detectMarkers(img)
        if idv is not None: ids = ' '.join(str(int(v)) for v in sorted(idv.flatten())); n_aruco += 1
    cw.writerow([i, seg, f'{tw:.4f}', f'{tm - t0:.4f}', ids])
    if pub and tm - last_pub >= 1 / a.pub_hz:
        last_pub = tm
        small = cv2.resize(img, (a.pub_width, int(H * a.pub_width / W)))
        m = CompressedImage(); m.format = 'jpeg'; m.header.frame_id = 'webcam'
        m.header.stamp.sec = int(tw); m.header.stamp.nanosec = int((tw % 1) * 1e9)    # 캡처 시각 (이 PC 시계)
        m.data = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        pub.publish(m)
    if a.show:
        v = img.copy(); cv2.putText(v, f'{a.trial}  {time.strftime("%H:%M:%S")}  #{i}  aruco:{ids or "-"}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow('webcam_rec', v)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    if i % 150 == 0: fc.flush()
    if a.seconds and tm - t0 >= a.seconds: break
    i += 1; ok, img = grab(i)

dur = time.monotonic() - t0
if vw: vw.release()
fc.close()
meta.update(end_wall=time.time(), frames=i + 1, duration_s=round(dur, 2), fps_actual=round((i + 1) / max(dur, 1e-6), 2),
            frames_with_aruco=n_aruco, ended_by='camera_lost' if not ok else 'user')
json.dump(meta, open(os.path.join(OUT, 'meta.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
if a.cam != 'test': cap.release()
if a.show: cv2.destroyAllWindows()
if pub: rclpy.shutdown()
print(f'■ 종료: {i + 1} 프레임, {dur:.1f} s, 실제 {meta["fps_actual"]} fps, ArUco 보인 프레임 {n_aruco}  → {OUT}')
if not ok: print('영상 파일 끝' if src not in ('synthetic',) and not src.isdigit() else '[경고] 웹캠 영상이 끊겨 종료됨')
