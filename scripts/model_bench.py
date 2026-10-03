#!/usr/bin/env python3
"""
모델 벤치 (어떤 ultralytics 호환 .pt 든 같은 기준으로 비교 — YOLOv8/10/11/12/26, RT-DETR 등).
  source ~/yolo_venv/bin/activate
  python model_bench.py weights/yolo11n_amr_best.pt weights/yolo12n_amr_best.pt [--sweep] [--onnx]
결과: logs/models/<모델이름>.json + logs/models/bench.md (누적 비교표)

측정 항목 (모두 같은 조건)
 1) 구조: 레이어 수, 파라미터, GFLOPs(nc=학습 클래스 수 기준), 파일 크기, 학습 메타(ultralytics 버전·epochs·사전학습 모델)
 2) 실물 데이터셋(amr_dataset) valid/test: P, R, mAP50, mAP50-95, 클래스별 AP50-95
    ※ valid/test 대부분이 train 과 같은 원본 이미지 → 상한치 참고용 (일반화 지표 아님)
 3) 추론 지연 (704x704 입력 → imgsz 640, batch 1, PyTorch FP32): GPU / CPU, 사전·후처리 포함 / --onnx 시 ONNX CPU
 4) --sweep (시뮬 실행 중일 때만): RC카 거리 4 × 방향 8 = 32 자세, car conf 분포와 임계값별 감지율
"""
import argparse, json, math, os, subprocess, time, zipfile
import numpy as np
from ultralytics import YOLO

ROOT = os.path.expanduser('~/tb4_follow_sim'); OUT = f'{ROOT}/logs/models'; DS = f'{ROOT}/dataset'
os.makedirs(OUT, exist_ok=True)


def dataset_yaml():
    if not os.path.isdir(DS): zipfile.ZipFile('/mnt/c/Users/kangm/Downloads/amr_dataset.zip').extractall(DS)
    y = f'{DS}/data_abs.yaml'
    open(y, 'w').write(f"path: {DS}\ntrain: train/images\nval: valid/images\ntest: test/images\nnc: 2\nnames: ['car', 'dummy']\n")
    return y


def latency(m, dev, n):
    img = np.random.randint(0, 255, (704, 704, 3), np.uint8)
    for _ in range(5): m.predict(img, device=dev, verbose=False)
    t = time.perf_counter()
    for _ in range(n): m.predict(img, device=dev, verbose=False)
    return round(1000 * (time.perf_counter() - t) / n, 1)


def sweep(m, light='default'):
    """시뮬: 로봇 원점(-2.5,0) 정면 기준 RC카 배치 → car 최고 conf. light = 지금 떠 있는 월드 조건 이름(기록용)
    ※ 실행 중 gz light_config 로 조명을 바꾸면 태양광이 꺼지는 문제가 있어, 조명은 월드 파일로 바꿔 시뮬을 다시 띄운다"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    def sp(name, x, y, yaw=0.0, z=0.0):
        q = f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}'
        subprocess.run(['gz', 'service', '-s', '/world/follow_arena/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype',
                        'gz.msgs.Boolean', '--timeout', '3000', '--req',
                        f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, {q}'], capture_output=True)
    rclpy.init(); n = Node('model_bench'); box = {}
    n.create_subscription(Image, '/robot1/oakd/rgb/image_raw', lambda msg: box.__setitem__('m', msg), 1)

    def frame():
        box.clear(); t = time.time()
        while time.time() - t < 0.6 or 'm' not in box: rclpy.spin_once(n, timeout_sec=0.05)
        msg = box['m']; return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
    sp('robot1/turtlebot4', -2.5, 0, 0, 0.01); sp('pillar', 6, 5); sp('pole', 6, -5)
    res = []
    for d in (0.8, 1.2, 1.8, 2.5):
        for yd in range(0, 360, 45):
            sp('rc_car', -2.5 + d, 0, math.radians(yd)); img = frame()
            r = m.predict(img, conf=0.05, verbose=False)[0]
            cs = [float(b.conf[0]) for b in r.boxes if m.names[int(b.cls[0])] == 'car']
            ds = [float(b.conf[0]) for b in r.boxes if m.names[int(b.cls[0])] != 'car']
            res.append({'dist': d, 'yaw': yd, 'conf': round(max(cs), 3) if cs else 0.0, 'other_conf': round(max(ds), 3) if ds else 0.0})
    sp('rc_car', -1.3, 0, 0); n.destroy_node(); rclpy.shutdown()
    c = np.array([x['conf'] for x in res])
    return {'light': light, 'poses': res, 'misclass_as_dummy@0.4': round(float(np.mean([x['other_conf'] >= 0.4 and x['conf'] < 0.4 for x in res])) * 100, 1), 'mean_conf': round(float(c.mean()), 3),
            'rate@0.25': round(float((c >= 0.25).mean()) * 100, 1), 'rate@0.4': round(float((c >= 0.4).mean()) * 100, 1),
            'rate@0.5': round(float((c >= 0.5).mean()) * 100, 1),
            'by_yaw@0.4': {yd: round(float(np.mean([x['conf'] >= 0.4 for x in res if x['yaw'] == yd])) * 100) for yd in range(0, 360, 45)},
            'by_dist@0.4': {d: round(float(np.mean([x['conf'] >= 0.4 for x in res if x['dist'] == d])) * 100) for d in (0.8, 1.2, 1.8, 2.5)}}


def bench(path, args):
    import torch
    name = os.path.splitext(os.path.basename(path))[0]
    m = YOLO(path); ck = torch.load(path, map_location='cpu', weights_only=False); ta = ck.get('train_args', {})
    from ultralytics.utils.torch_utils import get_flops, get_num_params
    layers = len(m.model.model); params = get_num_params(m.model); gflops = get_flops(m.model, 640)   # 비융합 기준
    r = {'name': name, 'path': path, 'file_MB': round(os.path.getsize(path) / 1e6, 2),
         'arch_yaml': getattr(ck['model'], 'yaml', {}).get('yaml_file'), 'layers': layers, 'params': params,
         'gflops': round(gflops, 2) if gflops else None, 'ultralytics_trained': ck.get('version'), 'trained_at': ck.get('date'),
         'train': {k: ta.get(k) for k in ('model', 'epochs', 'patience', 'batch', 'imgsz', 'optimizer', 'mosaic', 'scale')}}
    y = dataset_yaml()
    for split in ('val', 'test'):
        v = m.val(data=y, split=split, imgsz=640, conf=0.001, iou=0.6, verbose=False, plots=False,
                  project=f'{OUT}/val', name=f'{name}_{split}', exist_ok=True)
        r[split] = {'P': round(float(v.box.mp), 3), 'R': round(float(v.box.mr), 3), 'mAP50': round(float(v.box.map50), 3),
                    'mAP50-95': round(float(v.box.map), 3),
                    'per_class_mAP50-95': {m.names[i]: round(float(v.box.maps[i]), 3) for i in m.names}}
    r['latency_ms'] = {'gpu_torch': latency(m, 'cuda:0', 50), 'cpu_torch': latency(m, 'cpu', 20)}
    if args.onnx:
        f = m.export(format='onnx', imgsz=640, verbose=False); mo = YOLO(f)
        r['latency_ms']['cpu_onnx'] = latency(mo, 'cpu', 20)
    if args.sweep:
        old = {}
        if os.path.exists(f'{OUT}/{name}.json'): old = json.load(open(f'{OUT}/{name}.json')).get('sim_sweep', {})
        if not isinstance(old, dict) or 'poses' in old: old = {}
        old[args.light] = sweep(m, args.light); r['sim_sweep'] = old      # 조건별 누적 (dim, bright ...)
    json.dump(r, open(f'{OUT}/{name}.json', 'w'), ensure_ascii=False, indent=1)
    return r


def table():
    rows = [json.load(open(f'{OUT}/{f}')) for f in sorted(os.listdir(OUT)) if f.endswith('.json')]
    L = ['| 모델 | 구조 | params | GFLOPs | 파일MB | valid mAP50-95 | test mAP50-95 | GPU ms | CPU ms | ONNX CPU ms | 시뮬 감지율@0.4 | 시뮬 평균 conf |',
         '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        sw = r.get('sim_sweep', {}); lt = r['latency_ms']
        s = {'rate@0.4': ' / '.join(f"{k} {v['rate@0.4']}" for k, v in sw.items()) if sw else '-',
             'mean_conf': ' / '.join(f"{k} {v['mean_conf']}" for k, v in sw.items()) if sw else '-'}
        L.append(f"| {r['name']} | {r['arch_yaml']} | {r['params']:,} | {r['gflops']} | {r['file_MB']} | {r['val']['mAP50-95']} | "
                 f"{r['test']['mAP50-95']} | {lt['gpu_torch']} | {lt['cpu_torch']} | {lt.get('cpu_onnx', '-')} | "
                 f"{s.get('rate@0.4', '-')} | {s.get('mean_conf', '-')} |")
    open(f'{OUT}/bench.md', 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('models', nargs='*')
    ap.add_argument('--sweep', action='store_true'); ap.add_argument('--onnx', action='store_true')
    ap.add_argument('--light', default='dim', help='지금 떠 있는 월드의 조명 조건 이름 (결과에 누적 저장)')
    a = ap.parse_args()
    for p in a.models: print(json.dumps(bench(os.path.abspath(p), a), ensure_ascii=False)[:400])
    table()
