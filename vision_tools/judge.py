#!/usr/bin/env python3
"""이전 모델 vs 새 모델을 '학습에 쓰지 않은' 세트에서 같은 기준으로 판정.

예) python judge.py --old my_best.pt --new runs/detect/train/weights/best.pt --data dataset/test
    (--data 는 images/ 와 labels/ 가 있는 폴더. make_split.py 의 test 폴더 그대로)
출력: judge_report/report.md (판정표), per_image.csv, fp/ (오검출 장면 이미지), miss/ (놓친 장면)
판정 기준 (미리 정해 둔 합격선):
  1) 새 모델의 car 검출률(recall) ≥ 이전 모델
  2) 새 모델의 오검출(FP) 수 ≤ 이전 모델
  둘 다 만족 → 새 모델 채택 / 아니면 이전 모델 유지
추가 지표: 정밀도, 박스 변 오차(px, 평균=치우침 / 표준편차=흔들림) — 위치 정확도에 직접 영향.
"""
import argparse
import csv
import glob
import os

import cv2
import numpy as np
from ultralytics import YOLO

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def imwrite(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1] or '.jpg', img)
    if ok: buf.tofile(path)


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1])); i = ix * iy
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def load_gt(lab, w, h):
    out = []
    if os.path.exists(lab):
        for line in open(lab, encoding='utf-8'):
            p = line.split()
            if len(p) >= 5:
                c, x, y, bw, bh = int(p[0]), *map(float, p[1:5])
                out.append((c, [(x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h]))
    return out


def evaluate(model_path, files, data, conf, iou_thr, tag, out_dir, save_imgs):
    m = YOLO(model_path); names = m.names
    st = {c: dict(tp=0, fp=0, fn=0, err=[]) for c in names}
    per = []
    for f in files:
        r = m.predict(f, conf=conf, verbose=False)[0]
        h, w = r.orig_shape
        gt = load_gt(os.path.join(data, 'labels', os.path.splitext(os.path.basename(f))[0] + '.txt'), w, h)
        pr = [(int(b.cls), b.xyxy[0].tolist(), float(b.conf)) for b in r.boxes]
        used, fps, misses = set(), [], []
        for c, g in gt:                              # 정답마다 같은 클래스 예측 중 IoU 최대를 짝지음
            best, bi = 0.0, None
            for k, (pc, p, _) in enumerate(pr):
                if pc == c and k not in used and iou(p, g) > best: best, bi = iou(p, g), k
            if bi is not None and best >= iou_thr:
                used.add(bi); st[c]['tp'] += 1; st[c]['err'].append(np.array(pr[bi][1]) - np.array(g))
            else:
                st[c]['fn'] += 1; misses.append((c, g))
        for k, (pc, p, s) in enumerate(pr):
            if k not in used: st[pc]['fp'] += 1; fps.append((pc, p, s))
        per.append(dict(model=tag, image=os.path.basename(f), gt=len(gt), pred=len(pr), fp=len(fps), miss=len(misses)))
        if save_imgs and (fps or misses):
            img = r.orig_img.copy()
            for c, g in gt: cv2.rectangle(img, tuple(map(int, g[:2])), tuple(map(int, g[2:])), (0, 200, 0), 2)
            for pc, p, s in fps:
                cv2.rectangle(img, tuple(map(int, p[:2])), tuple(map(int, p[2:])), (0, 0, 255), 2)
                cv2.putText(img, f'FP {names[pc]} {s:.2f}', (int(p[0]), max(15, int(p[1]) - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            for c, g in misses:
                cv2.putText(img, f'MISS {names[c]}', (int(g[0]), int(g[3]) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
            sub = 'fp' if fps else 'miss'
            os.makedirs(os.path.join(out_dir, sub), exist_ok=True)
            imwrite(os.path.join(out_dir, sub, f'{tag}__{os.path.basename(f)}'), img)
    return names, st, per


def summary(names, st):
    rows = []
    for c, s in st.items():
        tp, fp, fn = s['tp'], s['fp'], s['fn']
        if tp + fp + fn == 0: continue
        e = np.array(s['err']) if s['err'] else np.zeros((0, 4))
        rows.append(dict(cls=names[c], tp=tp, fp=fp, fn=fn,
                         recall=tp / (tp + fn) if tp + fn else float('nan'),
                         precision=tp / (tp + fp) if tp + fp else float('nan'),
                         bias=np.round(e.mean(0), 1).tolist() if len(e) else '-',
                         jitter=np.round(e.std(0), 1).tolist() if len(e) else '-'))
    return rows


def main():
    ap = argparse.ArgumentParser(description='이전/새 YOLO 모델 판정')
    ap.add_argument('--old', required=True, help='이전 모델 .pt')
    ap.add_argument('--new', default=None, help='새 모델 .pt (없으면 이전 모델만 평가)')
    ap.add_argument('--data', required=True, help='images/ labels/ 가 있는 평가 폴더 (학습에 안 쓴 세트)')
    ap.add_argument('--target', default='car', help='판정 기준 클래스 (기본 car)')
    ap.add_argument('--conf', type=float, default=0.4, help='검출로 셀 최소 신뢰도 (추종 노드 기본값 0.4)')
    ap.add_argument('--iou', type=float, default=0.5, help='정답과 짝지을 최소 IoU (기본 0.5)')
    ap.add_argument('--out', default='judge_report', help='결과 폴더')
    ap.add_argument('--no-images', action='store_true', help='오검출/놓침 이미지 저장 안 함')
    a = ap.parse_args()

    files = sorted(f for f in glob.glob(os.path.join(a.data, 'images', '*')) if os.path.splitext(f)[1].lower() in IMG_EXT)
    if not files: raise SystemExit(f'평가 이미지가 없습니다: {a.data}/images')
    os.makedirs(a.out, exist_ok=True)
    res, per_all = {}, []
    for tag, path in (('old', a.old), ('new', a.new)):
        if not path: continue
        names, st, per = evaluate(path, files, a.data, a.conf, a.iou, tag, a.out, not a.no_images)
        res[tag] = summary(names, st); per_all += per

    L = [f'# 모델 판정 보고서', '', f'- 평가 세트: `{a.data}` ({len(files)} 장), conf ≥ {a.conf}, IoU ≥ {a.iou}',
         f'- 이전 모델: `{a.old}`' + (f'\n- 새 모델: `{a.new}`' if a.new else ''), '',
         '| 모델 | 클래스 | 맞춤(TP) | 오검출(FP) | 놓침(FN) | 검출률 | 정밀도 | 박스 치우침 [x1,y1,x2,y2] px | 박스 흔들림 px |',
         '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for tag, rows in res.items():
        for r in rows:
            L.append(f"| {tag} | {r['cls']} | {r['tp']} | {r['fp']} | {r['fn']} | {r['recall']:.3f} | {r['precision']:.3f} | {r['bias']} | {r['jitter']} |")
    verdict = '새 모델 없음 — 이전 모델 기준값만 기록'
    if 'new' in res:
        pick = lambda tag: next((r for r in res[tag] if r['cls'] == a.target), None)
        o, n = pick('old'), pick('new')
        fp_o, fp_n = sum(r['fp'] for r in res['old']), sum(r['fp'] for r in res['new'])
        if o and n:
            c1 = n['recall'] >= o['recall']; c2 = fp_n <= fp_o
            verdict = ('채택: 새 모델' if c1 and c2 else '보류: 이전 모델 유지') + \
                      f"  ({a.target} 검출률 {o['recall']:.3f} → {n['recall']:.3f} {'OK' if c1 else 'NG'}, " \
                      f"전체 오검출 {fp_o} → {fp_n} {'OK' if c2 else 'NG'})"
        else:
            verdict = f'판정 불가: 평가 세트에 {a.target} 정답이 없음'
    L += ['', f'**판정: {verdict}**', '',
          '읽는 법: 검출률 = 정답 중 찾은 비율, 정밀도 = 검출 중 맞은 비율. 박스 흔들림이 1 px 줄면 위치 오차가 약 0.3 cm 줄어듦(시뮬 기준).',
          '`fp/`, `miss/` 폴더의 이미지로 오검출·놓침 장면을 눈으로 확인하세요 (초록=정답, 빨강=오검출).']
    open(os.path.join(a.out, 'report.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    with open(os.path.join(a.out, 'per_image.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        wr = csv.DictWriter(f, fieldnames=per_all[0].keys()); wr.writeheader(); wr.writerows(per_all)
    print('\n'.join(L[5:]))
    print(f'\n결과: {a.out}/report.md, per_image.csv, fp/, miss/')


if __name__ == '__main__':
    main()
