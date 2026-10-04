#!/usr/bin/env python3
"""기존 YOLO 모델로 미리 라벨링 → 사람은 표시된 사진만 검토.

예) python prelabel.py --model my_best.pt --images frames/S1
출력 (images 폴더 옆에 생성):
  frames/S1_labels/*.txt      YOLO 형식 라벨 (Roboflow·CVAT·labelImg 로 불러와 수정)
  frames/S1_labels/classes.txt
  frames/S1_review.csv        사진별 검출 수·최저 신뢰도·검토 필요 여부와 이유
  frames/S1_preview/*.jpg     박스를 그린 미리보기 (검토 필요한 사진은 파일명 앞에 '!')
검토 필요 기준 (기본값, 옵션으로 변경 가능):
  - 검출 없음  - 신뢰도 < --review-conf (0.6)  - 박스가 화면 가장자리에 닿음
  - 같은 클래스가 --max-per-class 개 초과 (기본 car 1 대 가정)
"""
import argparse
import csv
import glob
import os

import cv2
from ultralytics import YOLO


def imwrite(path, img):
    """한글 경로에서도 저장되도록 (cv2.imwrite 는 Windows 한글 경로에서 조용히 실패함)."""
    ok, buf = cv2.imencode(os.path.splitext(path)[1] or '.jpg', img)
    if ok: buf.tofile(path)
    return ok

IMG_EXT = ('*.jpg', '*.jpeg', '*.png', '*.bmp')


def main():
    ap = argparse.ArgumentParser(description='YOLO 미리 라벨링 + 검토 목록')
    ap.add_argument('--model', required=True, help='가중치 파일 (.pt)')
    ap.add_argument('--images', required=True, help='이미지 폴더')
    ap.add_argument('--conf', type=float, default=0.25, help='라벨로 저장할 최소 신뢰도 (기본 0.25)')
    ap.add_argument('--review-conf', type=float, default=0.6, help='이보다 낮으면 검토 필요 (기본 0.6)')
    ap.add_argument('--max-per-class', type=int, default=1, help='클래스당 기대 개수, 넘으면 검토 (기본 1)')
    ap.add_argument('--edge', type=int, default=3, help='가장자리 판정 여유 px (기본 3)')
    ap.add_argument('--imgsz', type=int, default=640)
    a = ap.parse_args()

    base = os.path.normpath(a.images)
    lab_dir, prev_dir, review_csv = base + '_labels', base + '_preview', base + '_review.csv'
    os.makedirs(lab_dir, exist_ok=True); os.makedirs(prev_dir, exist_ok=True)
    files = sorted(f for e in IMG_EXT for f in glob.glob(os.path.join(base, e)))
    if not files:
        raise SystemExit(f'이미지가 없습니다: {base}')

    model = YOLO(a.model)
    names = model.names
    with open(os.path.join(lab_dir, 'classes.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(names[i] for i in sorted(names)) + '\n')

    rows, need = [], 0
    for path in files:
        r = model.predict(path, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
        h, w = r.orig_shape
        lines, reasons, counts, min_conf = [], [], {}, 1.0
        img = r.orig_img.copy()
        for b in r.boxes:
            c, p = int(b.cls), float(b.conf)
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            lines.append(f'{c} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}')
            counts[c] = counts.get(c, 0) + 1
            min_conf = min(min_conf, p)
            if x1 <= a.edge or y1 <= a.edge or x2 >= w - a.edge or y2 >= h - a.edge:
                reasons.append('가장자리')
            col = (0, 200, 0) if p >= a.review_conf else (0, 0, 255)
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
            cv2.putText(img, f'{names[c]} {p:.2f}', (int(x1), max(15, int(y1) - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        if not lines: reasons.append('검출없음')
        if lines and min_conf < a.review_conf: reasons.append(f'신뢰도<{a.review_conf}')
        for c, k in counts.items():
            if k > a.max_per_class: reasons.append(f'{names[c]} {k}개')
        stem = os.path.splitext(os.path.basename(path))[0]
        with open(os.path.join(lab_dir, stem + '.txt'), 'w') as f:
            f.write('\n'.join(lines) + ('\n' if lines else ''))
        flag = bool(reasons); need += flag
        imwrite(os.path.join(prev_dir, ('!' if flag else '') + stem + '.jpg'), img)
        rows.append(dict(image=os.path.basename(path), detections=len(lines),
                         min_conf=round(min_conf, 3) if lines else '', review='Y' if flag else '',
                         reasons=' / '.join(dict.fromkeys(reasons))))

    with open(review_csv, 'w', newline='', encoding='utf-8-sig') as f:
        wr = csv.DictWriter(f, fieldnames=rows[0].keys()); wr.writeheader(); wr.writerows(rows)
    print(f'완료: {len(files)} 장 중 검토 필요 {need} 장')
    print(f'  라벨      → {lab_dir}')
    print(f'  검토 목록 → {review_csv}  (엑셀로 열림, review=Y 만 보면 됨)')
    print(f'  미리보기  → {prev_dir}  (파일명 앞 ! = 검토 필요)')


if __name__ == '__main__':
    main()
