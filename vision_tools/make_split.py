#!/usr/bin/env python3
"""세션(촬영 회차) 단위로 train / valid / test 를 나눠 YOLO 데이터셋 폴더와 data.yaml 생성.
같은 세션의 비슷한 프레임이 train 과 test 에 섞이지 않게 하는 것이 목적.

입력 폴더 규칙: 세션마다 이미지 폴더 + 같은 이름의 라벨 파일 폴더
  frames/S1/*.jpg  +  frames/S1_labels/*.txt   (prelabel.py 출력 그대로)
기존 Roboflow 데이터셋(train/valid/test 구조)도 --add-old 로 합칠 수 있음.

예) python make_split.py --out dataset --names car dummy \\
        --train frames/S1 --test frames/S2 --add-old my_data
    → dataset/train/{images,labels}, dataset/valid/..., dataset/test/..., dataset/data.yaml
--valid 를 안 주면 기존 데이터셋의 valid 를, 그것도 없으면 train 세션의 마지막 15% 를 씀 (경고 출력).
"""
import argparse
import glob
import os
import shutil

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def pairs_from_session(img_dir):
    lab_dir = os.path.normpath(img_dir) + '_labels'
    out = []
    for f in sorted(glob.glob(os.path.join(img_dir, '*'))):
        if os.path.splitext(f)[1].lower() not in IMG_EXT: continue
        lab = os.path.join(lab_dir, os.path.splitext(os.path.basename(f))[0] + '.txt')
        out.append((f, lab if os.path.exists(lab) else None))
    return out


def pairs_from_old(root, split):
    out = []
    for f in sorted(glob.glob(os.path.join(root, split, 'images', '*'))):
        lab = os.path.join(root, split, 'labels', os.path.splitext(os.path.basename(f))[0] + '.txt')
        out.append((f, lab if os.path.exists(lab) else None))
    return out


def copy(pairs, out, split, tag):
    for d in ('images', 'labels'): os.makedirs(os.path.join(out, split, d), exist_ok=True)
    n = 0
    for img, lab in pairs:
        stem = f'{tag}__' + os.path.splitext(os.path.basename(img))[0]
        shutil.copy2(img, os.path.join(out, split, 'images', stem + os.path.splitext(img)[1]))
        dst = os.path.join(out, split, 'labels', stem + '.txt')
        if lab: shutil.copy2(lab, dst)
        else: open(dst, 'w').close()          # 라벨 없는 사진 = 물체 없음(배경)
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description='세션 단위 데이터셋 분할')
    ap.add_argument('--out', required=True, help='출력 데이터셋 폴더')
    ap.add_argument('--names', nargs='+', required=True, help='클래스 이름 (모델 순서대로, 예: car dummy)')
    ap.add_argument('--train', nargs='+', default=[], help='train 에 넣을 세션 이미지 폴더들')
    ap.add_argument('--valid', nargs='+', default=[], help='valid 에 넣을 세션 이미지 폴더들')
    ap.add_argument('--test', nargs='+', default=[], help='test 에 넣을 세션 이미지 폴더들 (학습에 안 씀)')
    ap.add_argument('--add-old', default=None, help='기존 Roboflow 데이터셋 폴더 (train→train, valid→valid, test→test)')
    a = ap.parse_args()

    if os.path.exists(a.out) and os.listdir(a.out):
        raise SystemExit(f'출력 폴더가 비어 있지 않습니다: {a.out} (덮어쓰기 방지)')
    count = {'train': 0, 'valid': 0, 'test': 0}
    for split in count:
        for s in getattr(a, split):
            count[split] += copy(pairs_from_session(s), a.out, split, os.path.basename(os.path.normpath(s)))
        if a.add_old:
            count[split] += copy(pairs_from_old(a.add_old, split), a.out, split, 'old')
    if count['valid'] == 0:
        imgs = sorted(glob.glob(os.path.join(a.out, 'train', 'images', '*')))
        k = max(1, len(imgs) * 15 // 100)
        print(f'경고: valid 가 비어 train 의 마지막 {k} 장을 valid 로 옮깁니다 (같은 세션이라 valid 수치는 낙관적)')
        os.makedirs(os.path.join(a.out, 'valid', 'images'), exist_ok=True); os.makedirs(os.path.join(a.out, 'valid', 'labels'), exist_ok=True)
        for f in imgs[-k:]:
            stem = os.path.splitext(os.path.basename(f))[0]
            shutil.move(f, os.path.join(a.out, 'valid', 'images', os.path.basename(f)))
            shutil.move(os.path.join(a.out, 'train', 'labels', stem + '.txt'), os.path.join(a.out, 'valid', 'labels', stem + '.txt'))
        count['train'] -= k; count['valid'] += k
    root = os.path.abspath(a.out).replace('\\', '/')
    with open(os.path.join(a.out, 'data.yaml'), 'w', encoding='utf-8') as f:
        f.write(f'path: {root}\ntrain: train/images\nval: valid/images\n')
        if count['test']: f.write('test: test/images\n')
        f.write(f'nc: {len(a.names)}\nnames: [{", ".join(repr(n) for n in a.names)}]\n')
    print(f"완료: train {count['train']} / valid {count['valid']} / test {count['test']} 장 → {a.out}/data.yaml")


if __name__ == '__main__':
    main()
