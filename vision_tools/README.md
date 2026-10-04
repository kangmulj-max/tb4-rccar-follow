# 웹캠 YOLO 라벨링·판정 도구 (vision_tools)

촬영 영상 → 프레임 추출 → 기존 모델로 미리 라벨링 → 사람 검토 → 세션 단위 분할 → (재학습) → 이전/새 모델 판정.
Python 만 있으면 되고 Claude Code 는 필요 없습니다. Windows / Ubuntu 모두 동작합니다.

## 0. 설치 (처음 한 번, 약 5~10분)

```bat
:: Python 3.10~3.12 권장 (3.14 는 torch 미지원일 수 있음)
python -m venv venv
venv\Scripts\activate          :: Ubuntu: source venv/bin/activate
pip install -r requirements.txt
```
GPU 가 없어도 라벨링·판정은 CPU 로 됩니다 (사진 1 장에 약 0.1~0.5 초). 재학습은 GPU(Colab) 권장.

## 1. 전체 순서 (팀 2명, 약 2시간)

| 단계 | 명령 | 시간 |
| --- | --- | --- |
| 1 프레임 추출 | `python extract_frames.py S1.mp4 frames/S1 --fps 0.5` | 세션당 1분 |
| 2 미리 라벨링 | `python prelabel.py --model my_best.pt --images frames/S1` | 세션당 1~2분 |
| 3 사람 검토 | `frames/S1_review.csv` 에서 review=Y 인 사진만 고침 (아래 4절) | 세션당 20~30분 |
| 4 분할 | `python make_split.py --out dataset --names car dummy --train frames/S1 --test frames/S2 --add-old my_data` | 1분 |
| 5 재학습 | `python train.py --model my_best.pt --data dataset/data.yaml --epochs 30` (Colab 권장) | GPU 수 분 |
| 6 판정 | `python judge.py --old my_best.pt --new runs/detect/finetune/weights/best.pt --data dataset/test` | 1~2분 |

`--names` 는 모델 클래스 순서 그대로 (현재 웹캠 모델: `car dummy`).

## 2. 촬영 체크리스트

- [ ] 웹캠 위치·각도·해상도는 지금과 동일 (테이프로 고정). 촬영 중 웹캠을 건드리지 않기
- [ ] 화면 아래쪽 좌우 모서리와 위쪽 먼 곳까지 RC카 보내기 (지금 데이터에서 가장 빈 곳)
- [ ] 정지 장면과 주행 장면(모션블러) 반반, 차 방향은 앞·뒤·옆·대각 골고루
- [ ] 터틀봇을 화면에 넣기 (차 옆, 차를 일부 가림, 로봇만). **로봇은 라벨하지 않음** → 오검출 방지 예시가 됨
- [ ] 세션 2 는 조명을 바꿔서 (평가가 조명 변화까지 보게)

## 3. 라벨 기준 (팀 전원 동일하게)

- 박스는 차체 끝에 딱 맞게, **바퀴 포함 / 그림자 제외**
- 화면 끝에 잘린 차도 라벨 (검출은 되어야 함)
- 로봇·사람·기타 물체는 라벨하지 않음
- 처음 10 장은 두 사람이 같이 보며 기준을 맞춘 뒤 나눠서 진행

## 4. 사람 검토 방법

1. `frames/S1_preview/` 를 탐색기에서 열고, 이름 앞에 `!` 가 붙은 사진만 봅니다 (초록 = 신뢰도 높음, 빨강 = 낮음).
2. 고칠 사진은 라벨 도구로 엽니다. 이미지 폴더 = `frames/S1`, 라벨 폴더 = `frames/S1_labels` (YOLO 형식, `classes.txt` 포함).
   - Roboflow: 이미지와 txt 를 함께 업로드하면 라벨이 그대로 붙음 (업로드 시 분할은 지정하지 말고, 4단계 `make_split.py` 로 분할)
   - 설치형: labelImg / CVAT 의 YOLO 형식 불러오기
3. `review_csv` 의 reasons 뜻: `검출없음` · `신뢰도<0.6` · `가장자리`(박스가 화면 끝에 닿음) · `car 2개`(1대여야 할 때)

## 5. 판정 기준과 결과 읽기

`judge_report/report.md` 마지막 줄의 **판정**:
- **채택: 새 모델** — test 세션에서 car 검출률이 이전 이상이고 오검출 수가 이전 이하
- **보류: 이전 모델 유지** — 둘 중 하나라도 나빠짐. 새로 찍은 데이터는 다음 학습에 보탭니다

`fp/` (오검출, 빨강 박스) · `miss/` (놓침) 폴더 이미지를 꼭 눈으로 확인하세요.
박스 흔들림(px)은 위치 정확도와 직결됩니다: 1 px 줄면 RC카 위치 오차가 약 0.3 cm 줄어듭니다 (시뮬 기준).

## 6. 자주 겪는 문제

| 증상 | 해결 |
| --- | --- |
| `영상을 열 수 없습니다` | 경로에 한글·공백이 있으면 영어 폴더로 옮겨서 실행 |
| `No module named ultralytics` | 가상환경 활성화(`venv\Scripts\activate`) 후 다시 실행 |
| `출력 폴더가 비어 있지 않습니다` | 덮어쓰기 방지입니다. `--out` 에 새 폴더 이름을 주세요 |
| 판정에서 `정답이 없음` | `--data` 폴더에 `labels/` 가 있는지, 라벨에 car 가 있는지 확인 |

## 파일

| 파일 | 역할 |
| --- | --- |
| `extract_frames.py` | 영상 → 프레임 (OpenCV, ffmpeg 불필요) |
| `prelabel.py` | 기존 모델로 YOLO 라벨 + 검토 목록 + 미리보기 |
| `make_split.py` | 세션 단위 train/valid/test 분할 + data.yaml |
| `train.py` | 기존 모델에서 이어서 학습 |
| `judge.py` | 이전/새 모델 판정 보고서 |
| `run_example.bat` | 위 순서를 한 번에 보여 주는 예시 (경로만 바꿔서 사용) |
