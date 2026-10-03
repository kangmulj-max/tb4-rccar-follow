# YOLO11n vs YOLO12n — 공식 자료와 실측 대조 (RC카 추종 관점)

이 문서의 숫자는 두 종류다. **[공식]** = 논문·Ultralytics 문서 (COCO 80 클래스, 사전학습 모델) / **[실측]** = 이 프로젝트 가중치(`yolo11n_amr_best.pt`, `yolo12n_amr_best.pt`, 2 클래스)로 이 세션에서 잰 값.
다른 모델을 추가 비교할 때는 9절 절차를 그대로 따르면 같은 표에 누적된다.

## 1. 한 줄 요약
- 실물 데이터셋 지표로는 **우열을 가릴 수 없다** (둘 다 mAP50 0.995, valid/test 대부분이 train 과 같은 원본).
- **속도**는 yolo11n 이 확실히 빠르다 (GPU 1.6 배, CPU 1.2 배).
- **감지**는 조명에 따라 순위가 뒤집힌다: 밝으면 12n, 어두우면 11n. 12n 이 조명 변화에 더 민감 (59 → 28 %).
- **추종 시나리오**(움직이는 RC카)에서는 4절 매트릭스 참고.
- 실기 선택은 실습장 조명에서 bag 을 녹화해 두 모델을 같은 장면으로 재생 비교한 뒤 결정할 것.

## 2. 출처·구조 (공식 자료)

| 항목 | YOLO11 | YOLO12 |
|---|---|---|
| 개발 | Ultralytics (2024-09-27 공개) [Ultralytics YOLO11 문서] | Tian, Ye, Doermann (University at Buffalo, UCAS). arXiv 2502.12524 (2025-02-18), **NeurIPS 2025** 발표 |
| Ultralytics 지원 | 8.3.0~ (기본 모델) | 8.3.78 부터 (2025-02) |
| 핵심 구조 | C3k2 블록, SPPF, C2PSA(부분 공간 어텐션) — CNN 중심 [arXiv 2410.17725 개관] | **Area Attention**(특징맵을 4 영역으로 나눠 어텐션, 기본 l=4), **R-ELAN**(잔차 스케일 + 재설계한 특징 집계), FlashAttention 활용(옵션), 7×7 분리 합성곱 위치 인지 |
| 지원 작업 (Ultralytics) | 검출·분할·분류·포즈·OBB, 전부 사전학습 가중치 | 검출만 사전학습 가중치. 분할·분류·포즈·OBB 는 학습/추론은 되나 사전학습 가중치 없음 |
| Ultralytics 의 권고 | — | "학습 불안정, 메모리 증가, 무거운 어텐션으로 CPU 처리량 저하 가능" → 대부분의 운영 환경에 YOLO11 또는 YOLO26 권장 |
| 원저자 저장소 메모 | — | 2025-06: "공식 구현은 ultralytics 버전과 다르며 메모리·학습 안정성이 더 좋다". 2025-03 turbo 버전 공개 |
| 라이선스 | AGPL-3.0 / Enterprise | AGPL-3.0 (원저자 저장소), Ultralytics 통합본은 Ultralytics 라이선스 |

## 3. 성능 수치 대조

### 3-1. 공식 (COCO val2017, 640)
| 모델 | mAP50-95 | CPU ONNX | T4 TensorRT | params | FLOPs |
|---|---|---|---|---|---|
| YOLO11n [Ultralytics] | 39.5 | 56.1 ms | 1.5 ms | 2.6 M | 6.5 G |
| YOLO12n [Ultralytics 문서] | 40.6 | 미기재 | 1.64 ms | 2.6 M | **7.5 G** |
| YOLOv12-N [arXiv v1] | 40.6 | — | 1.64 ms | 2.6 M | 6.5 G |
| YOLOv12-N [NeurIPS 판] | 40.5 | — | 1.62 ms | — | — |
| YOLOv12-N turbo [원저자 GitHub] | 40.4 | — | 1.60 ms | 2.5 M | 6.0 G |

→ 같은 "YOLO12n" 이라도 판본마다 수치가 다르다. FLOPs 는 Ultralytics 통합본(7.5 G)이 논문(6.5 G)보다 크다.
→ 논문 주장: YOLOv12-N 이 YOLO11-N 보다 mAP +1.2 (NeurIPS 판 +1.1), 속도는 비슷.

### 3-2. 실측 (이 프로젝트 가중치)
| 항목 | yolo11n_amr | yolo12n_amr | 공식과 대조 |
|---|---|---|---|
| 학습 조건 | Colab, ultralytics 8.4.171, COCO 사전학습 → 100 epoch, patience 20, batch 32, 640, 같은 증강 | 동일 | 두 모델 **같은 조건** — 공정 비교 가능 |
| 파라미터 | 2,590,230 | 2,568,438 | 공식 2.6 M 과 일치 |
| GFLOPs (2 클래스, 비융합) | 6.5 | 7.46 | 12n 이 더 큼 = Ultralytics 문서 7.5 G 와 일치 (논문 6.5 G 와 불일치) |
| 파일 | 5.48 MB | 5.53 MB | |
| 어텐션 구현 | — | **FlashAttention 미사용** (8.4.171 `AAttn` 은 일반 행렬곱+softmax) | 문서의 FlashAttention 가속은 이 환경에 해당 없음 |
| 실물 valid mAP50 / 50-95 | 0.995 / **0.971** | 0.995 / 0.950 | 포화 — 아래 4절 주의 |
| 실물 test mAP50 / 50-95 | 0.995 / 0.905 | 0.995 / 0.903 | 포화 |
| test 클래스별 mAP50-95 car / dummy | 0.854 / 0.956 | **0.903** / 0.903 | 따라갈 대상 'car' 는 12n 이 높음 (12 장 표본) |
| 지연 GPU (RTX 3060, PyTorch FP32, 704→640, 전후처리 포함) | **6.8 ms** | 10.7 ms | 공식 T4 TensorRT 1.5 vs 1.64 ms 보다 차이가 큼 (FP32·비TensorRT·FlashAttention 없음) |
| 지연 CPU (PyTorch) | **22.3 ms** | 27.6 ms | Ultralytics "12 는 CPU 처리량 저하" 경고와 방향 일치 |

(지연은 시뮬을 끈 상태 측정값. 시뮬이 돌 때는 GPU 9.2/13.3, CPU 63/92 ms 로 늘어남 — 실기 노트북에서도 다른 프로그램 부하에 따라 달라짐)

### 3-3. 시뮬 감지율 (같은 32 자세: 거리 0.8/1.2/1.8/2.5 m × 방향 8, conf ≥ 0.4)
| 조명 | yolo11n | yolo12n |
|---|---|---|
| 밝음 (`follow_arena_bright`) | 46.9 % (평균 conf 0.41) | **59.4 %** (0.49) |
| 어두움 (`follow_arena`, 매트릭스 조건) | **62.5 %** (0.54), dummy 오분류 25 % | 28.1 % (0.26), dummy 오분류 37.5 % |

- 12n 은 조명이 어두워지면 감지율이 절반 이하로 떨어지고 RC카를 dummy 로 오분류하는 비율이 커진다.
- 11n 은 두 조명에서 47~63 % 로 상대적으로 안정적이지만, 밝은 조명에서 정면·정후면(0°, 180°)을 전혀 못 잡는 등 자세별 편차가 크다.
- 두 모델 모두 **옆모습(90°, 270°)** 이 약하다 — 학습 데이터에 옆모습이 적고 dummy(검은 상자)와 실루엣이 비슷함. 녹화 영상에서도 옆모습이 `dummy 0.44` 로 찍힌 장면 확인.
- 표본: 자세당 1 프레임, 32 자세. 렌더링은 결정적이라 잡음은 없지만 자세 수가 적어 ±10 %p 차이는 의미 있게 보지 말 것.
- 시뮬 RC카는 사진이 아니라 단순 도형 모델 → 실물과 외형이 다르다. **절대 수치가 아니라 '조건이 바뀌면 순위가 바뀐다' 는 경향**이 결론.

## 4. 추종 매트릭스 (움직이는 RC카, v3 / v3_safe)
(최종 재실험 결과 — `logs/compare.md` 에서 생성. 아래 표는 run_all_v2 완료 후 채움)

<!-- MATRIX -->

## 5. 기능 비교 (이 프로젝트 관점)

| 기능 | yolo11n | yolo12n | 비고 |
|---|---|---|---|
| ultralytics `track()` (ByteTrack/BoT-SORT) | ✓ | ✓ | 같은 API |
| ONNX 내보내기 | ✓ | ✓ | 이 세션에서 둘 다 성공. 단 내보내기 시 패키지 자동 설치가 numpy 를 2.x 로 올려 cv_bridge 를 깨뜨림 (E6) |
| TensorRT / OpenVINO 등 | ✓ | ✓ (어텐션 연산 지원 여부는 대상별 확인 필요) | 미검증 |
| 노트북 CPU 실시간 (10 Hz 카메라) | 여유 있음 (22 ms) | 가능 (28 ms) | 다른 부하가 있으면 차이 커짐 |
| 학습 안정성 | 문제 보고 없음 | Ultralytics 가 불안정 가능성 경고 | 이 데이터에서는 둘 다 수렴 |

## 6. 결론 (현재 근거 기준)
1. 실물 지표로는 차이 없음 → 지표로 고르지 말 것.
2. 속도·안정성(조명 변화)은 11n 우세, 밝은 조건의 다양한 자세 감지는 12n 우세.
3. 실습장 형광등은 밝은 편 → 12n 이 유리할 가능성이 있지만, 창가 역광·저녁 조명에서 급락 위험.
4. **결정 방법**: 실기에서 `record_real.sh` 로 RC카 주행 bag 을 조명별로 녹화 → 같은 bag 을 두 모델로 재생 → 추종 유지율·놓침 횟수 비교 (실험일지에 기록).
5. 근본 개선은 데이터: 다른 장소·조명·옆모습 추가, valid/test 를 train 과 다른 원본으로.

## 7. 주의할 오해
- "12 가 11 보다 최신이니 더 좋다" — 공식 COCO 기준 +1.1 mAP 이지만, 소규모 커스텀 데이터·다른 하드웨어에서는 보장되지 않는다 (이 프로젝트에서 조명 따라 역전).
- "FLOPs 가 같으니 속도도 같다" — 어텐션은 FLOPs 대비 메모리 접근이 많아 FlashAttention 없이 느리다 (실측 1.6 배).
- "mAP 0.995 니 거의 완벽" — 같은 원본의 중복 이미지로 평가된 값.

## 8. 참고문헌
- Ultralytics YOLO11 문서 https://docs.ultralytics.com/models/yolo11/
- Ultralytics YOLO12 문서 https://docs.ultralytics.com/models/yolo12/ , v8.3.78 릴리스 https://community.ultralytics.com/t/new-release-ultralytics-v8-3-78/787
- Ultralytics YOLO26 문서 (차기 비교 후보: NMS-free, CPU ONNX 38.9 ms, COCO 40.9) https://docs.ultralytics.com/models/yolo26/
- Y. Tian, Q. Ye, D. Doermann, "YOLOv12: Attention-Centric Real-Time Object Detectors," NeurIPS 2025, arXiv:2502.12524. 원저자 코드 https://github.com/sunsmarterjie/yolov12
- R. Khanam, M. Hussain, "YOLOv11: An Overview of the Key Architectural Enhancements," arXiv:2410.17725

## 9. 다른 모델 추가 비교 절차
1. 같은 데이터(`amr_dataset.zip`)·같은 학습 조건(3-2 표의 학습 조건)으로 학습한 `.pt` 를 `weights/` 에 넣는다 (이름 예: `yolo26n_amr_best.pt`).
2. 오프라인 지표·지연: `python scripts/model_bench.py weights/<모델>.pt` (시뮬 끈 상태)
3. 조명별 감지: 시뮬을 `world_file:=follow_arena_bright` / 기본으로 각각 띄우고 `--sweep --light bright|dim`
4. 추종 매트릭스: `run_follow.sh` 는 `yolo${V}n_amr_best.pt` 를 찾으므로 V 에 `26` 처럼 넣으면 됨 → `NODE=v3safe bash experiment.sh square 26` 등
5. `python3 scripts/summarize2.py` 와 `logs/models/bench.md` 에 열이 추가됨 (summarize2 의 cols 에 모델 추가 필요)
6. 실기: 같은 bag 재생으로 비교, 실험일지에 모델 이름 기록
