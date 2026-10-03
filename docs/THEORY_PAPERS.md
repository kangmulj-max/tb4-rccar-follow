# 실기 수정(S13~S15)의 이론 근거와 관련 논문 해설

2026-10-03 실기 연동에서 바꾼 부분이 어떤 이론에 기대는지, 그 근거 논문이 정확히 무엇을 말하는지 정리한다.
기존 제어·추적·검출기 문헌(시각 서보 [1][2][3], ByteTrack [4], 추종 로봇 [6]~[9], YOLO [10][11])은
`FOLLOW_NODE_EXPLAINED.md` 7절에 있고, 여기서는 오늘 새로 쓴 이론만 다룬다.
서지는 DOI 를 Crossref 로 대조해 확인했다 (확인 못 한 것은 표시).

| 수정 | 문제 (실측) | 쓰인 이론 | 근거 |
|---|---|---|---|
| S13 압축 영상 구독 | raw 구독 시 RGB 15~20 s 지연, 동기화 0 회 | 센서 QoS·전송 대역폭 | [R8][R9] |
| S14 `ground_only` | 벽 TV 를 car 로 → 거리 7 m | 지면 평면·지평선 제약 | [R1] |
| S14 `min_fresh` | 재획득 직후 1 프레임 튐 | 강건 통계 — 중앙값 필터 | [R2][R3] |
| S14 `cut_bottom_px` | 0.45 m 에서 지붕을 읽어 +23 cm | 스테레오 최소거리·가시성 | [R4][R5] |
| (기존 S3) 속도 비례 정지거리 | — | 허용 속도(Dynamic Window) | [R6] |
| S15 탐색 | 172° 에서 포기, LOP 에서 대기 | 표적 소실 후 예측·탐색 | [R7][R10] |
| (제안) 라이다 보조 | 차가 라이다보다 낮아 15~65 % 만 보임 | 2D 레이저 군집·추적 | [R11][R12] |
| (질문) Nav2 | 뒤의 차 인식 | 내비게이션 스택 (인식 아님) | [R13][R14] |

---

## 1. 지면 평면·지평선 제약 — S14 `ground_only`

**[R1] D. Hoiem, A. A. Efros, M. Hebert, "Putting Objects in Perspective," *IJCV* 80(1):3–15, 2008.** doi:10.1007/s11263-008-0137-5
(학회판: *CVPR 2006*, vol. 2, pp. 2137–2144, doi:10.1109/CVPR.2006.232)

- 논문 요지: 물체 검출을 이미지 안에서만 판단하지 말고 **카메라 높이·지평선과 물체의 바닥 접점**을 함께 추론하면 오검출이 줄어든다.
  바닥에 놓인 물체는 바닥 접점 행 $v_b$ 가 지평선 $v_0$ 보다 아래에 있어야 하고, 크기·거리·접점 위치가 서로 맞아야 한다.
- 이 노드에서: 카메라 광축이 거의 수평이므로 지평선 ≈ 주점 $c_y$ (=355 px).
  RC카는 바닥 위에 있으므로 박스 아래 끝 $y_2 > c_y$ 여야 한다. 벽 TV 박스는 $y_2 = 149$ → 기하학적으로 바닥 물체일 수 없음 → 버림.
- 실측 효과: 오늘 bag 재생에서 7 m 튐 3 → 0, TRACKING 프레임 +12 %.
- 한계: 논문은 접점 행으로 **거리까지** 추정하지만, 우리 실측에서는 박스 바닥(그림자 포함)으로 잰 거리가 depth 와 10~24 % 어긋났다 → 거리 추정에는 쓰지 않고 '지평선 위/아래' 판정에만 쓴다.

## 2. 강건 통계 — 중앙값 필터와 최소 표본 수 — S14 `min_fresh`

**[R2] F. R. Hampel, "The Influence Curve and Its Role in Robust Estimation," *JASA* 69(346):383–393, 1974.** doi:10.1080/01621459.1974.10482962
**[R3] R. K. Pearson, "Outliers in Process Modeling and Identification," *IEEE Trans. Control Systems Technology* 10(1):55–63, 2002.** doi:10.1109/87.974338

- [R2] 요지: 추정량이 이상치 하나에 얼마나 끌려가는지(영향 함수)와 몇 % 까지 오염돼도 버티는지(붕괴점)를 정의. **평균은 붕괴점 0, 중앙값은 50 %** — 값 하나가 7 m 로 튀어도 중앙값은 움직이지 않는다.
- [R3] 요지: 시계열에서 이동 창 중앙값 + MAD 로 이상치를 걸러내는 필터(이른바 Hampel 필터)를 공정 데이터에 적용.
- 이 노드에서: 원래도 최근 5 개 거리의 중앙값을 썼지만, 재획득 직후에는 이력이 비어 **표본 1 개 = 그 값 그대로** 였다 (붕괴점 0 과 같음).
  `min_fresh=3` 은 "이상치 1 개를 이길 수 있는 가장 작은 중앙값 창(3 개 중 2 개가 정상)" 이라는 공학적 선택이다 — 논문이 3 을 직접 제시한 것은 아님.
- 비용: 재획득 후 전·후진 시작이 약 0.2~0.3 s 늦어진다 (회전은 즉시).

## 3. 스테레오 depth 의 오차와 최소거리 — S14 `cut_bottom_px`

**[R4] D. Gallup, J.-M. Frahm, P. Mordohai, M. Pollefeys, "Variable Baseline/Resolution Stereo," *CVPR 2008*, pp. 1–8.** doi:10.1109/CVPR.2008.4587671
**[R5] Luxonis, "Configuring Stereo Depth" (공식 문서).** https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth

- 스테레오 기본식: $Z = fB/d$ ( $f$ 초점거리[px], $B$ 기선, $d$ 시차[px] ).
  - 오차: 시차 오차 $\Delta d$ 가 일정하면 $\Delta Z \approx \frac{Z^2}{fB}\Delta d$ — **거리 제곱에 비례**해 커진다 [R4 의 출발점].
    오늘 실측도 같은 경향: 0.7 m σ 1 mm, 0.86 m σ 3 mm, 2~3 m 에서 잡음 중앙값 1.1 cm.
  - 최소거리: 시차 탐색 범위 $d_{max}$ 가 유한하므로 $Z_{min} = fB/d_{max}$.
    [R5] 의 OAK-D 예: $f$=882.5 px, $B$=7.5 cm, $d_{max}$=95 → 약 0.70 m; extended disparity(탐색 191) 이면 약 0.35 m.
    (Luxonis 문서는 오차 증가를 'exponential' 로 적었지만 식은 2 차 — 식을 인용할 것)
- 이 노드에서 실제로 걸린 것은 **최소거리보다 '가시성'** 이었다: 카메라 0.45 m 에서도 depth 는 유효하게 나왔지만(유효율 100 %),
  RC카 뒷면이 화면 아래로 잘려 depth 패치가 **뒤로 이어지는 지붕 윗면**을 읽었다 → +23 cm, 클래스도 dummy 로 바뀜.
  그래서 "박스가 화면 하단에 닿으면 거리를 믿지 않는다"(가시성 조건)로 막았다. 잘림 시작은 $y_2 \propto 1/Z$ 로 0.70 m 의 $y_2$=607 에서 외삽해 약 0.50 m.
- 가즈보 적용: depth 센서에 $\sigma_Z = k Z^2$ 잡음, 최소거리 컷, 카메라 FOV 아래로 잘린 물체의 depth 는 '보이는 면'(지붕)으로 나오도록 모델링.

## 4. 속도에 비례하는 정지거리 — (기존 S3) `stop_base + stop_gain·v`

**[R6] D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to Collision Avoidance," *IEEE Robotics & Automation Magazine* 4(1):23–33, 1997.** doi:10.1109/100.580977

- 요지: 가능한 속도 중 **최대 감속으로 장애물 앞에서 멈출 수 있는 속도만 허용**: $v \le \sqrt{2\,\mathrm{dist}(v,\omega)\,\dot v_b}$.
- 이 노드에서: 같은 원리를 선형 근사한 $d_{stop} = 0.12 + 0.6v$ 를 쓴다 (0.1 m/s → 0.18 m). 근거리 추종에서는
  RC카가 라이다 평면보다 낮아 **RC카에 대해서는 이 게이트가 거의 작동하지 않고** `min_dist`(depth) 가 대신한다 — 그래서 depth 신뢰성(3 절)이 곧 안전 문제다.
- 「주의」 실기 정지거리는 아직 실측 보정 안 함 (odom 이 명령보다 최대 0.136/0.10 m/s 로 오버슈트, T7).

## 5. 표적 소실 후 예측과 탐색 — S15

**[R7] M. J. Islam, J. Hong, J. Sattar, "Person-following by autonomous robots: A categorical overview," *IJRR* 38(14):1581–1618, 2019.** doi:10.1177/0278364919881683, arXiv:1803.08202 (= 기존 [9])
**[R10] R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems," *Trans. ASME J. Basic Engineering* 82(1):35–45, 1960.** doi:10.1115/1.3662552
(참고, 재식별 전용) K. Koide, J. Miura, "Identification of a specific person using color, height, and gait features for a person following robot," *RAS* 84:76–87, 2016. doi:10.1016/j.robot.2016.07.004

- [R7] 요지(원문): "Predictive and probabilistic models such as Kalman filters, particle filters, etc., are typically used to estimate the person's location in future, which can be used as prior knowledge in case of a missing target situation … the recovery planner can use that anticipated person's location as prior and search probable locations."
  → 표적을 놓치면 **마지막 위치/예측 위치를 사전정보로 탐색**하는 것이 분야의 표준 구조.
- [R10] 요지: 측정이 없을 때 예측 단계만 돌려 상태를 전파 $\hat x_{k|k-1} = F\hat x_{k-1}$. 등속 모델이면 화면 밖 RC카의 예상 위치를 계속 낼 수 있다.
- 이 노드의 현재 구현: LOP(마지막 관측 위치, odom 좌표, [6] 의 개념) 쪽으로 회전 → 0.7 s 확인 → **같은 방향으로 계속 회전**(S15).
  "같은 방향"은 차가 사라진 쪽 = 그쪽으로 계속 움직였을 것이라는 **가장 단순한 등속 가정**이다.
- 실측 근거: 원본 0.5 rad/s × 6 s = 172° → 포기 탐색 27 회 중 13 회가 165~174° (뒤를 보기 직전 포기). 수정 후 10 s 에 411° (오프라인 로직 시험).
- 다음 단계(제안): TRACKING 중 LOP 이력으로 RC카의 odom 속도를 칼만 필터로 추정 → 사라진 뒤 예측 위치 방향으로 회전 (C 안).
- Koide & Miura 는 놓친 뒤 **같은 사람인지 다시 알아보는**(색·키·걸음걸이) 재식별 논문이지 탐색 전략 논문이 아님 — 우리 REID(색 히스토그램)의 비교 대상.

## 6. 2D 레이저로 주변 물체 검출·추적 — (제안) 뒤쪽 보조 감지

**[R11] K. O. Arras, Ó. M. Mozos, W. Burgard, "Using Boosted Features for the Detection of People in 2D Range Data," *ICRA 2007*, pp. 3402–3407.** doi:10.1109/ROBOT.2007.363998
**[R12] A. Leigh, J. Pineau, N. Olmedo, H. Zhang, "Person Tracking and Following with 2D Laser Scanners," *ICRA 2015*, pp. 726–733.** doi:10.1109/ICRA.2015.7139259

- [R11]: 스캔을 구간(segment)으로 나누고 폭·직선성·원형성 같은 기하 특징을 AdaBoost 로 분류해 사람(다리)을 찾는다.
- [R12]: 낮게 단 2D 레이저로 다리 군집을 검출, 칼만 필터 + 데이터 연관으로 추적해 실제 로봇이 사람을 따라간다.
- 이 노드에 적용하면: 360° 라이다에서 뒤쪽의 '움직이는 작은 군집'을 찾아 회전 방향을 정하는 보조 신호.
  **한계(실측)**: RC카 높이 ~0.18 m < 라이다 0.19 m → 오늘 라이다가 차를 본 비율 15~65 %. 신뢰할 수 있는 단독 센서는 아님.

## 7. Nav2 — 뒤의 RC카를 인식할 수 있나?

**[R13] S. Macenski, F. Martín, R. White, J. Ginés Clavero, "The Marathon 2: A Navigation System," *IROS 2020*, pp. 2718–2725.** doi:10.1109/IROS45743.2020.9341207, arXiv:2003.00368
**[R14] Nav2 튜토리얼 "Dynamic Object Following" (Jazzy).** https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_dynamic_point_following/navigation2_dynamic_point_following/

- [R13] 요지: 행동 트리(BT) 기반으로 계획기·제어기·코스트맵·복구 동작을 조합하는 내비게이션 시스템. **입력은 센서(코스트맵)와 목표 자세이고, 물체를 알아보는 기능은 없다.**
- [R14] (Jazzy): 다른 노드가 내보내는 '따라갈 점'을 목표로 ComputePathToPose + FollowPath 를 반복 — 즉 **검출은 여전히 우리 카메라 노드가 해야** 한다.
  Rolling 버전에는 `desired_distance` 와 표적 소실 시 좌/우 회전 탐색을 가진 Following Server 가 있지만 **Jazzy 에는 없음**.
- 결론: Nav2 로 바꿔도 뒤의 RC카가 '인식'되지는 않는다. 쓸모는 (a) LOP/예측 위치까지 장애물을 피해 이동, (b) 코스트맵 기반 충돌 회피.
  도입 비용: 로컬라이제이션(지도 + AMCL 또는 odom 전용 롤링 코스트맵), 노트북에서 실행 (로봇 설치 불필요).

## 8. 센서 QoS 와 압축 전송 — S13

**[R8] ROS 2 Jazzy "Quality of Service settings".** https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
**[R9] image_transport / compressed_depth_image_transport (Jazzy).** https://docs.ros.org/en/jazzy/p/image_transport/ , https://docs.ros.org/en/jazzy/p/compressed_depth_image_transport/

- [R8]: 센서 데이터 프로필 = best-effort + 작은 큐 — "모든 샘플보다 최신 샘플을 빨리" (S12 와 같은 근거).
- [R9]: 영상은 `compressed`(jpeg), depth 는 `compressedDepth`(PNG 무손실, Jazzy 부터 RVL 도) 하위 토픽으로 보낼 수 있다.
- 실측: raw 1.4 MB + 1.0 MB/프레임 → 로봇 쪽 적체로 RGB 15~20 s 지연. compressed 86 KB + 210 KB → 10 Hz, 0.05/0.13 s.
  depth 는 PNG 무손실이라 **거리 정밀도 손실 없음** (RGB jpeg 손실은 YOLO 신뢰도 0.97~0.99 로 영향 미미).

---

### 서지 확인 상태
- DOI 대조 확인: R1(두 판), R2, R3, R4, R6, R7, R10, R11, R12, R13, Koide & Miura.
- URL 확인: R5, R9, R14 (HTTP 200). R8 은 봇 차단으로 검색 요약으로 내용 확인.
- 미확인: Tukey, *Exploratory Data Analysis* (1977) 출판 정보 — 인용하지 않음.
- 논문이 직접 말하지 않은 것(공학적 선택): `min_fresh=3`, `search_wait_s=0.7`, 잘림 판정 4 px, 지평선 = $c_y$ 근사.
