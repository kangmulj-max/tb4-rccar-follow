| 모델 | 구조 | params | GFLOPs | 파일MB | valid mAP50-95 | test mAP50-95 | GPU ms | CPU ms | ONNX CPU ms | 시뮬 감지율@0.4 | 시뮬 평균 conf |
|---|---|---|---|---|---|---|---|---|---|---|---|
| yolo11n_amr_best | yolo11n.yaml | 2,590,230 | 6.5 | 5.48 | 0.971 | 0.905 | 8.9 | 63.3 | - | bright 46.9 / dim 62.5 | bright 0.41 / dim 0.543 |
| yolo12n_amr_best | yolov12n.yaml | 2,568,438 | 7.46 | 5.53 | 0.95 | 0.903 | 13.8 | 97.5 | - | bright 59.4 / dim 28.1 | bright 0.494 / dim 0.258 |
| yolo8n_amr_best | None | 3,011,238 | 8.19 | 6.26 | 0.977 | 0.917 | 7.6 | 68.1 | - | bright 21.9 / dim 31.2 | bright 0.198 / dim 0.285 |
