@echo off
chcp 65001 >nul
REM ==== 경로만 바꿔서 쓰세요 (한글/공백 없는 폴더 권장) ====
set MODEL=my_best.pt
set OLD_DATA=my_data
set VIDEO1=S1.mp4
set VIDEO2=S2.mp4

call venv\Scripts\activate

echo [1] 프레임 추출
python extract_frames.py %VIDEO1% frames\S1 --fps 0.5 || goto :err
python extract_frames.py %VIDEO2% frames\S2 --fps 0.5 || goto :err

echo [2] 미리 라벨링
python prelabel.py --model %MODEL% --images frames\S1 || goto :err
python prelabel.py --model %MODEL% --images frames\S2 || goto :err

echo.
echo [3] 사람 검토: frames\S1_preview, frames\S2_preview 에서 이름 앞 ! 인 사진을 라벨 도구로 고친 뒤 아무 키나 누르세요
pause

echo [4] 세션 단위 분할 (S1+기존 = 학습, S2 = 평가)
python make_split.py --out dataset --names car dummy --train frames\S1 --test frames\S2 --add-old %OLD_DATA% || goto :err

echo [5] 재학습은 GPU 에서 (Colab 권장):  python train.py --model %MODEL% --data dataset\data.yaml --epochs 30
echo     끝나면 best.pt 경로를 아래 NEW 에 넣고 [6] 만 다시 실행
set NEW=runs\detect\finetune\weights\best.pt
if not exist %NEW% (echo 새 모델이 아직 없어 이전 모델만 평가합니다 & python judge.py --old %MODEL% --data dataset\test & goto :end)

echo [6] 판정
python judge.py --old %MODEL% --new %NEW% --data dataset\test || goto :err
goto :end

:err
echo 오류가 났습니다. 위 메시지와 README.md 의 '자주 겪는 문제'를 확인하세요.
:end
pause
