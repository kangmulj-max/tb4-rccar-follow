#!/bin/bash
# 텔레그램으로 메시지 보내기 (실험 완료·오류 알림용)
#   bash scripts/notify.sh "매트릭스 완료: 추종 유지율 v3safe/12n 80%"
# 설정 파일 ~/.config/tb4_notify.env (저장소에 포함하지 않음, 권한 600)
#   TG_TOKEN=123456:ABC...   ← @BotFather 에서 만든 봇 토큰
#   TG_CHAT=987654321        ← 비워두면 봇에게 보낸 첫 메시지에서 자동으로 찾아 저장
CFG=~/.config/tb4_notify.env
[ -f "$CFG" ] || { echo "설정 없음: $CFG"; exit 1; }
source "$CFG"
[ -n "${TG_TOKEN:-}" ] || { echo "TG_TOKEN 없음"; exit 1; }
API="https://api.telegram.org/bot${TG_TOKEN}"
if [ -z "${TG_CHAT:-}" ]; then        # 봇에게 먼저 아무 메시지나 보내 두면 그 대화 id 를 찾음
  TG_CHAT=$(curl -s "$API/getUpdates" | python3 -c "import sys,json; r=json.load(sys.stdin).get('result',[]); print(r[-1]['message']['chat']['id'] if r else '')")
  [ -n "$TG_CHAT" ] || { echo "봇에게 먼저 메시지를 보내세요 (텔레그램에서 봇 열고 /start)"; exit 1; }
  echo "TG_CHAT=$TG_CHAT" >> "$CFG"
fi
MSG=${1:-"(빈 메시지)"}
curl -s -o /dev/null -w "%{http_code}\n" "$API/sendMessage" --data-urlencode "chat_id=$TG_CHAT" \
     --data-urlencode "text=$MSG" --data-urlencode "disable_web_page_preview=true"
