#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  systemd 유닛 3종 설치: 재부팅/크래시 시 자동 재시작
#  (기존 docker-compose.yml의 restart: unless-stopped 대체)
# ─────────────────────────────────────────────────────────────────
set -e

if [ "$EUID" -eq 0 ]; then
    echo "[오류] root로 직접 실행하지 말 것. sudo가 필요할 때 물어본다."
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS="uel-rover-zenoh-bridge.service uel-rover-micro-ros-agent.service uel-rover.service"

for unit in $UNITS; do
    echo "[install_systemd] $unit 설치"
    sed -e "s|__USER__|$USER|g" -e "s|__REPO_DIR__|$REPO_DIR|g" \
        "$REPO_DIR/systemd/$unit" | sudo tee "/etc/systemd/system/$unit" > /dev/null
done

sudo systemctl daemon-reload

for unit in $UNITS; do
    sudo systemctl enable "$unit"
done

cat <<EOF

[완료] systemd 유닛 설치 및 부팅 시 자동시작 활성화됨.
지금 바로 시작하려면:
  sudo systemctl start uel-rover-zenoh-bridge uel-rover-micro-ros-agent uel-rover
상태 확인:
  systemctl status uel-rover-zenoh-bridge uel-rover-micro-ros-agent uel-rover
로그 확인:
  journalctl -u uel-rover -f
EOF
