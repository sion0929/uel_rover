#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  개발PC 시각화 환경 최초 설정 (Ubuntu 24.04 + ROS2 Jazzy 전제)
#  1회만 실행. sudo 필요.
# ─────────────────────────────────────────────────────────────────
set -e

if [ "$EUID" -eq 0 ]; then
    echo "[오류] root로 직접 실행하지 말 것. sudo가 필요할 때 물어본다."
    exit 1
fi

echo "[1/2] Eclipse Zenoh apt 저장소 추가 + zenoh-bridge-ros2dds 설치"
if ! apt-cache policy zenoh-bridge-ros2dds 2>/dev/null | grep -q Candidate; then
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | \
        sudo tee /etc/apt/sources.list.d/zenoh.list
    sudo apt-get update
fi
sudo apt-get install -y zenoh-bridge-ros2dds

echo "[2/2] rviz2 Nav2 플러그인 설치"
sudo apt-get install -y ros-jazzy-nav2-rviz-plugins

cat <<'EOF'

[완료] 개발PC 시각화 환경 설정 완료.
실행: ./rviz.sh
EOF
