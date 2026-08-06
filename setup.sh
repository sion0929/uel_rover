#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  UEL Rover 네이티브 실행 환경 최초 설정 (Jetson Orin, Ubuntu 24.04)
#  1회만 실행. sudo 필요.
# ─────────────────────────────────────────────────────────────────
set -e

if [ "$EUID" -eq 0 ]; then
    echo "[오류] root로 직접 실행하지 말 것. 일반 사용자로 실행하면 필요할 때 sudo를 물어본다."
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MICROROS_WS="$REPO_DIR/microros_ws"

echo "[1/7] apt 패키지 설치"
sudo apt-get update
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    cmake \
    build-essential \
    git \
    wget \
    curl \
    udev \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2 \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-realsense2-camera \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-map-server

echo "[2/7] Eclipse Zenoh apt 저장소 추가 + zenoh-bridge-ros2dds 설치"
if ! apt-cache policy zenoh-bridge-ros2dds 2>/dev/null | grep -q Candidate; then
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | \
        sudo tee /etc/apt/sources.list.d/zenoh.list
    sudo apt-get update
fi
sudo apt-get install -y zenoh-bridge-ros2dds

echo "[3/7] rosdep init/update"
sudo rosdep init 2>/dev/null || true
rosdep update

echo "[4/7] uel_rover_control 의존성 설치"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "$REPO_DIR/src" --ignore-src -r -y

echo "[5/7] micro-ROS agent 소스 빌드 (${MICROROS_WS})"
if [ ! -x "$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" ]; then
    mkdir -p "$MICROROS_WS/src"
    if [ ! -d "$MICROROS_WS/src/micro_ros_setup" ]; then
        git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git \
            "$MICROROS_WS/src/micro_ros_setup"
    fi
    (
        cd "$MICROROS_WS"
        source /opt/ros/jazzy/setup.bash
        rosdep update
        rosdep install --from-paths src --ignore-src -y
        colcon build
        source install/local_setup.bash
        ros2 run micro_ros_setup create_agent_ws.sh
        ros2 run micro_ros_setup build_agent.sh
    )
else
    echo "  이미 빌드됨, 건너뜀"
fi

echo "[6/7] uel_rover_control colcon build"
(
    cd "$REPO_DIR"
    source /opt/ros/jazzy/setup.bash
    colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
)

echo "[7/7] 시리얼 포트 접근을 위해 현재 사용자를 dialout 그룹에 추가"
sudo usermod -aG dialout "$USER"

cat <<'EOF'

[완료] 네이티브 설정 완료.
- dialout 그룹 추가는 재로그인(또는 재부팅) 후 적용됨. 지금 셸에서 바로 시리얼
  접근이 필요하면: newgrp dialout
- 수동/포그라운드 실행: ./run.sh
- 부팅 시 자동 실행(systemd)으로 설치: ./install_systemd.sh
EOF
