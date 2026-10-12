#!/usr/bin/env bash
# PX4 SITL 1회 준비 (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic) - spacek_drone_sim 3a단계
#   1) PX4-Autopilot v1.17.0 내려받기 + 의존성(Tools/setup/ubuntu.sh) + make px4_sitl
#   2) Micro XRCE-DDS Agent v2.4.3 설치 (PX4 <-> ROS 2)
#   3) px4_msgs (release/1.17) 를 워크스페이스 src 에 내려받기
# 기체 설정 파일(4901_gz_spacek_drone)은 실행할 때마다 launch 가 drone.yaml 로 새로 써 넣는다.
#
# 사용:  bash setup_px4.sh                              (PX4_DIR=~/PX4-Autopilot, WS_DIR=~/ros2_ws)
#        PX4_DIR=/opt/PX4-Autopilot WS_DIR=~/ws bash setup_px4.sh
set -euo pipefail

PX4_TAG="v1.17.0"
PX4_MSGS_BRANCH="release/1.17"
XRCE_TAG="v2.4.3"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"

echo "[1/3] PX4-Autopilot ${PX4_TAG} -> ${PX4_DIR}"
if [ ! -d "${PX4_DIR}/.git" ]; then
  git clone --recursive --branch "${PX4_TAG}" https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  have="$(git -C "${PX4_DIR}" describe --tags --exact-match 2>/dev/null || echo unknown)"
  if [ "${have}" != "${PX4_TAG}" ]; then
    echo "오류: ${PX4_DIR} 의 PX4 버전이 ${have} 입니다 (필요: ${PX4_TAG}, px4_msgs ${PX4_MSGS_BRANCH} 와 짝)." >&2
    echo "  다른 폴더를 쓰거나 (PX4_DIR=...), 그 폴더에서" >&2
    echo "  git checkout ${PX4_TAG} && git submodule update --init --recursive 후 다시 실행하세요." >&2
    exit 1
  fi
fi
bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx
(cd "${PX4_DIR}" && make px4_sitl)
test -x "${PX4_DIR}/build/px4_sitl_default/bin/px4"

echo "[2/3] Micro XRCE-DDS Agent ${XRCE_TAG}"
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  tmp="$(mktemp -d)"
  git clone --branch "${XRCE_TAG}" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${tmp}/agent"
  cmake -S "${tmp}/agent" -B "${tmp}/agent/build"
  cmake --build "${tmp}/agent/build" -j"$(nproc)"
  sudo cmake --install "${tmp}/agent/build"
  sudo ldconfig /usr/local/lib/
fi

echo "[3/3] px4_msgs (${PX4_MSGS_BRANCH}) -> ${WS_DIR}/src/px4_msgs"
mkdir -p "${WS_DIR}/src"
if [ ! -d "${WS_DIR}/src/px4_msgs" ]; then
  git clone --branch "${PX4_MSGS_BRANCH}" https://github.com/PX4/px4_msgs.git "${WS_DIR}/src/px4_msgs"
fi

echo "완료. 다음: cd ${WS_DIR} && colcon build && source install/setup.bash"
echo "(ubuntu.sh 가 사용자 그룹을 바꿨으면 한 번 로그아웃 후 다시 로그인)"
