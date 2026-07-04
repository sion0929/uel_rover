#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  Docker 컨테이너 실행 스크립트
# ─────────────────────────────────────────────────────────────────

IMAGE="uel_rover:jazzy"

# Jetson Orin이면 arm64 이미지 사용
if [ "$(uname -m)" = "aarch64" ]; then
    IMAGE="uel_rover:jazzy-arm64"
fi

docker run -it --rm \
    --name uel_rover \
    --network host \
    --privileged \
    -e ROS_DOMAIN_ID=0 \
    -v "$(pwd)/src:/ros2_ws/src" \
    --device /dev/ttyACM0:/dev/ttyACM0 \
    --device /dev/ttyTHS1:/dev/ttyTHS1 \
    "$IMAGE" \
    bash
