# ═══════════════════════════════════════════════════════════════
#  UEL Rover Docker 이미지
#
#  개발 환경 : Ubuntu 24.04 (x86_64)
#  배포 환경 : Jetson Orin (ARM64, JetPack 6.x, Ubuntu 24.04)
#
#  빌드 명령:
#    x86_64 개발용:  docker build -t uel_rover:jazzy .
#    Jetson Orin용:  docker buildx build --platform linux/arm64 \
#                     -t uel_rover:jazzy-arm64 .
# ═══════════════════════════════════════════════════════════════
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ── 시스템 의존성 설치 ───────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # 빌드 도구
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    cmake \
    build-essential \
    git \
    wget \
    # ROS2 패키지
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2 \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-robot-state-publisher \
    # Nav2 + SLAM
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    # RealSense D455 + 스캔 변환
    ros-jazzy-realsense2-camera \
    ros-jazzy-pointcloud-to-laserscan \
    # 위치추정(AMCL/map_server)
    #   ※ RMW는 기본 DDS 유지. 젯슨↔개발PC는 zenoh-bridge-ros2dds(별도
    #     컨테이너)로 브릿지하므로 이미지에 rmw_zenoh_cpp 불필요.
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-map-server \
    # 유틸
    udev \
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# ── 워크스페이스 설정 ────────────────────────────────────────────
WORKDIR /ros2_ws

COPY src/ src/

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── 환경 설정 ────────────────────────────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash"       >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash"      >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0"                  >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
