# Docker 전면 제거 — 젯슨/개발PC 모두 네이티브 실행으로 전환

**작성일:** 2026-08-06
**대상:** 저장소 루트(젯슨 배포 스택), `rviz/`(개발PC 시각화)

## 1. 배경 / 문제

현재 두 곳이 Docker로 격리되어 있다.

1. **루트** — `Dockerfile` + `docker-compose.yml`: 젯슨 Orin에서 `zenoh_bridge`,
   `micro_ros_agent`, `uel_rover`(odom_publisher + Nav2 + SLAM + RealSense) 3개
   컨테이너를 띄운다.
2. **`rviz/`** — `Dockerfile.rviz` + `docker-compose.rviz.yml`: 개발PC에서
   `zenoh_bridge`(개발PC측) + `rviz2` 2개 컨테이너를 띄운다.

젯슨(JetPack 6.x)과 개발PC 모두 실제로는 **Ubuntu 24.04**이고 ROS2 배포판(jazzy)도
동일하다. 배포 대상 아키텍처가 다양하지 않고(둘 다 네이티브 Ubuntu 24.04), 컨테이너가
주던 격리 이점(다른 배포판/버전 혼재 방지) 대비 다음 비용이 더 크다고 판단해 제거한다.

- 이미지 빌드/유지보수 비용
- `--privileged` + `ipc: host`로 컨테이너 경계를 사실상 무력화해야만 RealSense USB와
  fastdds SHM이 동작하는 상태(`docker-compose.yml`의 `ipc: host` 주석 참고) — 격리 이점이
  실질적으로 이미 없음
- 컨테이너 재시작 정책(`restart: unless-stopped`)을 systemd로 대체해도 기능 손실 없음

## 2. 변경 사항

### 2.1 삭제할 파일
- `Dockerfile`, `docker-compose.yml`, `build_docker.sh`, `run_docker.sh`,
  `entrypoint.sh`, `.dockerignore`
- `rviz/Dockerfile.rviz`, `rviz/docker-compose.rviz.yml`

### 2.2 젯슨 — 신규 파일

**`setup.sh`** (최초 1회 실행, sudo 필요)
- `Dockerfile`이 설치하던 apt 패키지 전부 설치: 빌드 도구
  (`python3-colcon-common-extensions`, `python3-rosdep`, `cmake`, `build-essential` 등),
  ROS2 패키지(`ros-jazzy-tf2-ros`, `ros-jazzy-nav-msgs`, `ros-jazzy-robot-state-publisher`,
  `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`,
  `ros-jazzy-realsense2-camera`, `ros-jazzy-pointcloud-to-laserscan`,
  `ros-jazzy-nav2-amcl`, `ros-jazzy-nav2-map-server`)
- Eclipse Zenoh 공식 apt 저장소(`download.eclipse.org/zenoh/debian-repo`) 추가 +
  `zenoh-bridge-ros2dds` 설치
- **micro-ROS agent 소스 빌드**: apt 패키지가 없으므로 `micro_ros_setup`(jazzy 브랜치)을
  별도 워크스페이스 `microros_ws/`에 클론해 `create_agent_ws.sh` + `build_agent.sh`로
  빌드. 공식 `microros/micro-ros-agent:jazzy` 이미지도 내부적으로 동일 과정을 거친다.
- `rosdep install --from-paths src --ignore-src -r -y` (`uel_rover_control` 의존성)
- 최초 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 현재 사용자를 `dialout` 그룹에 추가(시리얼 `/dev/ttyACM0` 접근). RealSense udev 규칙은
  `ros-jazzy-realsense2-camera` 설치 시 함께 등록됨(컨테이너의 `privileged: true`가
  우회하던 것을 네이티브에선 정식 udev 규칙으로 대체)

**`run.sh`** (개발/디버그용 수동 포그라운드 실행, systemd 설치 없이 즉시 반복 테스트)
- `zenoh-bridge-ros2dds -l tcp/0.0.0.0:7447` 백그라운드
- `microros_ws/install/.../micro_ros_agent serial --dev /dev/ttyACM0 -b 115200`
  (env: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `MICROROS_DISABLE_SHM=1`) 백그라운드
- `ros2 launch uel_rover_control mapping.launch.py` 포그라운드 (env: `ROS_DOMAIN_ID=0`)
- `trap`으로 스크립트 종료(Ctrl+C) 시 백그라운드 프로세스 정리

**`systemd/uel-rover-zenoh-bridge.service`**
**`systemd/uel-rover-micro-ros-agent.service`**
**`systemd/uel-rover.service`**
- `docker-compose.yml`의 `restart: unless-stopped` 3개를 대체. 모두 `Restart=always`.
- `uel-rover.service`는 `After=`/`Requires=`로 앞의 두 서비스 뒤에 시작하도록 순서 보장
  (기존 `depends_on`과 동일한 의도)
- `--privileged`/`ipc: host`는 네이티브에선 대상이 없어 항목 자체가 사라짐(컨테이너 경계가
  없으므로 SHM 격리 문제가 애초에 발생하지 않음)

**`install_systemd.sh`** — 위 3개 unit 파일을 `/etc/systemd/system/`에 복사, 현재 사용자로
`User=`/`WorkingDirectory=` 치환, `systemctl daemon-reload` + `enable`. sudo 필요.

### 2.3 개발PC — `rviz/` (앞서 승인된 설계 그대로)
- **`rviz/setup.sh`** (신규, 최초 1회): Eclipse Zenoh apt 저장소 추가 + `zenoh-bridge-ros2dds`,
  `ros-jazzy-nav2-rviz-plugins` 설치
- **`rviz/rviz.sh`** (기존 파일 재작성): `xhost +local:docker` 제거. `zenoh-bridge-ros2dds -e
  tcp/192.168.0.112:7447` 백그라운드 → `rviz2 -d uel_rover.rviz` 포그라운드, `trap`으로
  종료 시 bridge 정리. `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 명시 지정 — 개발PC 호스트의
  기본 RMW가 `rmw_zenoh_cpp`로 설정되어 있음을 확인함(이 세션에서 `printenv
  RMW_IMPLEMENTATION` 확인). 명시하지 않으면 rviz2가 로컬 DDS 그래프 대신 자체 Zenoh
  세션으로 붙어 zenoh_bridge가 재주입한 토픽을 못 볼 수 있음 — 기존 Docker 이미지가
  (의도치 않게) 격리해주던 부분이라 네이티브 전환 시 명시적으로 고정해야 함

### 2.4 문서
- **`docs/system-overview.md`**: 물리 구성 다이어그램의 "· 전부 Docker, headless"를
  네이티브 서술(systemd 3종)로 갱신

## 3. 영향 없음

- ROS2 노드 구성, 토픽, TF 트리, launch 파일 로직: 변경 없음. 오직 "무엇이 노드를
  실행/재시작하는가"만 컨테이너→systemd/스크립트로 바뀐다.
- `ROS_DOMAIN_ID=0`, RMW 선택(젯슨 쪽은 그래프 전체 fastrtps 유지), zenoh 브릿지 주소·포트:
  기존 docker-compose 값 그대로 유지

## 4. 검증 체크리스트

이 세션 환경(샌드박스)에서 확인 가능한 것:
1. 각 스크립트(`setup.sh`, `run.sh`, `install_systemd.sh`, `rviz/setup.sh`, `rviz/rviz.sh`)
   문법 오류 없음 (`bash -n`)
2. systemd unit 파일 문법 오류 없음 (`systemd-analyze verify` 또는 수동 검토)
3. `download.eclipse.org` apt 저장소 접근 가능(이미 확인됨: HTTP 200)
4. `colcon build`로 `uel_rover_control` 정상 빌드(이미 확인됨, Docker와 무관하게 항상
   통과해야 함)
5. rviz2 자체 기동 확인(이 머신에 이미 `rviz2`/`nav2-rviz-plugins` 설치되어 있어 가능)

사용자가 OpenRB-150(USB 시리얼)과 RealSense D455를 이 컴퓨터에 연결해주기로 해서 추가로
다음도 실기 검증 가능:
6. `setup.sh`의 micro-ROS agent 소스 빌드가 실제로 성공하고 실행 파일이 생성되는지
7. `micro_ros_agent serial --dev /dev/ttyACM0 -b 115200`이 실제 MCU와 핸드셰이크하고
   `/wheel_velocity`, `/drive_mode` 토픽이 올라오는지
8. `realsense2_camera_node`가 실제 D455를 인식해 `/camera/camera/depth/color/points`를
   발행하는지, `pointcloud_to_laserscan`을 거쳐 `/scan`까지 이어지는지
9. `install_systemd.sh`로 설치한 3개 서비스가 `systemctl start`로 정상 기동하고,
   `systemctl status`에서 `active (running)`인지
10. 강제로 프로세스를 죽였을 때(`kill`) systemd가 `Restart=always`로 재기동하는지

검증 불가(이 환경에 없는 것): 젯슨 실기 자체, 무선 구간(Zenoh) 왕복 풀 테스트, 재부팅 후
자동시작(원격 젯슨 재부팅은 이 세션 범위 밖) — 이 부분은 실제 배포 시 사용자가 직접 확인.

## 5. 범위 밖

- 젯슨/개발PC 외 제3의 배포 대상(예: 다른 아키텍처)에 대한 크로스 컴파일 지원은 다루지
  않는다. Dockerfile 상단의 "Jetson Orin용 buildx --platform linux/arm64" 코멘트가 있었지만,
  네이티브 전환 후에는 각 머신에서 직접 `setup.sh`를 실행하는 것으로 대체되어 크로스빌드
  자체가 불필요해진다.
- mapping/localization 모드 전환 방식(현재 `uel-rover.service`가 `mapping.launch.py`를
  하드코딩, 기존 docker-compose와 동일 동작 유지)은 이번 스펙에서 개선하지 않는다.
