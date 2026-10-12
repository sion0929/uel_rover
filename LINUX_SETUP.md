# 리눅스 PC 설치/실행 (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17)

이 저장소 = colcon 워크스페이스 `ros2_ws` 의 소스입니다 (build/install/log 는 각자 빌드).

| 패키지 | 내용 |
|---|---|
| `src/spacek_rover` | 1단계: 다이나믹셀 스키드 조향 로버 (바퀴 4, DiffDrive, 주행 시험) |
| `src/drone_landing_platform` | 착륙 포획 플랫폼 + 단순 드론, 2단계: 정지 로버 위 플랫폼 (`rover:=true`) |
| `src/spacek_drone_sim` | 3a단계: PX4 로 나는 대회 기체 모델 + 플랫폼 마커 5개 + 실행 파일 |

## 1. 받기

```bash
git clone https://github.com/sion0929/uel_rover.git ~/ros2_ws   # -> ~/ros2_ws/src/...
```

## 2. 의존성 (처음 한 번)

ROS 2 Jazzy 설치: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html (`ros-jazzy-desktop ros-dev-tools`)

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard python3-yaml python3-pytest python3-opencv
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -y
```

## 3. 빌드 + 시험

```bash
cd ~/ros2_ws
colcon build --packages-select spacek_rover drone_landing_platform spacek_drone_sim
source install/setup.bash
colcon test --packages-select spacek_rover drone_landing_platform spacek_drone_sim && colcon test-result --verbose
```

알려진 실패 3건: 각 패키지 `test_pep257` 의 D213 (여러 줄 독스트링 요약 줄 위치, 동작 무관).

## 4. PX4 준비 (3a단계, 처음 한 번, 20~30분, 인터넷 필요)

> **바뀔 예정:** PX4 연동은 실기체 펌웨어 계열(PX4Swarm `v1.17/robot_aircraft`) + 기체 내부 DDS(XRCE) / 외부 Zenoh 로
> 다시 정리합니다. 아래 스크립트는 그 전의 순정 PX4 v1.17.0 기준입니다. 지금 검증된 것은 `px4:=false` 모드입니다.

```bash
bash ~/ros2_ws/install/spacek_drone_sim/share/spacek_drone_sim/scripts/setup_px4.sh
cd ~/ros2_ws && colcon build && source install/setup.bash     # 내려받은 px4_msgs 까지 빌드
```
PX4-Autopilot v1.17.0 (`~/PX4-Autopilot`), Micro XRCE-DDS Agent v2.4.3, `px4_msgs` (release/1.17) 를 준비합니다.

## 5. 실행

```bash
# 1단계: 로버 단독 (키보드 주행)
ros2 launch spacek_rover rover_sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rover/cmd_vel

# 1단계: 자동 주행 시험 (전진/제자리 선회/호 주행/정지) -> ~/rover_results
ros2 launch spacek_rover rover_sim.launch.py drive_test:=true gui:=false shutdown_on_done:=true

# 2단계: 정지 로버 위 플랫폼 + 단순 드론 착륙 (시나리오 center / x30 / x60 / yaw30)
ros2 launch drone_landing_platform rover_landing_sim.launch.py scenario:=center gui:=false \
    shutdown_on_done:=true csv_dir:=~/landing_results/rover

# 3a단계 (PX4 없이): 드론·로버·플랫폼 모델만 (드론은 지면에 놓여 있음, 로버는 키보드 주행 가능)
ros2 launch spacek_drone_sim px4_rover_landing.launch.py px4:=false

# 3a단계: PX4 드론 + 마커 플랫폼 + 로버
ros2 launch spacek_drone_sim px4_rover_landing.launch.py                         # 드론: 로버 뒤 지면
ros2 launch spacek_drone_sim px4_rover_landing.launch.py drone_start:=platform marker_probe:=true
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander takeoff                  # 다른 터미널
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander land
```

3a단계 확인 목록과 토픽·인자 설명: `src/spacek_drone_sim/README.md` (7절 = 합격 기준).
2단계 합격 기준: `~/landing_results/rover/<시나리오>_result.json` 의 `success: true` 와 `<시나리오>_rover_watch.json` 의 `passed: true`.

## 6. 문제가 생기면

* GUI 가 느리거나 검은 화면: `export LIBGL_ALWAYS_SOFTWARE=1` 또는 `gui:=false` (3a단계 카메라는 그래픽카드 필요)
* NVIDIA 드라이버 자동 업데이트 뒤 재부팅 전이면 `Driver/library version mismatch` 로 GUI·카메라가 죽습니다 → 재부팅
* 셸 기본 RMW 가 `rmw_zenoh_cpp` 이면 launch 가 Zenoh 라우터를 노드보다 먼저 띄웁니다. 라우터 없이 뜬 노드는 나중에 라우터를 켜도
  브리지가 명령을 못 받아 접이판이 서지 않습니다 (`plate_coupler` 가 10 s 뒤 오류 로그). PX4 `/fmu/...` 는 `rmw_fastrtps_cpp` 필요
* 다른 PC 의 gz 와 섞일 때: `export GZ_IP=127.0.0.1`
* `PX4 SITL 빌드를 찾을 수 없습니다` → 4절 스크립트를 실행했는지, `px4_dir:=<경로>` 확인
* 마커가 안 읽힘 (`marker_probe`) → `src/spacek_drone_sim/config/markers.yaml` 의 `flip_v: true`
* 실행 확인 (2026-10-12, RTX 3060 Ti): `px4:=false` 헤드리스·GUI, 카메라 2대 30 Hz, 로버 최고 0.459 m/s. PX4 비행은 아직 미검증.
