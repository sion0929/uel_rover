# spacek 가제보 시뮬레이션

드론(PX4) + 스키드 조향 로버 + 로버 위 착륙 플랫폼을 Gazebo Harmonic 에서 돌리는 ROS 2 Jazzy 워크스페이스.
드론과 로버의 하드웨어 값은 실기체에 맞춰 두었다 (아래 "하드웨어 값의 원본").

설치·빌드·실행: [LINUX_SETUP.md](LINUX_SETUP.md)

## 패키지

| 패키지 | 내용 | 자세히 |
|---|---|---|
| `src/spacek_rover` | 스키드 조향 로버 (바퀴 4, DiffDrive, 자동 주행 시험) | [README](src/spacek_rover/README.md) |
| `src/drone_landing_platform` | 착륙 포획 플랫폼 (접이판 4, 마커), 정지 로버 위 착륙 시나리오 | [README](src/drone_landing_platform/README.md) |
| `src/spacek_drone_sim` | 대회 드론 (Pixhawk 6X / PX4, H-Flow, 카메라 2대) + 마커 플랫폼 + 로버 통합 실행 | [README](src/spacek_drone_sim/README.md) |

## 빠른 시작 (PX4 없이)

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build && source install/setup.bash
ros2 launch spacek_drone_sim px4_rover_landing.launch.py px4:=false     # 드론·로버·플랫폼, 카메라 토픽
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rover/cmd_vel
```

주요 토픽: `/drone/front_camera/image_raw`, `/drone/down_camera/image_raw` (+ `camera_info`), `/rover/cmd_vel`, `/rover/odom`.

## 하드웨어 값의 원본

값은 설정 파일 한 곳에만 있고, 모델 SDF 와 PX4 기체 설정은 여기서 만들어진다. 바꿀 때는 이 파일만 고친다.

| 파일 | 담긴 것 | 실기체 근거 |
|---|---|---|
| `src/spacek_drone_sim/config/drone.yaml` | 질량 1.9 kg, 형상(CAD), 7인치 3엽 프롭, 호버 출력 0.35, 센서 위치·화각, H-Flow 0.08~30 m, `px4_params` | 09-21 비행 로그(PX4 파라미터·호버 추력), 카메라 실측 초점거리 |
| `src/spacek_rover/config/rover.yaml` | 바퀴 반지름 0.0877 m, 바퀴 최대 50 RPM (최고 0.459 m/s) | 1 m 직진 오도메트리, 펌웨어 MAX_RPM |
| `src/drone_landing_platform/config/*.yaml` | 플랫폼 치수, 접이판, 마커 | CAD |

전방 카메라는 60° 케이스(수평에서 아래로 60°)를 배터리 케이스 앞면에 끼운 배치다.

## 현재 상태와 한계

- 검증됨: `px4:=false` (모델 배치, 로버 주행, 카메라 2대 30 Hz, 거리센서). 단위 시험 205개 (실패 3 = 기존 D213 독스트링 경고).
- 다음 단계: PX4 v1.17 (실기체 계열 PX4Swarm) 연동 — 기체 내부 DDS(XRCE), 외부 Zenoh. 그 뒤 비행 시험으로 `px4_params` 검증.
- 최대 추력/무게 5.0 은 추정값 (모터·프롭 사양으로 교체 예정).
- 카메라 렌즈 왜곡은 렌더링되지 않는다 (가제보 ogre2 한계). 시뮬 `camera_info` 의 왜곡 계수는 0 이다.
