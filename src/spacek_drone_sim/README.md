# spacek_drone_sim

대회 기체(Pixhawk 6X + PX4)를 **PX4 SITL** 로 나는 가제보 모델 `spacek_drone` 과 로버 플랫폼 마커 5개 (3a단계).
명령 하나로 가제보 + 로버(플랫폼 장착) + 드론 + PX4 + ROS 2 연결이 뜬다.
설계: `ros2_ws/docs/superpowers/specs/2026-10-09-px4-drone-testbed-design.md`

## 1. 무엇이 있나

| 파일 | 내용 |
|---|---|
| `config/drone.yaml` | 드론의 **단일 원본**: 질량 1.9 kg, 모터 배치·상수, 센서 사양과 장착 위치 |
| `models/spacek_drone/` | 드론 SDF (3엽 7인치 프롭 4, IMU·기압계·지자기 (+GPS 센서는 있으나 꺼 둠), H-Flow 광류+거리, 전방 60° 카메라, 하방 카메라) |
| `config/markers.yaml` | 플랫폼 마커 (기본 AprilTag 36h11: 바닥 0번 160 mm, 접이판 1~4번 80 mm) |
| `worlds/rover_landing_px4.sdf` | 야외 월드 (지면 z = 0, GPS 기준 위치) |
| `launch/px4_rover_landing.launch.py` | 전체 실행 |
| `px4/4901_gz_spacek_drone` | PX4 기체 설정 (참고용; 실행 때 drone.yaml 로 새로 만들어 PX4 에 설치) |
| `scripts/setup_px4.sh` | 리눅스 1회 준비 (PX4 v1.17.0, Micro XRCE-DDS Agent v2.4.3, px4_msgs release/1.17) |
| `marker_probe` (노드) | 카메라 영상에서 마커 번호를 1 Hz 로 출력 (마커가 보이는지 확인용) |

드론 좌표: 원점 = 기체 중심, **+X 앞**, Y 왼쪽, Z 위, 발 바닥 z = −0.1644 m.
CAD 확인: `로버플랫폼프로토타입/로버_가제보/drone_gazebo.SLDASM` (하부 프레임 6S5200 + 파트1 + 60° 케이스).

| 센서 | 위치 (m, 드론 좌표) | 방향 |
|---|---|---|
| 전방 카메라 (See3CAM_24CUG, 60° 케이스) | (0.110, 0, −0.074) — 앞 양 다리 사이 가운데, T자 클립을 배터리 케이스 앞면에 끼움 | 앞, 수평 아래 60° |
| 하방 카메라 | (0, 0, −0.120) — 가운데 클립 블록 밑 | 아래 |
| H-Flow 광류 / ToF | (−0.056, 0 / 0.008, −0.105) — 배터리 케이스 밑 H-Flow 자리 | 아래 |
| IMU·기압계·지자기 (GPS 센서는 PX4 에서 꺼 둠) | 기체 중심 | — |

카메라 = See3CAM_24CUG 2대, 실기 출력과 같은 1280×720 rgb8 30 fps. 가로 화각은 실측 초점거리로 유도한다
(전방 f 727 px → 82.7°, 하방 f 594 px → 94.3°; AVISDrone `cam_*_info.yaml` 2026-08-28).

실기체에 맞춘 값 (2026-10-12, 출처: `~/비행테스트 로그` 09-21 ulog, AVISDrone 설정):

| 항목 | 값 | 근거 |
|---|---|---|
| 호버 출력 | 0.35 | 실비행 PX4 `hover_thrust_estimate` 0.347~0.354 |
| 최대 추력 / 무게 | 5.0 (추정) | 저출력 구간 로그 + 7인치 6S 일반값 — 모터·프롭 사양으로 교체할 것 |
| 프롭 | 7인치 3엽 | 사용자 |
| 위치 추정 | GPS 없음, 광류 + 거리센서, 높이 기준 = 거리센서 | 실기 EKF2 설정 |
| 거리센서 범위 | 0.08 ~ 30 m | 실기 `UAVCAN_RNG_MIN/MAX` |
| 자세 PID | rate P 0.14 / I 0.3 / D 0.004 (roll·pitch), 자이로 필터 30 Hz | 실기 튜닝값 |
| PX4 파라미터 목록 | `config/drone.yaml` 의 `px4_params` | 로그의 기본값과 다른 값 중 시뮬에 의미 있는 것 |

## 2. 1회 준비 (리눅스 PC)

```bash
cd ~/ros2_ws && colcon build --packages-select spacek_rover drone_landing_platform spacek_drone_sim
source install/setup.bash
bash install/spacek_drone_sim/share/spacek_drone_sim/scripts/setup_px4.sh   # PX4 빌드 20~30분
cd ~/ros2_ws && colcon build && source install/setup.bash                    # px4_msgs 포함 다시 빌드
```
`PX4_DIR`, `WS_DIR` 환경변수로 위치를 바꿀 수 있다 (기본 `~/PX4-Autopilot`, `~/ros2_ws`). 인터넷 필요.

## 3. 실행

```bash
ros2 launch spacek_drone_sim px4_rover_landing.launch.py                       # 드론: 로버 뒤 1.5 m 지면
ros2 launch spacek_drone_sim px4_rover_landing.launch.py drone_start:=platform  # 드론: 플랫폼 바닥판 위
ros2 launch spacek_drone_sim px4_rover_landing.launch.py px4:=false             # PX4 없이 모델만
```

셸 RMW 가 `rmw_zenoh_cpp` 이면 launch 가 Zenoh 라우터(`rmw_zenohd`)를 노드보다 먼저 띄운다 (이미 떠 있으면 그것을 씀).
PX4 XRCE-DDS(`/fmu/...`)를 ROS 2 에서 보려면 모든 터미널에서 `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

| 인자 | 기본 | 설명 |
|---|---|---|
| `gui` | true | false 이면 헤드리스 (카메라 렌더링은 계속, 그래픽카드 필요) |
| `px4` | true | false 이면 PX4 확인·기체 파일 설치·PX4·XRCE Agent 를 건너뜀. 드론은 지면에 놓여 있고 카메라 토픽은 나옴, 로버는 `/rover/cmd_vel` 로 주행 |
| `px4_dir` | `~/PX4-Autopilot` | `make px4_sitl` 이 끝난 PX4 폴더. 없으면 실행 전에 안내 메시지와 함께 멈춘다 |
| `px4_shell` | false | true 이면 PX4 콘솔을 xterm 창으로 띄움 (`sudo apt install xterm`) |
| `drone_start` | ground | `ground` / `platform` |
| `lat`, `lon`, `alt` | (빈칸) | GPS 기준 위치. 셋 다 주거나 셋 다 비울 것 (빈칸 = PX4 기본 위치) |
| `marker_probe` | false | true 이면 카메라별 검출된 마커 번호 출력 |
| `markers_config` | (빈칸) | 다른 마커 설정 파일 |
| `xrce_agent` | true | PX4 ↔ ROS 2 중계 (`MicroXRCEAgent udp4 -p 8888`) |
| `spawn_delay`, `px4_delay` | 3.0, 3.0 | gz 기동 후 로버 스폰 / 드론 스폰 후 PX4 시작 대기 [s] (드론은 로버 스폰이 끝난 뒤 스폰) |
| `visual` | auto | 로버·플랫폼 메시/상자 |

## 4. 비행 명령

다른 터미널에서 (PX4 콘솔 명령):
```bash
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander takeoff
~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander land
```
또는 `px4_shell:=true` 로 띄운 콘솔에서 `commander takeoff`, `commander land`. QGroundControl 도 UDP 14550 으로 자동 연결된다.
개발 코드는 ROS 2 `/fmu/in/...` (오프보드) 로 PX4 를 조종한다 (px4_msgs).

## 5. ROS 2 토픽·좌표

| 토픽 | 내용 |
|---|---|
| `/drone/front_camera/image_raw`, `/drone/front_camera/camera_info` | 전방 카메라 (frame_id `front_camera_optical`) |
| `/drone/down_camera/image_raw`, `/drone/down_camera/camera_info` | 하방 카메라 (frame_id `down_camera_optical`) |
| `/fmu/out/...`, `/fmu/in/...` | PX4 (XRCE-DDS, 실기와 같음) |
| `/tf_static` | `base_link -> front_camera_optical`, `base_link -> down_camera_optical` (광학 좌표: z 앞, x 오른쪽, y 아래) |
| `/rover/cmd_vel`, `/rover/odom`, `/rover/odom_gt`, `/joint_states`, `/platform/*/cmd_pos` | 로버·플랫폼 (2단계와 같음) |
| `/rover/tf` | 로버 `odom -> base_link` (2단계의 `/tf` 에서 옮김: `/tf` 의 `base_link` 는 드론 카메라 TF 의 부모) |

H-Flow(광류·거리)는 실기처럼 PX4 가 직접 받는다 (PX4 콘솔 `listener vehicle_optical_flow`, `listener distance_sensor`).

## 6. 값 바꾸기

- 드론: `config/drone.yaml` 만 고친다. 다음 실행 때 드론 SDF 와 PX4 기체 설정(모터 범위, 호버 추력, EKF 센서 위치)이 자동 반영된다.
  패키지 안의 템플릿·참고 파일도 맞추려면 (패키지 폴더에서):
  `python3 -m spacek_drone_sim.drone_sdf config/drone.yaml models/spacek_drone/model.sdf`,
  `python3 -m spacek_drone_sim.px4_airframe config/drone.yaml px4/4901_gz_spacek_drone`.
- 마커: `config/markers.yaml` (종류·번호·크기). 실행할 때 그림(PNG)·사각형(OBJ)을 만들어 플랫폼에 붙인다.
  `marker_probe:=true` 에서 마커가 하나도 안 읽히면 `flip_v: true` 로 바꿔 볼 것 (텍스처 상하 반전).

## 7. 리눅스 확인 목록 (3a 합격 기준)

| 항목 | 확인 방법 | 기준 |
|---|---|---|
| PX4 연결 | 실행 로그 / PX4 콘솔 | 센서 오류 없음, 광류·거리센서로 위치 추정 정상 (GPS 없음), 시동 가능 |
| 비행 | `px4-commander takeoff` → 정지비행 → `land` | 프로펠러 4개 회전, 2.5 m 이륙·정지비행 안정 (yaw 돌지 않음), 착륙 |
| 카메라 | `ros2 topic hz /drone/down_camera/image_raw`, `rqt_image_view` | 영상 나옴, 플랫폼 위에서 마커 5개 보임, 전방은 60° 아래를 봄 |
| 마커 | `marker_probe:=true` | 플랫폼 위 1~2 m 에서 하방 카메라가 0~4 번 검출 |
| 광류·거리 | PX4 콘솔 `listener vehicle_optical_flow`, `listener distance_sensor` | 값이 들어옴, 이륙 시 거리 증가 |
| 로버·플랫폼 | 가제보 화면 | 2단계처럼 서 있음, 접이판 대기 자세 |
| 플랫폼 위 착륙 | `drone_start:=platform` → takeoff → land | 바닥판에 닿으면 바닥이 내려가고 접이판이 접힘 |

## 8. 알려진 한계

1. 최대 추력/무게 5.0, 모터 상수(x500 값), 로터 관성은 추정값. 호버 출력 0.35 만 실측에 맞췄다.
2. 렌즈 왜곡·주점 오프셋 없음: 가제보 ogre2 렌더러에 왜곡 처리가 없다. 시뮬 `camera_info` 의 D = 0 이므로 실기 보정값(D ≠ 0)과 섞어 쓰지 말 것.
3. 프롭 하강풍·지면 효과·바람 없음. 프롭가드는 보이기만 하고 충돌하지 않는다.
4. 마커 그림 방향(거울상 여부)은 리눅스에서 `marker_probe` 로 처음 확인한다.
5. PX4 가 쓰는 가제보(OSRF Harmonic)와 ROS 쪽 가제보 버전이 섞이면 광류 플러그인 로드가 실패할 수 있다 → 실행 로그의 `OpticalFlowSystem` 오류 확인.
6. 리눅스 실행 확인은 `px4:=false` 까지 (2026-10-12). PX4 비행과 `px4_params` 값은 PX4 연동 후 검증한다.
7. `lat/lon/alt` 를 바꿔도 가제보 지자기는 월드에 고정된 자기장을 쓴다. PX4 가 그 위치의 편각(WMM)과 비교해
   나침반 경고를 낼 수 있다 → 문제가 되면 기본 위치(인자 비움)로 실행.
