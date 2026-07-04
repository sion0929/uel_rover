# UEL Rover — rviz2 + SLAM 목표점 자율주행 설계

**작성일:** 2026-07-04
**대상:** `uel_rover_control` 패키지 (Jetson Orin 배포 + 개발 PC 시각화)

## 1. 목표

RealSense D455로 SLAM 맵을 생성하고, 개발 PC의 rviz2에서 목표점을 클릭하면
로버가 그 지점으로 자율주행하게 한다. 조종기(FS-iA10B) RC 스위치의
RC/AUTO/STOP 3모드를 안전 스위치로 활용한다.

핵심 요구사항:
- rviz2는 **개발 PC(Ubuntu)** 에서 원격 실행 → 젯슨은 headless 유지
- SLAM 맵을 **매핑(생성/저장)** 과 **위치추정 주행(저장맵+AMCL)** 두 모드로 운용
- **RC 수동 주행 중에도 맵 생성** 가능
- 통신 미들웨어를 기본 DDS → **Zenoh (rmw_zenoh_cpp)** 로 전환
- rviz2는 드라이브 모드와 **독립적으로** 사용자가 상시 실행

## 2. 아키텍처

```
┌─────────────────── 젯슨 오린 (headless, Docker) ───────────────────┐
│  zenoh_router (rmw_zenohd, :7447)                                   │
│  micro_ros_agent ── OpenRB-150 (바퀴 RPM/명령, 시리얼)              │
│  odom_publisher, mode_manager                                       │
│  RealSense D455 → pointcloud_to_laserscan → /scan                   │
│                                                                     │
│  [모드① 매핑]   slam_toolbox      → /map + map→odom TF              │
│  [모드② 주행]   map_server+AMCL   → 저장맵 + map→odom TF            │
│  Nav2 (controller/planner/behavior/bt/waypoint) + lifecycle_manager │
└──────────────────── Zenoh (RMW=rmw_zenoh_cpp, host net) ───────────┘
                                   ↕  같은 LAN (192.168.0.x)
┌─────────────── 개발 PC (Ubuntu, Docker jazzy-desktop) ─────────────┐
│  rviz2 ── 맵/스캔/로봇모델 시각화 + "2D Goal Pose" 클릭 → /goal_pose │
│         (드라이브 모드와 무관하게 상시 실행)                         │
└────────────────────────────────────────────────────────────────────┘
```

- **역할 분리:** 젯슨 = 센서·SLAM·Nav2 연산 / 개발 PC = 화면·목표점 지정
- **통신:** 양쪽 모두 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, 개발 PC는 젯슨의
  Zenoh 라우터(`tcp/192.168.0.112:7447`)에 client로 접속
- **모드 전환:** 젯슨에서 실행하는 launch 파일만 교체 (매핑 ↔ 주행)

## 3. 젯슨쪽 구성 (접근 A — launch 분리)

### 3.1 launch 파일 구조

```
launch/
 ├─ bringup_common.launch.py   ← 공통: rover(odom/mode), robot_state_publisher,
 │                                카메라, pointcloud_to_laserscan,
 │                                Nav2 서버 5개, lifecycle_manager
 ├─ mapping.launch.py          ← common + slam_toolbox(온라인 async 매핑)
 └─ localization.launch.py     ← common + map_server + AMCL (map:= 인자로 저장맵)
```

- 기존 `nav2_bringup.launch.py`의 내용을 `bringup_common.launch.py`로 이전
- SLAM 부분만 각 모드 파일로 분리
- `lifecycle_manager`의 `node_names`를 모드별로 구성
  (매핑: `slam_toolbox` / 주행: `map_server`, `amcl` 추가)
- `nav2_bringup.launch.py`는 제거하거나 `mapping.launch.py`로의 별칭 처리

### 3.2 config 변경

- `nav2_params.yaml`에 **`amcl:` + `map_server:` 파라미터 블록 추가** (현재 없음)
- `global_costmap`에 `static_layer` 활성화 (주행 모드에서 저장맵 사용)
- `slam_toolbox.yaml`은 매핑 모드에서만 사용 (변경 없음)

### 3.3 맵 저장/보존

- 매핑 주행 후:
  `ros2 run nav2_map_server map_saver_cli -f /maps/<name>` → `.yaml` + `.pgm`
- `docker-compose.yml`에 `/maps` 볼륨 마운트로 컨테이너 밖 영구 보존

### 3.4 Zenoh 전환

- `Dockerfile`에 `ros-jazzy-rmw-zenoh-cpp` 설치 추가
- 모든 서비스 환경변수: `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- `docker-compose.yml`에 **`zenoh_router` 서비스 신설**
  (`ros2 run rmw_zenoh_cpp rmw_zenohd`), 포트 `7447/tcp` 개방,
  `restart: unless-stopped`
- `micro_ros_agent`는 micro-ROS(시리얼 브릿지)라 RMW 변경과 무관

## 4. 개발 PC 구성 (rviz2 + Zenoh)

### 4.1 새 파일

```
rviz/
 ├─ docker-compose.rviz.yml    ← 개발 PC 전용 실행 구성
 ├─ Dockerfile.rviz            ← ros:jazzy-desktop 기반
 ├─ uel_rover.rviz             ← rviz2 화면 프리셋
 ├─ zenoh_client.json5         ← 젯슨 라우터 접속 설정
 └─ rviz.sh                    ← xhost + docker compose 원줄 실행 스크립트
```

### 4.2 Dockerfile.rviz

- 베이스: `osrf/ros:jazzy-desktop` (rviz2 기본 포함)
- 추가 설치: `ros-jazzy-nav2-rviz-plugins`, `ros-jazzy-rmw-zenoh-cpp`
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (젯슨과 동일 필수)

### 4.3 docker-compose.rviz.yml (핵심)

```yaml
environment:
  - DISPLAY=${DISPLAY}
  - RMW_IMPLEMENTATION=rmw_zenoh_cpp
  - ZENOH_SESSION_CONFIG_URI=/config/zenoh_client.json5
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
  - ./rviz:/config
network_mode: host
command: rviz2 -d /config/uel_rover.rviz
```

- 실행 전 호스트에서 `xhost +local:docker` (rviz.sh가 처리)

### 4.4 zenoh_client.json5

```json5
{ mode: "client", connect: { endpoints: ["tcp/192.168.0.112:7447"] } }
```

### 4.5 uel_rover.rviz 프리셋 디스플레이

- Fixed Frame = `map`
- Map(`/map`), LaserScan(`/scan`), RobotModel(URDF), TF
- Nav2 Path / Costmap
- 툴바: **2D Goal Pose**(목표점), **2D Pose Estimate**(초기위치)

## 5. 드라이브 모드 (기존 RC/AUTO/STOP 유지)

`mode_manager` 코드 변경 없음. RC 스위치(ch5) → `/drive_mode`:

| 모드 | 역할 |
|------|------|
| `RC`   | 수동 조종 (Nav2 명령 무시). **매핑 시 이 모드로 수동 주행** |
| `AUTO` | Nav2 `/cmd_vel` → 모터 전달 (rviz2 목표점으로 주행) |
| `STOP` | 즉시 정지 |

- **RC 모드 매핑이 되는 이유:** `odom_publisher`는 `/wheel_velocity`(엔코더
  RPM)만으로 odom을 계산 → 드라이브 모드와 무관하게 항상 발행. slam_toolbox는
  `/scan` + `odom→base_link` TF만 있으면 맵 생성. `mode_manager`는 Nav2
  `/cmd_vel` 경로만 게이팅하므로 SLAM에 영향 없음.
- **안전:** AUTO 중 위험 시 RC/STOP으로 즉시 회수 (조종기 = 데드맨 스위치)
- rviz2는 이 모드와 독립 — 사용자가 상시 실행

## 6. 사용 흐름

### A. 매핑 (RC 수동 주행)
```
젯슨$  docker compose up -d zenoh_router micro_ros_agent
젯슨$  docker compose run uel_rover \
         ros2 launch uel_rover_control mapping.launch.py
개발PC$ ./rviz/rviz.sh
→ 조종기 RC 모드로 천천히 주행, rviz2에서 /map 확장 확인
→ ros2 run nav2_map_server map_saver_cli -f /maps/office
```

### B. 주행 (저장맵 + AMCL + rviz2 목표점)
```
젯슨$  ros2 launch uel_rover_control localization.launch.py map:=/maps/office.yaml
개발PC$ ./rviz/rviz.sh  (이미 떠 있으면 유지)
→ rviz2 "2D Pose Estimate"로 초기위치 지정
→ rviz2 "2D Goal Pose"로 목표점 클릭  (Nav2 경로 계산·대기)
→ 조종기 AUTO 전환                    (출발)
→ 위험 시 RC/STOP                     (즉시 회수)
```

## 7. 검증 체크리스트

1. **Zenoh 연결:** 개발 PC `ros2 topic list`에 젯슨 토픽(`/scan`,`/map`,`/odom`) 표시
2. **TF 트리:** `map→odom→base_link→camera_link` 연결 (`view_frames`)
3. **/wheel_velocity:** RC 수동 주행 중에도 발행 (`ros2 topic hz /wheel_velocity`) — 매핑 가능 여부의 핵심
4. **매핑:** rviz2에서 `/map`이 주행에 따라 확장
5. **맵 저장/로드:** `office.yaml`+`.pgm` 생성, localization 모드에서 로드
6. **목표점 주행:** rviz2 목표 클릭 → AUTO 시 실제 주행, STOP 시 정지

## 8. 리스크 / 주의

- **RMW 일치:** 젯슨·개발 PC 모두 jazzy + `rmw_zenoh_cpp` 여야 통신됨
- **방화벽:** Zenoh 포트 `7447/tcp` 개방 필요
- **odom drift:** 긴 수동 매핑 시 누적 오차 → slam_toolbox 루프 클로저로 보정
  (급격한 회전 자제)
- **펌웨어 확인:** OpenRB-150이 RC 모드에서도 `/wheel_velocity` 발행 유지해야 함
  (검증 3번)
- **X11 권한:** 개발 PC 재부팅 시 `xhost +local:docker` 재실행 필요 (rviz.sh가 처리)

## 9. 범위 밖 (YAGNI)

- Gazebo 시뮬레이션 (별도 프로젝트로 분리, 휴지통의 `uel_rover_gazebo` 참고)
- 다중 맵 관리 UI, 웨이포인트 자동 순회, 도킹
- rviz2 자동 실행/모드 연동 (의도적으로 수동·독립 유지)
