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
│  micro_ros_agent ── OpenRB-150 (바퀴 RPM/명령, 시리얼)              │
│  odom_publisher, mode_manager                                       │
│  RealSense D455 → pointcloud_to_laserscan → /scan                   │
│  ── 위 노드 전부 로컬 DDS(ROS_DOMAIN_ID=0)로 통신 ──                 │
│  [모드① 매핑]   slam_toolbox      → /map + map→odom TF              │
│  [모드② 주행]   map_server+AMCL   → 저장맵 + map→odom TF            │
│  Nav2 (controller/planner/behavior/bt/waypoint) + lifecycle_manager │
│  zenoh_bridge (zenoh-bridge-ros2dds, listen :7447) ── DDS↔Zenoh     │
└──────────────────────────────── │ Zenoh (무선 링크) ───────────────┘
                                   ↕  같은 LAN/WiFi (192.168.0.x)
┌─────────────── 개발 PC (Ubuntu, Docker jazzy-desktop) ─────────────┐
│  zenoh_bridge (connect 192.168.0.112:7447) ── 젯슨 토픽을 로컬 DDS로 │
│  rviz2 ── (로컬 DDS 구독) 맵/스캔/로봇모델 + "2D Goal Pose" → /goal_pose │
│         (드라이브 모드와 무관하게 상시 실행)                         │
└────────────────────────────────────────────────────────────────────┘
```

- **역할 분리:** 젯슨 = 센서·SLAM·Nav2 연산 / 개발 PC = 화면·목표점 지정
- **통신(방법 A):** RMW는 **양쪽 모두 기본 DDS 유지**. 각 머신에
  `zenoh-bridge-ros2dds`를 두고, **젯슨↔개발PC 무선 구간만 Zenoh로 브릿지**한다.
  개발 PC 브릿지가 젯슨 브릿지(`tcp/192.168.0.112:7447`)에 접속해 젯슨 토픽을
  개발 PC 로컬 DDS 그래프로 재주입 → rviz2는 로컬 토픽처럼 구독.
  - **왜 rmw_zenoh_cpp를 안 쓰나:** `micro_ros_agent`(Micro XRCE-DDS Agent)는
    DDS 네이티브라 RMW를 Zenoh로 못 바꾼다. RMW를 Zenoh로 통일하면 agent만
    DDS 섬으로 남아 `/wheel_velocity`·모터명령이 끊긴다. 그래서 RMW는 DDS로
    두고 머신 경계에서만 Zenoh 브릿지를 쓴다(ROS2+Zenoh 표준 사용법).
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

### 3.4 Zenoh 브릿지 (방법 A — RMW는 DDS 유지)

- **RMW 변경 없음.** 젯슨 그래프(agent·rover·Nav2·SLAM)는 전부 기본 DDS
  (`ROS_DOMAIN_ID=0`)로 통신 → micro-ROS agent 정상.
- `docker-compose.yml`에 **`zenoh_bridge` 서비스 신설**:
  이미지 `eclipse/zenoh-bridge-ros2dds`, `command: ["-l","tcp/0.0.0.0:7447"]`,
  `network_mode: host`, `ROS_DOMAIN_ID=0`, `restart: unless-stopped`.
  로컬 DDS 그래프를 발견해 `7447/tcp`로 Zenoh 노출.
- `Dockerfile`에는 `rmw_zenoh_cpp`를 **넣지 않는다** (브릿지는 별도 이미지).
  amcl/map_server만 추가.
- `micro_ros_agent`는 변경 없음 (DDS 그대로, 이제 그래프 전체가 DDS라 정상 통신).

## 4. 개발 PC 구성 (rviz2 + Zenoh 브릿지)

### 4.1 새 파일

```
rviz/
 ├─ docker-compose.rviz.yml    ← 개발 PC 전용 실행 (zenoh_bridge + rviz2)
 ├─ Dockerfile.rviz            ← ros:jazzy-desktop 기반 (기본 DDS)
 ├─ uel_rover.rviz             ← rviz2 화면 프리셋
 └─ rviz.sh                    ← xhost + docker compose 원줄 실행 스크립트
```
(라우터 접속 설정 `zenoh_client.json5`는 불필요 — 브릿지가 CLI `-e` 플래그로 접속)

### 4.2 Dockerfile.rviz

- 베이스: `osrf/ros:jazzy-desktop` (rviz2 기본 포함)
- 추가 설치: `ros-jazzy-nav2-rviz-plugins` (rmw_zenoh 불필요 — rviz2는 로컬 DDS)

### 4.3 docker-compose.rviz.yml (핵심 — 두 서비스)

```yaml
services:
  zenoh_bridge:                       # 젯슨 브릿지로 접속 → 토픽을 로컬 DDS로 재주입
    image: eclipse/zenoh-bridge-ros2dds:latest
    network_mode: host
    environment: [ROS_DOMAIN_ID=0]
    command: ["-e", "tcp/192.168.0.112:7447"]
  rviz2:                              # 로컬 DDS 그래프 구독
    build: { context: ., dockerfile: Dockerfile.rviz }
    network_mode: host
    depends_on: [zenoh_bridge]
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./:/config
    command: rviz2 -d /config/uel_rover.rviz
```

- 실행 전 호스트에서 `xhost +local:docker` (rviz.sh가 처리)
- 젯슨·개발 PC의 `zenoh-bridge-ros2dds` 이미지 **태그를 동일하게** 유지
  (Zenoh 프로토콜 버전 일치)

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
젯슨$  docker compose up              (zenoh_bridge, micro_ros_agent, uel_rover=매핑)
개발PC$ ./rviz/rviz.sh                (zenoh_bridge + rviz2)
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

- **브릿지 이미지 태그 일치:** 젯슨·개발 PC의 `zenoh-bridge-ros2dds` 태그를
  동일하게 (Zenoh 프로토콜 버전 일치). `:latest`는 실기에서 특정 버전으로 고정 권장.
- **ROS_DOMAIN_ID 일치:** 양쪽 브릿지·노드 모두 `0`.
- **방화벽:** Zenoh 포트 `7447/tcp` 개방 필요 (젯슨에서 listen)
- **RMW 불변경:** 그래프는 DDS 유지 — `micro_ros_agent`(DDS 네이티브)와의
  통신을 위해 rmw_zenoh_cpp로 바꾸지 말 것 (바꾸면 agent가 고립됨).
- **odom drift:** 긴 수동 매핑 시 누적 오차 → slam_toolbox 루프 클로저로 보정
  (급격한 회전 자제)
- **펌웨어 확인:** OpenRB-150이 RC 모드에서도 `/wheel_velocity` 발행 유지해야 함
  (검증 3번)
- **X11 권한:** 개발 PC 재부팅 시 `xhost +local:docker` 재실행 필요 (rviz.sh가 처리)

## 9. 범위 밖 (YAGNI)

- Gazebo 시뮬레이션 (별도 프로젝트로 분리, 휴지통의 `uel_rover_gazebo` 참고)
- 다중 맵 관리 UI, 웨이포인트 자동 순회, 도킹
- rviz2 자동 실행/모드 연동 (의도적으로 수동·독립 유지)
