# rviz2 + SLAM 목표점 자율주행 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** RealSense SLAM 맵을 만들고 개발 PC rviz2에서 찍은 목표점으로 로버가 자율주행하게 하며, 통신을 Zenoh로 전환한다.

**Architecture:** 기존 `nav2_bringup.launch.py`를 공통부(`bringup_common`)와 모드별(`mapping`/`localization`) launch로 분리(접근 A). 젯슨은 headless 연산, 개발 PC(Ubuntu)는 Docker rviz2로 원격 시각화. RMW를 `rmw_zenoh_cpp`로 통일하고 젯슨에 Zenoh 라우터를 둔다. 드라이브 모드(RC/AUTO/STOP)는 기존 `mode_manager` 그대로 사용.

**Tech Stack:** ROS 2 Jazzy, Nav2, slam_toolbox, nav2_amcl, nav2_map_server, rmw_zenoh_cpp, Docker/Compose, RealSense D455.

## Global Constraints

- ROS 2 배포판: **jazzy** (젯슨·개발 PC·모든 컨테이너 동일)
- RMW: **`rmw_zenoh_cpp`** (양쪽 반드시 동일해야 통신)
- 젯슨 Zenoh 라우터 주소: **`tcp/192.168.0.112:7447`**
- `ROS_DOMAIN_ID=0`, 모든 컨테이너 `network_mode: host`
- `/cmd_vel`은 비-stamped Twist (mode_manager 호환) — 기존 유지
- 패키지 `CMakeLists.txt`는 `launch`·`config` 디렉토리를 통째로 install → **새 launch/config 파일은 CMake 수정 없이 자동 설치**
- 이 저장소는 아직 git이 아님 → 커밋 스텝은 `git init` 이후에만 유효 (Task 0 참조)

---

## File Structure

**수정/생성 (젯슨 패키지 `src/uel_rover_control/`):**
- `launch/bringup_common.launch.py` — 생성 (공통 노드 전부)
- `launch/mapping.launch.py` — 생성 (common + slam_toolbox)
- `launch/localization.launch.py` — 생성 (common + map_server + amcl)
- `launch/nav2_bringup.launch.py` — 삭제 (mapping/localization으로 대체)
- `config/localization.yaml` — 생성 (amcl + map_server + global_costmap static 오버레이)

**수정/생성 (프로젝트 루트):**
- `Dockerfile` — 수정 (rmw_zenoh_cpp 설치)
- `docker-compose.yml` — 수정 (zenoh_router 서비스, RMW env, /maps 볼륨)

**생성 (개발 PC용, `rviz/`):**
- `rviz/Dockerfile.rviz`
- `rviz/docker-compose.rviz.yml`
- `rviz/zenoh_client.json5`
- `rviz/uel_rover.rviz`
- `rviz/rviz.sh`

---

## Task 0: git 초기화 (선택, 커밋 스텝 활성화용)

**Files:**
- Create: `.gitignore`

**Interfaces:**
- Produces: git 저장소 → 이후 태스크의 `git commit` 스텝이 동작

- [ ] **Step 1: 저장소 초기화**

```bash
cd /home/a/uel_rover
git init
```

- [ ] **Step 2: .gitignore 작성**

```
build/
install/
log/
.vscode/browse.vc.db*
maps/
__pycache__/
```

- [ ] **Step 3: 최초 커밋**

```bash
cd /home/a/uel_rover
git add -A
git commit -m "chore: initialize git repository"
```

> 이 태스크를 건너뛰면 이후 모든 `git commit` 스텝을 생략하고 파일 저장까지만 수행한다.

---

## Task 1: bringup_common.launch.py 추출

기존 `nav2_bringup.launch.py`에서 SLAM을 제외한 전부(rover 노드, robot_state_publisher, 카메라, pointcloud_to_laserscan, Nav2 서버 5개, lifecycle_manager)를 공통 launch로 옮긴다.

**Files:**
- Create: `src/uel_rover_control/launch/bringup_common.launch.py`

**Interfaces:**
- Produces: `generate_launch_description()` — `use_sim_time` 인자 선언, slam/localization을 제외한 모든 노드 실행. mapping/localization launch가 `IncludeLaunchDescription`으로 포함한다.

- [ ] **Step 1: 공통 launch 파일 작성**

`src/uel_rover_control/launch/bringup_common.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 공통 브링업: SLAM/localization을 제외한 모든 노드
# (rover 노드, TF, 카메라, scan 변환, Nav2 서버, lifecycle_manager)
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')

    urdf_path   = os.path.join(pkg, 'urdf',   'uel_rover.urdf')
    params      = os.path.join(pkg, 'config', 'rover_params.yaml')
    p2l_params  = os.path.join(pkg, 'config', 'pointcloud_to_laserscan.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── 로버 기본 노드 ────────────────────────────────────────
        Node(package='uel_rover_control', executable='odom_publisher',
             name='odom_publisher', output='screen', parameters=[params]),
        Node(package='uel_rover_control', executable='mode_manager',
             name='mode_manager', output='screen', parameters=[params]),

        # ── URDF → TF ─────────────────────────────────────────────
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc,
                          'use_sim_time': use_sim_time}]),

        # ── RealSense D455 ────────────────────────────────────────
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='camera', output='screen',
             parameters=[{
                 'initial_reset': True,
                 'enable_depth': True,
                 'enable_color': True,
                 'enable_infra1': False,
                 'enable_infra2': False,
                 'pointcloud.enable': True,
                 'enable_sync': True,
                 'depth_module.depth_profile': '640x480x30',
                 'rgb_camera.color_profile': '640x480x30',
                 'enable_gyro': False,
                 'enable_accel': False,
             }]),

        # ── PointCloud2 → LaserScan (/scan) ───────────────────────
        Node(package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen',
             parameters=[p2l_params],
             remappings=[('cloud_in', '/camera/depth/color/points'),
                         ('scan', '/scan')]),

        # ── Nav2 핵심 서버 ────────────────────────────────────────
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[nav2_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=[nav2_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[nav2_params]),

        # ── Nav2 라이프사이클 매니저 (Nav2 서버만 관리) ───────────
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True, 'node_names': nav2_lifecycle_nodes}]),
    ])
```

- [ ] **Step 2: 로컬 문법 검증 (ROS2 불필요)**

Run: `python3 -m py_compile src/uel_rover_control/launch/bringup_common.launch.py && echo OK`
Expected: `OK` (문법 오류 없음)

- [ ] **Step 3: 커밋**

```bash
git add src/uel_rover_control/launch/bringup_common.launch.py
git commit -m "feat: extract bringup_common launch (nav2 + sensors, no SLAM)"
```

---

## Task 2: mapping.launch.py

**Files:**
- Create: `src/uel_rover_control/launch/mapping.launch.py`

**Interfaces:**
- Consumes: `bringup_common.launch.py` (include), `config/slam_toolbox.yaml`
- Produces: `mapping.launch.py` — 매핑 모드 엔트리. `lifecycle_manager_slam`이 `slam_toolbox` 노드를 관리.

- [ ] **Step 1: 파일 작성**

`src/uel_rover_control/launch/mapping.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 매핑 모드: 공통 브링업 + slam_toolbox(온라인 async 매핑)
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_common.launch.py'))),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[slam_params, {'use_sim_time': use_sim_time}]),
    ])
```

- [ ] **Step 2: 로컬 문법 검증**

Run: `python3 -m py_compile src/uel_rover_control/launch/mapping.launch.py && echo OK`
Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
git add src/uel_rover_control/launch/mapping.launch.py
git commit -m "feat: add mapping launch (common + slam_toolbox)"
```

---

## Task 3: localization.yaml (AMCL + map_server + static global costmap)

위치추정 모드 전용 파라미터 오버레이. `map_server`·`amcl`을 정의하고, `global_costmap`을 rolling → static_layer로 바꾼다.

**Files:**
- Create: `src/uel_rover_control/config/localization.yaml`

**Interfaces:**
- Produces: `map_server`, `amcl`, `global_costmap` 파라미터. `localization.launch.py`가 `nav2_params.yaml`과 함께 로드(뒤 파일이 global_costmap 오버라이드).

- [ ] **Step 1: 파일 작성**

`src/uel_rover_control/config/localization.yaml`:

```yaml
# ═══════════════════════════════════════════════════════════════
#  위치추정 모드 오버레이 — map_server + AMCL + static global costmap
#  localization.launch.py에서 nav2_params.yaml 뒤에 로드하여 병합
# ═══════════════════════════════════════════════════════════════

# ── 저장맵 서버 ─────────────────────────────────────────────────
map_server:
  ros__parameters:
    yaml_filename: ""          # localization.launch.py의 map:= 인자로 주입
    topic_name: "map"
    frame_id: "map"

# ── AMCL 위치추정 (map→odom TF 제공) ────────────────────────────
amcl:
  ros__parameters:
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    scan_topic: "scan"
    laser_model_type: "likelihood_field"
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    min_particles: 500
    max_particles: 2000
    update_min_d: 0.15
    update_min_a: 0.2
    laser_max_range: 6.0
    laser_min_range: 0.1
    max_beams: 60
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    transform_tolerance: 0.5
    tf_broadcast: true

# ── global_costmap: rolling → static (저장맵 사용) ──────────────
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: false
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
```

- [ ] **Step 2: 로컬 YAML 문법 검증 (ROS2 불필요)**

Run: `python3 -c "import yaml,sys; yaml.safe_load(open('src/uel_rover_control/config/localization.yaml')); print('OK')"`
Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
git add src/uel_rover_control/config/localization.yaml
git commit -m "feat: add localization params (amcl + map_server + static costmap)"
```

---

## Task 4: localization.launch.py

**Files:**
- Create: `src/uel_rover_control/launch/localization.launch.py`

**Interfaces:**
- Consumes: `bringup_common.launch.py`, `config/nav2_params.yaml`, `config/localization.yaml`
- Produces: `localization.launch.py` — `map` 인자로 저장맵 경로 수신. `map_server`·`amcl` 실행 + 이들을 관리하는 `lifecycle_manager_localization`.

- [ ] **Step 1: 파일 작성**

`src/uel_rover_control/launch/localization.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 위치추정 모드: 공통 브링업 + map_server + AMCL (저장맵 주행)
def generate_launch_description():
    pkg = get_package_share_directory('uel_rover_control')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    loc_params  = os.path.join(pkg, 'config', 'localization.yaml')

    map_yaml     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        DeclareLaunchArgument('map', description='저장맵 .yaml 경로 (예: /maps/office.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup_common.launch.py'))),

        # map_server: nav2_params + localization 오버레이 + map 경로 주입
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[loc_params, {'yaml_filename': map_yaml,
                                      'use_sim_time': use_sim_time}]),

        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=[loc_params, {'use_sim_time': use_sim_time}]),

        # map_server/amcl 전용 lifecycle 매니저
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': lifecycle_nodes}]),
    ])
```

> 참고: `bringup_common`의 Nav2 서버들은 `nav2_params.yaml`만 로드하므로, static global_costmap 오버레이(`localization.yaml`)를 controller/planner에도 반영하려면 Task 8(선택)에서 다룬다. 기본 동작(AMCL 위치추정 + 목표 주행)은 이 태스크로 성립한다.

- [ ] **Step 2: 로컬 문법 검증**

Run: `python3 -m py_compile src/uel_rover_control/launch/localization.launch.py && echo OK`
Expected: `OK`

- [ ] **Step 3: 커밋**

```bash
git add src/uel_rover_control/launch/localization.launch.py
git commit -m "feat: add localization launch (map_server + amcl)"
```

---

## Task 5: 기존 nav2_bringup.launch.py 제거

**Files:**
- Delete: `src/uel_rover_control/launch/nav2_bringup.launch.py`

**Interfaces:**
- Consumes: 없음
- Produces: 없음 (mapping/localization으로 완전 대체)

- [ ] **Step 1: 파일 삭제**

```bash
git rm src/uel_rover_control/launch/nav2_bringup.launch.py
```

(git 미사용 시: `rm src/uel_rover_control/launch/nav2_bringup.launch.py`)

- [ ] **Step 2: 잔존 참조 확인**

Run: `grep -rn "nav2_bringup" src/ docker-compose.yml Dockerfile 2>/dev/null || echo "참조 없음"`
Expected: `참조 없음` (docker-compose는 Task 7에서 mapping/localization으로 교체됨 — 이 시점엔 아직 참조가 남아있을 수 있으니, 남으면 Task 7에서 정리)

- [ ] **Step 3: 커밋**

```bash
git add -A
git commit -m "refactor: remove nav2_bringup launch (replaced by mapping/localization)"
```

---

## Task 6: Dockerfile — rmw_zenoh_cpp 설치

**Files:**
- Modify: `Dockerfile` (apt 설치 블록, `ros-jazzy-pointcloud-to-laserscan` 다음 줄)

**Interfaces:**
- Produces: 이미지에 `rmw_zenoh_cpp` 포함 → Zenoh RMW 사용 가능

- [ ] **Step 1: apt 목록에 zenoh RMW + amcl/map_server 추가**

`Dockerfile`에서 아래 줄을 찾는다:

```dockerfile
    ros-jazzy-pointcloud-to-laserscan \
```

바로 다음에 삽입:

```dockerfile
    # Zenoh RMW + 위치추정(AMCL/map_server)
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-map-server \
```

> `ros-jazzy-navigation2` 메타패키지에 amcl/map_server가 포함되지만, 명시 설치로 확실히 보장한다.

- [ ] **Step 2: Dockerfile 문법 스모크 체크 (빌드 환경, 하드웨어 불필요)**

Run: `grep -n "rmw-zenoh-cpp" Dockerfile && echo OK`
Expected: 해당 줄 출력 + `OK`

- [ ] **Step 3: (빌드 가능 환경에서) 이미지 빌드**

Run: `docker build -t uel_rover:jazzy .`
Expected: 성공 (`rmw-zenoh-cpp` 설치 로그 확인). *네트워크/도커 필요 — 실기/개발 PC에서 수행.*

- [ ] **Step 4: 커밋**

```bash
git add Dockerfile
git commit -m "feat: install rmw_zenoh_cpp + amcl/map_server in image"
```

---

## Task 7: docker-compose.yml — Zenoh 라우터 + RMW env + /maps 볼륨 + 실행 모드

**Files:**
- Modify: `docker-compose.yml`

**Interfaces:**
- Consumes: `mapping.launch.py` (기본 실행), `zenoh_router` 서비스
- Produces: `docker compose up`으로 zenoh_router + micro_ros_agent + uel_rover(기본 매핑) 기동

- [ ] **Step 1: zenoh_router 서비스 추가**

`docker-compose.yml`의 `services:` 아래(맨 위 서비스로) 추가:

```yaml
  # ── Zenoh 라우터 (다중 머신 discovery 허브) ────────────────────
  zenoh_router:
    image: uel_rover:jazzy
    container_name: zenoh_router
    network_mode: host
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
    command: >
      bash -c "source /opt/ros/jazzy/setup.bash &&
               ros2 run rmw_zenoh_cpp rmw_zenohd"
    restart: unless-stopped
```

- [ ] **Step 2: micro_ros_agent는 그대로 두고, uel_rover 서비스에 RMW env + /maps 볼륨 추가 및 실행 모드 변경**

`uel_rover` 서비스의 `environment`를 아래로 교체:

```yaml
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

`volumes`에 `/maps` 추가 (기존 두 줄 유지):

```yaml
    volumes:
      - ./src:/ros2_ws/src
      - /dev:/dev
      - ./maps:/maps
```

`depends_on`에 zenoh_router 추가:

```yaml
    depends_on:
      - micro_ros_agent
      - zenoh_router
```

`command`의 launch를 매핑 모드로 교체:

```yaml
    command: >
      bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        ros2 launch uel_rover_control mapping.launch.py
      "
```

- [ ] **Step 3: 로컬 YAML 문법 검증**

Run: `python3 -c "import yaml; yaml.safe_load(open('docker-compose.yml')); print('OK')"`
Expected: `OK`

- [ ] **Step 4: (도커 있는 환경) compose 구성 검증**

Run: `docker compose config >/dev/null && echo VALID`
Expected: `VALID`

- [ ] **Step 5: 커밋**

```bash
git add docker-compose.yml
git commit -m "feat: add zenoh router, RMW env, maps volume; default to mapping mode"
```

---

## Task 8: 개발 PC rviz2 패키지

**Files:**
- Create: `rviz/Dockerfile.rviz`
- Create: `rviz/zenoh_client.json5`
- Create: `rviz/docker-compose.rviz.yml`
- Create: `rviz/uel_rover.rviz`
- Create: `rviz/rviz.sh`

**Interfaces:**
- Consumes: 젯슨의 Zenoh 라우터(`tcp/192.168.0.112:7447`), 토픽 `/map`,`/scan`, TF, `/robot_description`
- Produces: 개발 PC에서 `./rviz/rviz.sh`로 rviz2 실행

- [ ] **Step 1: Dockerfile.rviz 작성**

`rviz/Dockerfile.rviz`:

```dockerfile
FROM osrf/ros:jazzy-desktop
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-nav2-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

- [ ] **Step 2: zenoh_client.json5 작성**

`rviz/zenoh_client.json5`:

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/192.168.0.112:7447"],
  },
}
```

- [ ] **Step 3: docker-compose.rviz.yml 작성**

`rviz/docker-compose.rviz.yml`:

```yaml
services:
  rviz2:
    build:
      context: .
      dockerfile: Dockerfile.rviz
    image: uel_rover_rviz:jazzy
    container_name: uel_rover_rviz
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_SESSION_CONFIG_URI=/config/zenoh_client.json5
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./:/config
    command: >
      bash -c "source /opt/ros/jazzy/setup.bash &&
               rviz2 -d /config/uel_rover.rviz"
```

- [ ] **Step 4: uel_rover.rviz 작성 (최소 프리셋)**

`rviz/uel_rover.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
Visualization Manager:
  Global Options:
    Fixed Frame: map
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
    - Class: rviz_default_plugins/Map
      Name: Map
      Enabled: true
      Topic:
        Value: /map
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Enabled: true
      Topic:
        Value: /scan
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
        Durability Policy: Transient Local
    - Class: nav2_rviz_plugins/Path
      Name: Nav2 Path
      Enabled: true
      Topic:
        Value: /plan
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
```

- [ ] **Step 5: rviz.sh 작성**

`rviz/rviz.sh`:

```bash
#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
xhost +local:docker
docker compose -f docker-compose.rviz.yml up --build
```

- [ ] **Step 6: 실행 권한 + 로컬 문법 검증**

Run:
```bash
chmod +x rviz/rviz.sh
python3 -c "import yaml; yaml.safe_load(open('rviz/docker-compose.rviz.yml')); yaml.safe_load(open('rviz/uel_rover.rviz')); print('OK')"
bash -n rviz/rviz.sh && echo "SH OK"
```
Expected: `OK` 및 `SH OK`

- [ ] **Step 7: 커밋**

```bash
git add rviz/
git commit -m "feat: add dev-PC rviz2 package (docker + zenoh client + preset)"
```

---

## Task 9: 실기 통합 검증 (하드웨어 필요 — 젯슨 + 개발 PC)

빌드/문법 검증을 넘어 실제 동작을 확인한다. **RealSense·OpenRB-150·젯슨·개발 PC가 있어야 한다.** 코드 산출물이 아니라 검증 절차이므로 커밋은 없다.

**Files:** 없음 (검증 전용)

- [ ] **Step 1: 이미지 빌드 (젯슨, 개발 PC)**

젯슨: `docker build -t uel_rover:jazzy .`
개발 PC: `docker compose -f rviz/docker-compose.rviz.yml build`
Expected: 양쪽 성공

- [ ] **Step 2: 젯슨 기동 + Zenoh 라우터 확인**

Run(젯슨): `docker compose up -d && docker compose logs zenoh_router | tail`
Expected: `rmw_zenohd` 라우터 리스닝 로그, 포트 7447

- [ ] **Step 3: 개발 PC에서 원격 토픽 발견 (Zenoh 연결 검증)**

Run(개발 PC): `./rviz/rviz.sh` 실행 후 별도 셸에서 컨테이너 진입해
`ros2 topic list`
Expected: `/scan`, `/odom`, `/map`, `/robot_description` 등 젯슨 토픽이 보임

- [ ] **Step 4: TF 트리 검증**

Run: `ros2 run tf2_tools view_frames`
Expected: 매핑 모드에서 `map→odom→base_link→camera_link` 연결

- [ ] **Step 5: RC 매핑 핵심 — /wheel_velocity 발행 확인**

조종기를 **RC 모드**로 두고 로버를 손으로 조종하며:
Run: `ros2 topic hz /wheel_velocity`
Expected: 0이 아닌 주기적 발행 (미발행 시 OpenRB-150 펌웨어가 RC 모드에서 엔코더 발행을 끊는 것 → 펌웨어 수정 필요)

- [ ] **Step 6: 매핑 → 저장**

RC로 공간을 주행해 rviz2에서 `/map` 확장 확인 후:
Run(젯슨 컨테이너): `ros2 run nav2_map_server map_saver_cli -f /maps/office`
Expected: `/maps/office.yaml` + `/maps/office.pgm` 생성 (호스트 `./maps/`에 보존)

- [ ] **Step 7: 위치추정 모드 주행**

Run(젯슨): `ros2 launch uel_rover_control localization.launch.py map:=/maps/office.yaml`
개발 PC rviz2에서:
1. "2D Pose Estimate"로 초기위치 지정 → AMCL 수렴 확인
2. "2D Goal Pose"로 목표점 클릭 → `/plan` 경로 표시
3. 조종기 **AUTO** 전환 → 로버가 목표로 주행
4. 조종기 **STOP/RC** 전환 → 즉시 정지/수동 회수
Expected: 목표 도달, 모드 전환 안전 동작

---

## Self-Review (작성자 점검 결과)

**Spec 커버리지:**
- rviz2 원격(개발 PC) → Task 8 ✅
- 2모드 SLAM(매핑/위치추정) → Task 2, 3, 4 ✅
- RC 수동 매핑 → Task 9 Step 5·6 (mode_manager 무변경, 검증으로 커버) ✅
- Zenoh 전환 → Task 6(이미지), Task 7(라우터/env), Task 8(클라이언트) ✅
- rviz2 드라이브 모드 독립 → Task 8(별도 compose, 상시 실행) ✅
- 맵 저장/보존 → Task 7(/maps 볼륨), Task 9 Step 6 ✅
- 검증 체크리스트 6항목 → Task 9 Step 2~7에 대응 ✅

**Placeholder 스캔:** `map_server.yaml_filename: ""`는 의도적(런타임 `map:=`로 주입, launch에서 오버라이드). `/maps/office`는 예시명. TODO/TBD 없음.

**타입/이름 일관성:** launch executable·노드명(`odom_publisher`, `mode_manager`, `controller_server` 등)은 기존 코드/CMakeLists와 일치. lifecycle 노드명(`map_server`,`amcl`)이 Task 3 params 키와 일치. Zenoh env 키 `RMW_IMPLEMENTATION`, `ZENOH_SESSION_CONFIG_URI` 전 태스크 동일.

**알려진 한계(의도적):** Task 4 노트대로 controller/planner는 `nav2_params.yaml`만 로드 → global static costmap은 map_server가 발행하는 `/global_costmap`가 아니라 별도 반영 필요. 기본 목표주행은 성립하며, 정적맵 기반 전역계획 고도화는 향후 튜닝(별도 태스크)으로 분리.
