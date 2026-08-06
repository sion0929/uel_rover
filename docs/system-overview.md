# UEL Rover — 시스템 구조 / 토픽 흐름 정리

> 현재 `feature/rviz2-slam-nav` 브랜치 기준. 통신은 **RMW=DDS**(그래프 전체) +
> 젯슨↔개발PC 무선 구간만 **zenoh-bridge-ros2dds**로 브릿지 (방법 A).

## 1. 물리 구성

```
OpenRB-150(펌웨어)  ─USB 시리얼(XRCE-DDS)─  [젯슨 오린]  ─WiFi(Zenoh)─  [개발 PC]
 · 바퀴 엔코더                                · 센서/SLAM/Nav2 연산         · rviz2 시각화
 · 모터 드라이버                              · 전부 네이티브 실행,         · 목표점 지정
 · RC 수신(FS-iA10B ch5)                       systemd 3종으로 상시 구동
```

## 2. 노드(모듈) 목록과 역할

### 젯슨 — 커스텀 노드 (이 저장소에서 구현)
| 노드 | 구현 | 역할 |
|------|------|------|
| `odom_publisher` | `src/odom_publisher.cpp` | 바퀴 RPM → 오도메트리 계산 |

### 젯슨 — 외부 패키지 노드 (설치본 사용)
| 노드 | 패키지 | 역할 |
|------|--------|------|
| `micro_ros_agent` | micro-ROS | OpenRB-150 시리얼 ↔ ROS2 브릿지 |
| `robot_state_publisher` | robot_state_publisher | URDF → TF(base_link→camera_link 등) |
| `camera` | realsense2_camera | D455 depth/color/pointcloud 발행 |
| `pointcloud_to_laserscan` | pointcloud_to_laserscan | 3D 포인트클라우드 → 2D `/scan` |
| `slam_toolbox` | slam_toolbox | **[매핑 모드]** 지도 생성 + map→odom |
| `map_server` | nav2_map_server | **[주행 모드]** 저장맵 발행 |
| `amcl` | nav2_amcl | **[주행 모드]** 저장맵 기반 위치추정 + map→odom |
| `controller_server` | nav2_controller | 지역 경로추종 → `/cmd_vel` |
| `planner_server` | nav2_planner | 전역 경로계획 → `/plan` |
| `bt_navigator` | nav2_bt_navigator | 목표(`/goal_pose`) 받아 내비 오케스트레이션 |
| `behavior_server` | nav2_behaviors | 복구 동작(spin/backup/wait) |
| `waypoint_follower` | nav2_waypoint_follower | 웨이포인트 순회 |
| `lifecycle_manager_*` | nav2_lifecycle_manager | Nav2/localization 노드 lifecycle 관리 |
| `zenoh_bridge` | zenoh-bridge-ros2dds | 로컬 DDS 그래프 ↔ Zenoh(무선, listen :7447) |

### 개발 PC
| 노드 | 역할 |
|------|------|
| `zenoh_bridge` | 젯슨 브릿지에 접속 → 젯슨 토픽을 로컬 DDS로 재주입 |
| `rviz2` | 지도/스캔/로봇 시각화 + 2D Goal Pose/Pose Estimate 발행 |

## 3. 핵심 토픽 (누가 pub → 누가 sub)

| 토픽 | 타입 | 발행 | 구독 |
|------|------|------|------|
| `/wheel_velocity` | Float32MultiArray `[vL_rpm, vR_rpm]` | OpenRB-150(agent 경유) | `odom_publisher` |
| `/drive_mode` | String `"RC"/"AUTO"/"STOP"` | OpenRB-150(agent 경유) | `mode_manager` |
| `/odom` | nav_msgs/Odometry | `odom_publisher` | Nav2(참조) |
| `/camera/camera/depth/color/points` | PointCloud2 | `camera` | `pointcloud_to_laserscan` |
| `/scan` | LaserScan | `pointcloud_to_laserscan` | slam_toolbox / amcl / costmap |
| `/map` | OccupancyGrid | slam_toolbox(매핑) 또는 map_server(주행) | amcl, global_costmap, rviz2 |
| `/goal_pose` | PoseStamped | **rviz2 (2D Goal Pose)** | `bt_navigator` |
| `/initialpose` | PoseWithCovarianceStamped | **rviz2 (2D Pose Estimate)** | `amcl` |
| `/plan` | Path | `planner_server` | rviz2 |
| `/cmd_vel_out` | Twist(비-stamped) | `controller_server` (remap: `cmd_vel`→`/cmd_vel_out`) | OpenRB-150(agent 경유) |

## 4. TF 트리 (좌표계 연결)

```
map ──(slam_toolbox 또는 amcl)──▶ odom ──(odom_publisher)──▶ base_link ──(URDF/robot_state_publisher)──▶ camera_link, 바퀴들
```
- `map→odom` : 매핑=slam_toolbox / 주행=amcl (둘 중 하나만)
- `odom→base_link` : `odom_publisher` (바퀴 RPM 적분, **드라이브 모드와 무관하게 항상**)
- `base_link→...` : URDF 정적 변환

## 5. 데이터 흐름 (모드별)

### 매핑 모드 (`mapping.launch.py`, RC 수동 주행)
```
RC 조종 → 모터 회전 → 엔코더 → /wheel_velocity → odom_publisher → /odom + odom→base_link
D455 → /camera/.../points → pointcloud_to_laserscan → /scan
       /scan + (odom→base_link) → slam_toolbox → /map + map→odom
개발PC rviz2에서 /map 실시간 확인 → map_saver_cli로 저장
```

### 주행 모드 (`localization.launch.py` + rviz2 목표점)
```
map_server → /map(저장맵)
/scan + /map + odom→base_link → amcl → map→odom (위치추정)
rviz2 "2D Goal Pose" → /goal_pose → bt_navigator → planner_server(/plan)
                                                  → controller_server(/cmd_vel)
/cmd_vel → mode_manager ──[AUTO일 때만]──▶ /cmd_vel_out → OpenRB-150 → 모터
```

## 6. 드라이브 모드 게이팅 (안전 스위치)

게이팅은 OpenRB-150(MCU) 단일 지점에서 일어난다. MCU가 RC 채널(ch5)을 직접 읽어
`RC`/`AUTO`/`STOP`을 판정하고, `/cmd_vel_out`(Nav2 `controller_server`가 remap으로
직접 발행)을 다음과 같이 처리한다:
- **RC** : `/cmd_vel_out` 무시, RC 채널 값으로 직접 구동(수동 조종). 매핑은 이 모드로 주행.
- **AUTO** : `/cmd_vel_out` 그대로 적용 (rviz2 목표점으로 자율주행).
- **STOP** : 즉시 정지.

Jetson 쪽엔 이 게이팅을 중계하는 노드가 없다(2026-07-22 `mode_manager` 제거 이후
MCU 단일 게이팅으로 일원화). `/drive_mode`는 MCU가 계속 발행하며 관측용으로
rviz2/rqt에서 직접 구독 가능하다.

→ Nav2가 폭주해도 조종기 스위치를 RC/STOP으로 넘기면 즉시 회수(데드맨). SLAM은
이 게이팅과 무관하게 항상 동작(odom·scan은 계속 흐름).

## 7. 통신 계층 (방법 A)

```
[젯슨 로컬 DDS 그래프: agent + odom + mode + camera + p2l + slam/amcl + Nav2]
        └── zenoh_bridge (listen :7447) ══ WiFi/Zenoh ══ zenoh_bridge (connect) ──┐
                                                                                   ▼
                                                    [개발 PC 로컬 DDS 그래프: rviz2]
```
- RMW은 **전부 기본 DDS** → `micro_ros_agent`(DDS 네이티브) 정상 통신.
- Zenoh는 오직 **머신 경계 브릿지**로만 사용 (무선 discovery/전송 안정화).
- 자세한 배경: `docs/superpowers/specs/2026-07-04-rviz2-slam-goal-navigation-design.md` §3.4/§8
