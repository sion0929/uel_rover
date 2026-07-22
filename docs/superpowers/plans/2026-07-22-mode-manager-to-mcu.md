# mode_manager 제거 — 드라이브 모드 게이팅을 MCU로 일원화 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson 쪽 `mode_manager` 노드를 제거하고, Nav2 `controller_server`의 `/cmd_vel` 출력을 launch remap으로 곧바로 `/cmd_vel_out`에 연결해 드라이브 모드(RC/AUTO/STOP) 게이팅을 OpenRB-150(MCU) 한 곳으로 일원화한다.

**Architecture:** MCU(`openrb150_motor.ino`)는 이미 RC 채널을 직접 읽어 `AUTO`일 때만 `/cmd_vel_out` 목표 RPM을 적용하는 게이트를 갖고 있다. Jetson 쪽 `mode_manager`는 `/drive_mode`를 구독해 같은 판정을 다시 내려 `/cmd_vel`→`/cmd_vel_out`을 중계하던 중복 계층이었다. 이 계층을 제거하고 launch remap으로 대체한다.

**Tech Stack:** ROS 2 Jazzy, `launch_ros`, CMake/ament, colcon, Docker(Jetson 배포 컨테이너).

## Global Constraints

- 패키지: `uel_rover_control` (경로: `src/uel_rover_control/`)
- `odom_publisher`는 변경 없음 — 이번 작업은 `mode_manager`에만 국한
- 펌웨어(`firmware/openrb150_motor/openrb150_motor.ino`)는 변경 없음(이미 올바르게 게이팅 중, 2026-07-22 실기 조사로 코드 확인됨)
- Jetson은 `/home/spacek/uel_rover`, 컨테이너 이름 `uel_rover` (docker compose 서비스명도 동일)
- 참조 스펙: `docs/superpowers/specs/2026-07-22-mode-manager-to-mcu-design.md`

---

## File Structure

**수정:**
- `src/uel_rover_control/CMakeLists.txt` — `mode_manager` 빌드 타깃 제거
- `src/uel_rover_control/launch/bringup_common.launch.py` — `mode_manager` Node 제거, `controller_server`에 remap 추가
- `docs/system-overview.md` — 노드/토픽 표, §6 갱신

**삭제:**
- `src/uel_rover_control/src/mode_manager.cpp`
- `src/uel_rover_control/launch/rover.launch.py` (고아 파일 — 유일한 참조자였던 `nav2_bringup.launch.py`가 이미 삭제되어 저장소 어디서도 include하지 않음)

---

## Task 1: mode_manager 소스 삭제 + CMakeLists 빌드 타깃 제거

**Files:**
- Delete: `src/uel_rover_control/src/mode_manager.cpp`
- Modify: `src/uel_rover_control/CMakeLists.txt`

**Interfaces:**
- Consumes: 없음
- Produces: `uel_rover_control` 패키지가 더 이상 `mode_manager` 실행 파일을 빌드/설치하지 않음

- [ ] **Step 1: 소스 파일 삭제**

```bash
cd /home/spacek/uel_rover
git rm src/uel_rover_control/src/mode_manager.cpp
```

- [ ] **Step 2: CMakeLists.txt에서 빌드 타깃 제거**

`src/uel_rover_control/CMakeLists.txt`에서 아래 3곳을 수정한다.

기존:
```cmake
add_executable(odom_publisher src/odom_publisher.cpp)
add_executable(mode_manager   src/mode_manager.cpp)

ament_target_dependencies(odom_publisher rclcpp nav_msgs std_msgs tf2_ros tf2 geometry_msgs)
ament_target_dependencies(mode_manager   rclcpp geometry_msgs std_msgs)

# ── 설치 ─────────────────────────────────────────────────────────
install(TARGETS
  odom_publisher
  mode_manager
  DESTINATION lib/${PROJECT_NAME}
)
```

변경 후:
```cmake
add_executable(odom_publisher src/odom_publisher.cpp)

ament_target_dependencies(odom_publisher rclcpp nav_msgs std_msgs tf2_ros tf2 geometry_msgs)

# ── 설치 ─────────────────────────────────────────────────────────
install(TARGETS
  odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 3: 잔존 참조 확인**

Run: `grep -n "mode_manager" src/uel_rover_control/CMakeLists.txt || echo "참조 없음"`
Expected: `참조 없음`

- [ ] **Step 4: 커밋**

```bash
git add src/uel_rover_control/CMakeLists.txt src/uel_rover_control/src/mode_manager.cpp
git commit -m "refactor: remove mode_manager node (gating moves to MCU)"
```

---

## Task 2: bringup_common.launch.py — mode_manager 제거 + controller_server remap 추가

**Files:**
- Modify: `src/uel_rover_control/launch/bringup_common.launch.py`

**Interfaces:**
- Consumes: 없음
- Produces: `controller_server`가 `/cmd_vel` 대신 `/cmd_vel_out`으로 직접 발행

- [ ] **Step 1: mode_manager Node 제거**

기존:
```python
        # ── 로버 기본 노드 ────────────────────────────────────────
        Node(package='uel_rover_control', executable='odom_publisher',
             name='odom_publisher', output='screen', parameters=[params]),
        Node(package='uel_rover_control', executable='mode_manager',
             name='mode_manager', output='screen', parameters=[params]),
```

변경 후:
```python
        # ── 로버 기본 노드 ────────────────────────────────────────
        Node(package='uel_rover_control', executable='odom_publisher',
             name='odom_publisher', output='screen', parameters=[params]),
```

- [ ] **Step 2: controller_server에 remap 추가**

기존:
```python
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params]),
```

변경 후:
```python
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[nav2_params],
             remappings=[('cmd_vel', '/cmd_vel_out')]),
```

- [ ] **Step 3: 로컬 문법 검증**

Run: `python3 -m py_compile src/uel_rover_control/launch/bringup_common.launch.py && echo OK`
Expected: `OK`

- [ ] **Step 4: 잔존 참조 확인**

Run: `grep -n "mode_manager" src/uel_rover_control/launch/bringup_common.launch.py || echo "참조 없음"`
Expected: `참조 없음`

- [ ] **Step 5: 커밋**

```bash
git add src/uel_rover_control/launch/bringup_common.launch.py
git commit -m "feat: remap controller_server cmd_vel directly to /cmd_vel_out"
```

---

## Task 3: 고아 launch 파일(rover.launch.py) 삭제

**Files:**
- Delete: `src/uel_rover_control/launch/rover.launch.py`

**Interfaces:**
- Consumes: 없음
- Produces: 없음 (참조자가 없던 파일 정리)

- [ ] **Step 1: 참조 여부 재확인**

Run: `grep -rln "rover.launch" src/ 2>/dev/null || echo "참조 없음"`
Expected: `참조 없음` (2026-07-22 조사 시점 기준으로 이미 확인됨 — 실행 전 재확인용)

- [ ] **Step 2: 파일 삭제**

```bash
git rm src/uel_rover_control/launch/rover.launch.py
```

- [ ] **Step 3: 커밋**

```bash
git commit -m "chore: remove orphaned rover.launch.py (no longer referenced)"
```

---

## Task 4: 문서 갱신 (docs/system-overview.md)

**Files:**
- Modify: `docs/system-overview.md`

**Interfaces:**
- Consumes: 없음
- Produces: 없음 (문서만 갱신)

- [ ] **Step 1: 노드 목록에서 mode_manager 행 제거**

`## 2. 노드(모듈) 목록과 역할` 아래 "젯슨 — 커스텀 노드" 표에서 기존:
```markdown
| 노드 | 구현 | 역할 |
|------|------|------|
| `odom_publisher` | `src/odom_publisher.cpp` | 바퀴 RPM → 오도메트리 계산 |
| `mode_manager` | `src/mode_manager.cpp` | RC/AUTO/STOP에 따라 모터 명령 게이팅 |
```

변경 후:
```markdown
| 노드 | 구현 | 역할 |
|------|------|------|
| `odom_publisher` | `src/odom_publisher.cpp` | 바퀴 RPM → 오도메트리 계산 |
```

- [ ] **Step 2: 토픽 표에서 /cmd_vel, /cmd_vel_out 행 통합**

`## 3. 핵심 토픽` 표에서 기존:
```markdown
| `/cmd_vel` | Twist(비-stamped) | `controller_server` | `mode_manager` |
| `/cmd_vel_out` | Twist | `mode_manager` | OpenRB-150(agent 경유) |
```

변경 후:
```markdown
| `/cmd_vel_out` | Twist(비-stamped) | `controller_server` (remap: `cmd_vel`→`/cmd_vel_out`) | OpenRB-150(agent 경유) |
```

- [ ] **Step 3: §6 드라이브 모드 게이팅 재작성**

기존:
```markdown
## 6. 드라이브 모드 게이팅 (안전 스위치)

`mode_manager`가 `/cmd_vel`(Nav2) → `/cmd_vel_out`(모터) 경로를 제어:
- **RC** : Nav2 명령 무시(수동 조종). 매핑은 이 모드로 주행.
- **AUTO** : `/cmd_vel` → `/cmd_vel_out` 통과 (rviz2 목표점으로 자율주행).
- **STOP** : 즉시 정지.

→ Nav2가 폭주해도 조종기 스위치를 RC/STOP으로 넘기면 즉시 회수(데드맨). SLAM은
이 게이팅과 무관하게 항상 동작(odom·scan은 계속 흐름).
```

변경 후:
```markdown
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
```

- [ ] **Step 4: 커밋**

```bash
git add docs/system-overview.md
git commit -m "docs: reflect mode_manager removal (MCU-only gating)"
```

---

## Task 5: 빌드 검증 (Jetson, 컨테이너)

컨테이너 안에서 패키지가 `mode_manager` 없이 정상 빌드되는지 확인한다. 코드 변경이 아니라 검증이므로 커밋 없음.

**Files:** 없음

- [ ] **Step 1: 컨테이너 안에서 재빌드**

Run:
```bash
docker exec -it uel_rover bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --symlink-install --packages-select uel_rover_control"
```
Expected: `Summary: 1 package finished` — 에러 없음, `mode_manager` 관련 빌드 로그 없음

- [ ] **Step 2: 빌드 산출물에 mode_manager 실행 파일이 없는지 확인**

Run: `docker exec uel_rover bash -c "ls /ros2_ws/install/uel_rover_control/lib/uel_rover_control/"`
Expected: `odom_publisher`만 있고 `mode_manager`는 없음

---

## Task 6: 실기 통합 검증 (하드웨어 필요)

빌드를 넘어 실제 동작을 확인한다. OpenRB-150이 연결되어 있어야 한다. 코드 산출물이 아니라 검증 절차이므로 커밋 없음.

**Files:** 없음

- [ ] **Step 1: 컨테이너 재기동 후 노드 목록 확인**

Run:
```bash
docker compose restart uel_rover
sleep 8
docker exec uel_rover bash -c "source /opt/ros/jazzy/setup.bash && ros2 daemon stop && sleep 1 && ros2 daemon start && sleep 2 && ros2 node list"
```
Expected: `mode_manager` 없음, `controller_server`는 있음

- [ ] **Step 2: AUTO 모드에서 /cmd_vel_out 직접 발행 확인**

조종기를 **AUTO**로 두고 rviz2에서 목표점 지정 후:
Run: `docker exec uel_rover bash -c "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /cmd_vel_out"`
Expected: `controller_server`의 속도 명령이 직접 보임 (중계 노드 없이)

- [ ] **Step 3: RC 모드 회귀 확인 (안전 게이팅)**

조종기를 **RC**로 전환한 채 목표 주행을 시도:
Expected: 모터가 Nav2 명령을 무시하고 RC 입력만 따름 (MCU 게이팅 정상 — 회귀 없음)

- [ ] **Step 4: STOP 모드 회귀 확인 (데드맨 스위치)**

조종기를 **STOP**으로 전환:
Expected: 즉시 정지 (회귀 없음)

---

## Self-Review (작성자 점검 결과)

**스펙 커버리지:** §2(변경 사항) 5개 항목 전부 Task 1~4에 대응. §4(검증 체크리스트) 5개 항목 전부 Task 5~6에 대응. §3(영향 없음) 항목은 별도 태스크 불필요(변경하지 않는 것 자체가 확인 사항이며, Task 6 Step 3에서 펌웨어 게이팅 회귀 없음으로 실측 검증됨).

**Placeholder 스캔:** 없음. 모든 코드 스니펫이 실제 현재 파일 내용 기준의 완전한 before/after.

**타입/이름 일관성:** `controller_server`, `cmd_vel_out`, `odom_publisher` 등 명칭이 스펙·기존 코드와 동일. remap 문법(`remappings=[('cmd_vel', '/cmd_vel_out')]`)은 저장소 내 다른 launch 파일(`bringup_common.launch.py`의 `pointcloud_to_laserscan` remap)과 동일한 패턴 사용.
