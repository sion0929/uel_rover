# drone_landing_platform

드론 착륙 **포획 플랫폼**(로버 없음)의 ROS 2 **Jazzy** + **Gazebo Harmonic**(gz-sim 8, ros_gz) 시뮬레이션 패키지입니다.

드론이 착지해 모터를 끄면 무게로 **가동 바닥**이 서스펜션을 누르며 내려가고, 바닥 가장자리에 힌지된
**접이식 플레이트 4장**이 (끝단 핀이 프레임의 수평 가이드 z = 135 mm 를 따라 미끄러지므로) 대기 10.37° 에서
위로 접혀 올라오면서 드론 하부 프레임을 중앙/0° 로 밀어 넣습니다. 이륙하면 바닥이 복귀하고 플레이트가 다시 열립니다.

---

## 1. 구조

```
                 ┌────────────────────── Gazebo Harmonic (gz sim, DART, 1 ms) ──────────────────────┐
                 │  world: landing_test                                                             │
                 │  ┌─ model landing_platform ─────────────────┐   ┌─ model simple_drone ─────────┐ │
                 │  │ frame ══fixed══ world                    │   │ anchor ══fixed══ world        │ │
                 │  │   └ floor_joint (prismatic -Z, k,c,예압)  │   │  └ drone_x ─ drone_y ─ drone_z │ │
                 │  │       └ floor                             │   │     └ yaw ─ pitch ─ roll ─ body│ │
                 │  │          ├ plate_px_joint ┐ (revolute)    │   │  ApplyJointForce x6           │ │
                 │  │          ├ plate_mx_joint │ JointPosition │   │  JointStatePublisher          │ │
                 │  │          ├ plate_py_joint │ Controller x4 │   └───────────────────────────────┘ │
                 │  │          └ plate_my_joint ┘               │                                     │
                 │  │ JointStatePublisher                       │                                     │
                 │  └───────────────────────────────────────────┘                                     │
                 └───────▲───────────────────┬──────────────────────────────▲───────────────┬─────────┘
       .../0/cmd_pos     │ gz.msgs.Double    │ gz.msgs.Model (joint_state)  │ cmd_force     │ joint_state
                 ┌───────┴───────────────────▼──────────────────────────────┴───────────────▼─────────┐
                 │                         ros_gz_bridge (config/bridge.yaml)  + /clock               │
                 └───────▲───────────────────┬──────────────────────────────▲───────────────┬─────────┘
 /platform/<j>/cmd_pos   │        /joint_states (d, θ)         /model/simple_drone/joint/  │ /drone/joint_states
              ┌──────────┴─────────┐         │                    <j>/cmd_force          │
              │   plate_coupler    │◄────────┤            ┌───────────┴──────────────────▼──┐
              │ θ(d) 법칙 / 테이블 │         └───────────►│       landing_controller         │──► /landing/state
              └────────────────────┘                      │ APPROACH→TOUCHDOWN→CAPTURED→     │──► /landing/result (JSON)
                                                          │ TAKEOFF→CLIMB→DONE , CSV 기록     │──► ~/landing_results/*.csv
                                                          └──────────────────────────────────┘
```

| 파일 | 역할 |
|---|---|
| `models/landing_platform/model.sdf` | 프레임/바닥/플레이트 링크, 조인트, 서스펜션, JointPositionController ×4, JointStatePublisher |
| `models/simple_drone/model.sdf` | 1.9 kg 드론, 6자유도 가상 조인트 + ApplyJointForce ×6 |
| `worlds/landing_test.sdf` | 지면, 태양, DART 1 ms, Physics/UserCommands/SceneBroadcaster/Contact/ApplyLinkWrench, 플랫폼 include |
| `drone_landing_platform/coupling.py` | θ(d) 법칙과 룩업 테이블 (ROS 비의존, 단위시험 대상) |
| `drone_landing_platform/plate_coupler.py` | `/joint_states` 의 바닥 침하량 → 플레이트 4개 위치 명령 |
| `drone_landing_platform/landing_controller.py` | 착륙 상태기계, 결과 판정, CSV/JSON 기록 |
| `drone_landing_platform/sdf_tools.py` | launch 시 SDF 복사·수정 (k, c, 예압, 스톱, PID, 드론 질량, 메시/박스 비주얼 선택) |
| `launch/landing_sim.launch.py` | 전체 기동 (`scenario`, `mode`, `gui`, `visual`, `damping` …) |
| `scripts/run_all.sh` | 4개 시나리오 헤드리스 연속 실행 + 결과 요약 |
| `config/*.yaml` | platform(물성), scenarios, coupling, controller, bridge |
| `drone_landing_platform/rover_mount.py` | 로버(spacek_rover) + 플랫폼 SDF 를 모델 하나(`rover_platform`)로 합침 (13장) |
| `worlds/rover_landing.sdf`, `config/rover_mount.yaml`, `config/bridge_rover.yaml`, `launch/rover_landing_sim.launch.py` | 로버 위 플랫폼 시뮬레이션 |

---

## 2. 기구 ↔ Gazebo 조인트 매핑

좌표계: **플랫폼 좌표계** (Z 위, 원점 = 베이스 플레이트 윗면 중심, SDF 는 m 단위). 월드 원점 = 플랫폼 원점.
모든 플랫폼 링크 좌표계는 **대기 자세의 플랫폼 원점**에 있습니다(링크 pose = 0). 조인트 pose 만 힌지/축 위치에 둡니다.

| 실기 | Gazebo | 비고 |
|---|---|---|
| 고정 프레임 (베이스 280×280×5, 코너 블록 4, 타워 벽 8, 코너 데크 4) | 링크 `frame`, `frame_fixed`(world 와 fixed) | 충돌 = 박스. 핀 가이드(z=135)는 충돌 없음 |
| 가동 바닥 200×200×15 (0.486 kg), 대기 윗면 z = 121.1 | 링크 `floor` + `floor_joint` **prismatic, 축 (0,0,-1)** | 조인트 위치 q = 침하량 d (아래 +). 한계 0 (대기) ~ 0.0606 m (하단 스톱) |
| 쇼바 2개 (RC #180007, 경사 장착) | `floor_joint` 의 `<spring_stiffness>`, `<spring_reference>`, `<damping>` | 수직 등가 k = 250 N/m, 예압 8.1 N, c = 40 N·s/m |
| 플레이트 4장 (0.176 kg, 113.25×200×6 박스) | 링크 `plate_px/mx/py/my` + `plate_*_joint` **revolute** (부모 = floor) | 조인트 위치 = **θ − θ0** (대기 = 0). 한계 −10.37° ~ +84.63° (= 절대각 0~95°) |
| 힌지 (plate_px): x = 107.2, z = 115.7 mm, 축 ∥ Y | `plate_px_joint` pose `0.1072 0 0.1157`, 축 `0 -1 0` | +각 = 바깥 끝이 올라감. mx: 축 `0 1 0`, py: `1 0 0`, my: `-1 0 0` |
| 끝단 핀이 z = 135 가이드를 따라 미끄러짐 (기구학 구속) | **JointPositionController** ×4 + `plate_coupler` 노드 | sin θ = (19.3 + d) / 107.25 |

커플링: 핀 높이 135.0 은 고정, 힌지는 바닥과 함께 d 만큼 내려가므로

```
sin θ = (h0 + d) / L,   h0 = 135.0 − 115.7 = 19.3 mm,   L = 107.25 mm
θ(0) = 10.37°,   θ(60.6 mm, 하단 스톱) = 48.16° (상대 37.79°),   θ = 90° 는 d = 87.95 mm (특이점, 도달 불가)
```

### spring_reference 부호와 예압 계산

SDF/DART 의 스프링 힘(조인트 축 방향 일반화 힘) = `−k (q − spring_reference)` 입니다.
`floor_joint` 축이 −Z 이므로 q = d 이고, 축의 + 방향 = 아래쪽입니다. d = 0 에서 **위로** F 를 내려면
`−k (0 − q_ref) = −F` → **`spring_reference = −F / k`** (음수: 자유 길이가 대기 위치보다 위에 있음).

Gazebo 에는 실기의 핀 가이드(프레임이 플레이트 무게 일부를 받침)가 없어 플레이트 4장 무게가 전부 바닥에 걸립니다.
SolidWorks 에서 바닥 좌표에 걸리는 유효 하중은 8.4 N (바닥 4.77 N + 플레이트 4 × 0.90 N)이므로,
가이드가 받치는 몫 `4 × (0.176·g − 0.90) = 3.30 N` 을 예압에 더해 같은 정적 거동을 만듭니다.

```
F  = 8.1 + 3.30 = 11.40 N   →   spring_reference = −11.40 / 250 = −0.0456 m
드론 없음: 순하중 11.67 − 11.40 = 0.27 N 아래 → d ≈ 1.1 mm (SolidWorks: 이륙 후 ≈ 1.5 mm 로 복귀)
드론 1.9 kg: 정적 침하 (11.67 + 18.63 − 11.40)/250 = 75.6 mm > 60.6 mm → 하단 스톱에 3.8 N 으로 안착, θ = 48.2°
```

(`config/platform.yaml` 의 `plate_guide_support_n` 으로 조정. launch 가 자동 계산해 SDF 에 넣고 로그에 출력합니다.)

---

## 3. 파라미터 표 (기본값 = SolidWorks Motion 모델과 동일)

| 항목 | 값 | 위치 |
|---|---|---|
| 프레임 윗면 / 바닥 윗면(대기) | z = 144.0 / 121.1 mm | `platform.yaml: geometry` |
| 힌지 (px, 대기) / 끝단 핀 (대기) | (107.2, 115.7) / (212.7, 135.0) mm | `model.sdf`, `coupling.yaml: law` |
| 힌지-핀 거리 L, 높이차 h0, 대기각 θ0 | 107.25 mm, 19.3 mm, 10.367° | `coupling.yaml` |
| 서스펜션 강성 k (2 쇼바 등가 수직) | 250 N/m | `platform.yaml: suspension.stiffness_n_per_m` / `spring_k:=` |
| 대기 예압 | 8.1 N (+ 가이드 보정 3.30 N) | `preload_n` / `preload:=` |
| 감쇠 c | 40 N·s/m | `damping_ns_per_m` / `damping:=` |
| 하단 스톱 | 60.6 mm (쇼바_경사스톱, 이때 각 쇼바 ≈ 27.5 mm 압축 < 37 mm 스트로크) | `hard_stop_m` |
| 기구 최대 침하 (참고) | 84 mm (θ = 74.4°) — 사용 안 함 | 주석 |
| 쇼바 실물 | RC #180007 ×2, 알루미늄, 128 mm(eye-eye)/120 mm, 스트로크 37 mm, Ø16, 구멍 3 mm, 50 g | 참고 |
| 바닥 / 플레이트 질량 | 0.486 kg / 0.176 kg ×4 | `model.sdf` |
| 드론 질량 | 1.9 kg (기체 1.674 + 착륙 프레임 proxy 0.226) | `platform.yaml: drone.mass_kg` / `drone_mass:=` |
| 드론 하부 프레임 (충돌) | 184 × 183 × 160 mm, 기어 바닥 = 원점 −164.4 mm | `simple_drone/model.sdf` |
| 모터 추력점 | (±132.9, ±132.9, +4.4) mm | 비주얼/참고 |
| 드론 관성 (박스 근사) | Ixx = Iyy = 0.01203, Izz = 0.01956 kg·m², CoM z = −9.1 mm | `model.sdf`, `controller.yaml` |
| 플레이트 서보 PID | P 30 N·m/rad, I 0, D 0.3 N·m·s/rad, 최대 3 N·m | `platform.yaml: plate_controller` |
| 접촉 마찰 μ | 드론 0.3, 바닥 0.5, 플레이트 0.2 (DART 는 작은 값 사용) | `model.sdf` |
| 물리 | DART, 1 ms, RTF 1 | `landing_test.sdf` |
| 하강 속도 / 접근 고도 | 0.3 m/s / 기어 바닥이 프레임 위 0.5 m | `controller.yaml` |
| 이륙 추력 (closed_loop) | 1.5 × W, 프레임 위 0.1 m 까지 수평/요 제어 OFF | `takeoff_thrust_ratio` |
| SolidWorks 추력 스케줄 (sw_openloop) | 0.85 W (0~0.30 s) → 0 (0.45 s) → 이륙 램프 2.0~2.2 s → 1.10 W | `ol_*` (초기 안내값은 1.25 W, 결과 보고값 1.10 W 사용) |
| SolidWorks 시작 높이 | 기어 바닥이 프레임 윗면 위 20 mm | `spawn_gap_openloop_m` |
| 판정 허용치 | \|x\|,\|y\| ≤ 20 mm, yaw(90° 배수 기준) ≤ 12° | `pos_tol`, `yaw_tol_deg` (아래 "한계" 참고) |

SolidWorks Motion 결과 (center, 비교 기준): 착지 0.25 s, 바닥 스톱 도달 ≈ 0.6 s, 최종 x = −1.1 mm, yaw = −0.3°,
이륙 후 1 s 이내 바닥 ≈ 1.5 mm 로 복귀. 결과 JSON 의 `touchdown_time_s`, `floor_stop_time_s`, `capture`, `floor_sink_now_mm` 와 비교하세요
(`mode:=sw_openloop` 이 같은 추력 스케줄/시작 높이를 사용합니다).

---

## 4. 비행/제어 구현

### 왜 ApplyLinkWrench 가 아니라 ApplyJointForce 인가
Harmonic 의 `ApplyLinkWrench` persistent 토픽(`/world/<w>/wrench/persistent`)은 받은 렌치를 **목록에 계속 추가**하고,
`/wrench/clear` 는 다음 스텝에 그 엔티티의 persistent 렌치를 **전부** 지웁니다. 매 제어 주기마다 렌치를 바꾸면
힘이 누적되거나(clear 없이), clear 와 새 렌치가 같은 스텝에 도착해 새 렌치까지 지워지는 경합이 생깁니다.
1회성 토픽(`/world/<w>/wrench`)은 1 스텝(1 ms)만 적용되어 1 kHz 로 끊김 없이 보내야 합니다.
그래서 드론을 **월드 =fixed= anchor → 직동 X → 직동 Y → 직동 Z → 회전 yaw → pitch → roll → body** 의
6자유도 가상 조인트 체인으로 만들고, 각 조인트에 `gz::sim::systems::ApplyJointForce`(마지막 명령을 매 스텝 유지)를 붙였습니다.
조인트 마찰/감쇠/스프링이 0 이고 중간 링크가 5 g 더미이므로 **힘 0 = 자유 강체**(플레이트가 x, y, yaw 로 밀 수 있음)입니다.
추력은 월드 Z 방향 일반화 힘으로 가합니다(자세를 수평으로 유지하는 구간에서는 실제 추력과 동등, 모터 OFF 구간은 차이 없음).
`OdometryPublisher` 는 이 구조에서 모델 프레임(고정 앵커)을 내보내므로 쓰지 않고, 상태는 조인트 상태(정확한 위치/속도)로 받습니다.

### 상태기계 (`mode:=closed_loop`)
| 상태 | 동작 | 전이 조건 |
|---|---|---|
| APPROACH | 공중(기어 바닥 = 프레임 위 0.5 m)에서 시나리오 x, y, yaw 로 정렬(≥ 2 s) 후 0.3 m/s 하강. x/y/z PD, yaw PD, roll/pitch 수평 유지 | **착지 감지**: (기어 바닥 − 바닥 윗면) < 12 mm **그리고** \|드론 vz − 바닥 vz\| < 0.08 m/s 가 0.15 s 지속 |
| TOUCHDOWN | 모든 힘/토크를 0.05 s 에 0 으로 (모터 OFF, 수평/요/자세 제어 OFF) | 바닥 속도 < 5 mm/s 가 0.3 s 지속 (최대 3 s) |
| CAPTURED | 힘 0 유지 3 s → x, y, yaw, 침하량, 플레이트 각 기록 (`/landing/result`) | 3 s 경과 |
| TAKEOFF | 추력 1.5 W, roll/pitch 만 수평 유지, x/y/yaw 제어 OFF | 기어 바닥 > 프레임 위 0.1 m (시간 초과 5 s) |
| CLIMB | 현재 x, y, yaw 유지, 0.5 m/s 로 기어 바닥 = 프레임 위 0.6 m 까지 상승 | 고도 오차 < 2 cm, 1.5 s 유지 |
| DONE | 호버 유지, 최종 결과 발행 (`shutdown_on_done:=true` 이면 1 s 후 종료) | — |

상대 수직속도를 쓰므로 바닥이 함께 내려가는 중(스프링이 눌리는 중)에도 "바닥 위에 얹힘" 을 바로 감지합니다.
게인은 가속도 단위(kp_xy 16, kd_xy 8 → ω = 4 rad/s, ζ = 1; kp_z 25, kd_z 10; yaw 25/10; roll·pitch 100/20)이며
질량/관성을 곱해 N, N·m 로 바꿉니다. 제어 주기 200 Hz(시뮬레이션 시간).

`mode:=sw_openloop` 은 위 SolidWorks 추력 스케줄을 시간 함수로 그대로 가하고(자세 제어 없음), 이륙 후 프레임 위 0.1 m 를
지나면 CLIMB(폐루프)으로 넘어갑니다. 포획 결과는 1.95 s(이륙 직전)에 기록합니다.

---

## 5. 설치 (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)

```bash
# ROS 2 Jazzy (이미 설치했다면 생략) - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Gazebo Harmonic + ros_gz (Jazzy 의 공식 짝)
sudo apt install -y ros-jazzy-ros-gz python3-yaml python3-pytest

# 빌드
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws            # src/drone_landing_platform, src/spacek_rover 를 복사해 둔 워크스페이스
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select spacek_rover drone_landing_platform
# (고정 플랫폼만 쓸 때는 drone_landing_platform 만 빌드해도 됨. rover:=true 는 spacek_rover 필요)
source install/setup.bash
```

## 6. Windows 에서 WSL2 로 돌리기

WSL2 는 직접 설치해야 합니다(관리자 PowerShell). 이후 Ubuntu 안에서 5장과 동일하게 진행합니다.

```powershell
wsl --install -d Ubuntu-24.04      # 재부팅 후 Ubuntu 사용자 생성
wsl --update                       # WSLg(GUI) 포함 최신 커널
```

```bash
# Ubuntu-24.04 (WSL) 안에서: Windows 바탕화면의 소스를 워크스페이스로 복사
mkdir -p ~/ros2_ws/src
cp -r "/mnt/c/Users/sion0/Desktop/로버플랫폼프로토타입/ros2_ws/src/drone_landing_platform" ~/ros2_ws/src/
cp -r "/mnt/c/Users/sion0/Desktop/로버플랫폼프로토타입/ros2_ws/src/spacek_rover" ~/ros2_ws/src/
# (5장의 설치/빌드 명령 실행)
export GZ_IP=127.0.0.1             # WSL 에서 gz-transport 탐색 문제 예방 (~/.bashrc 에 추가 권장)
# GUI 가 느리거나 검은 화면이면: export LIBGL_ALWAYS_SOFTWARE=1  또는 gui:=false (헤드리스)
```
`/mnt/c` 위에서 직접 빌드하면 매우 느리므로 반드시 리눅스 홈(`~/ros2_ws`)으로 복사해서 빌드하세요.

---

## 7. 실행

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

ros2 launch drone_landing_platform landing_sim.launch.py scenario:=center   # 중앙, yaw 0
ros2 launch drone_landing_platform landing_sim.launch.py scenario:=x30      # x = +30 mm
ros2 launch drone_landing_platform landing_sim.launch.py scenario:=x60      # x = +60 mm
ros2 launch drone_landing_platform landing_sim.launch.py scenario:=yaw30    # yaw 30°

# 헤드리스 (gz sim -s -r), 끝나면 자동 종료
ros2 launch drone_landing_platform landing_sim.launch.py scenario:=x60 gui:=false shutdown_on_done:=true

# SolidWorks Motion 과 같은 추력 스케줄/시작 높이
ros2 launch drone_landing_platform landing_sim.launch.py mode:=sw_openloop

# 4개 시나리오 연속 실행 (헤드리스) → ~/landing_results/
bash ~/ros2_ws/src/drone_landing_platform/scripts/run_all.sh
MODE=sw_openloop bash ~/ros2_ws/src/drone_landing_platform/scripts/run_all.sh
```

모니터링:
```bash
ros2 topic echo /landing/state
ros2 topic echo /landing/result        # JSON (포획 직후 1회 + 최종 1회)
ros2 topic echo /joint_states          # floor_joint = 침하량 [m], plate_* = θ − θ0 [rad]
```

결과 파일 (`csv_dir`, 기본 `~/landing_results`):
* `<scenario>.csv` (sw_openloop 은 `<scenario>_sw_openloop.csv`), 50 Hz:
  `t_s, state, x_mm, y_mm, z_mm, gear_z_mm, yaw_deg, pitch_deg, roll_deg, floor_sink_mm, theta_px_deg, theta_mx_deg, theta_py_deg, theta_my_deg, fz_cmd_n`
  (θ 는 절대각, t_s 는 드론 첫 상태 수신 시각 기준)
* `<scenario>_result.json`: 착지/스톱 도달 시각, 포획 x/y/yaw/침하량/플레이트 각, 이륙 성공 여부, `success`

---

## 8. 튜닝

* **스프링/감쇠/예압**: `config/platform.yaml` 의 `suspension` 을 고치거나 launch 인자로 즉시 바꿉니다.
  `ros2 launch drone_landing_platform landing_sim.launch.py damping:=25.0 spring_k:=300.0 preload:=9.0`
  (패키지를 다시 빌드할 필요 없음: launch 가 SDF 를 임시 폴더에 복사해 값만 바꿔 씁니다. YAML 을 고쳤다면 `colcon build` 후 실행.)
  감쇠비 ζ ≈ c / (2√(k·m)), m ≈ 바닥+플레이트+드론 3.09 kg → 기본 ζ ≈ 0.72.
* **플레이트 서보**: `plate_controller.p_gain/d_gain/cmd_max`. 플레이트 힌지 관성 ≈ 7.0e-4 kg·m² 이므로
  P 30 → ω ≈ 208 rad/s, 임계감쇠 D ≈ 0.29. 떨리면 P 를 낮추고 D 를 P 에 맞춰 2√(P·I) 근처로, 드론을 못 밀면 `cmd_max` 를 올립니다.
* **드론 제어**: `config/controller.yaml` 의 `kp_*`, `kd_*`(가속도 단위). 하강 속도 `descent_speed`, 착지 감지
  `touchdown_gap_tol / touchdown_vz_tol / touchdown_confirm_time`, 이륙 추력 `takeoff_thrust_ratio`.
* **드론 질량**: `drone_mass:=2.2` (SDF 관성 비례 스케일 + 제어기 피드포워드에 동시에 반영).

## 9. 커플링 테이블을 SolidWorks 측정값으로 교체

`config/coupling.yaml`:
```yaml
plate_coupler:
  ros__parameters:
    mode: table                 # law -> table
    table:
      d_mm:      [0.0, 5.0, 10.0, ...]      # 바닥 침하량 (엄격히 증가)
      theta_deg: [10.37, 13.1, 15.9, ...]   # 플레이트 절대각 (수평 기준)
    plate_zero_angle_deg: 10.367            # SDF 플레이트 형상의 각도(=조인트 0) - 바꾸지 마세요
```
선형 보간, 범위 밖은 양 끝 값으로 클램프합니다. 숫자는 `10.0` 처럼 소수점으로 쓰는 것을 권장합니다.
수정 후 `colcon build --packages-select drone_landing_platform` (또는 `--symlink-install` 로 빌드해 두면 재빌드 불필요).
`colcon test` 의 `test_coupling.py` 가 기본 테이블과 법칙의 일치/단조성을 검사하므로, 측정값으로 바꾸면
`test_default_yaml_table_matches_law` 는 실패할 수 있습니다(의도된 경고 — 측정값과 법칙의 차이를 확인하는 용도).

## 10. 메시 (STL) 내보내기

| 파일 | 경로 | 좌표계 |
|---|---|---|
| `frame.stl`, `floor.stl`, `plate_px.stl`, `plate_mx.stl`, `plate_py.stl`, `plate_my.stl` | `models/landing_platform/meshes/` | **플랫폼 좌표계 그대로**, 대기 자세(바닥 d = 0, 플레이트 10.37°), mm, Z 위, 이동 없음 |
| `drone.stl` | `models/simple_drone/meshes/` | 드론 좌표계: 원점 = 기체 중심, 기어 바닥 z = −164.4 mm, yaw 0 (184 mm 변 ∥ X), mm |

SDF 비주얼은 `model://.../meshes/<name>.stl`, `<scale>0.001 0.001 0.001</scale>`, pose 항등입니다(링크 좌표계가 위 좌표계와 같음).
`visual:=auto`(기본)는 STL 이 모두 있으면 메시 비주얼만, 없으면 박스 비주얼만 사용합니다. 충돌은 항상 박스입니다.
STL 추가 후 재빌드하세요.

## 11. 테스트

```bash
cd ~/ros2_ws && colcon test --packages-select spacek_rover drone_landing_platform && colcon test-result --verbose
# 또는 패키지 폴더에서:  python3 -m pytest test/test_coupling.py test/test_sdf_wellformed.py test/test_rover_mount.py -v
# (test_rover_mount.py 는 옆 폴더 ../spacek_rover 또는 설치된 spacek_rover 의 모델을 읽음)
```

---

## 12. 알려진 한계 / 주의

1. **포획 정밀도의 기하학적 한계**: 하단 스톱(60.6 mm)에서 플레이트는 48.2° 까지만 섭니다. 바닥 윗면 높이에서 플레이트 안쪽 면은
   중심에서 약 108 mm (힌지 107.2 mm, 판 두께 6 mm 를 힌지-핀 선 중앙에 둔다고 가정) → 드론 반폭 92 / 91.5 mm 와 비교해
   **x, y 는 약 ±16 mm, yaw 는 약 ±11° 까지 밀어 줄 수 없습니다**. 따라서 x30/x60 은 x ≈ +16 mm, yaw30 은 ≈ 11° 근처에서
   멈출 가능성이 큽니다. 기본 허용치(20 mm, 12°)는 이 기하 한계에 맞춘 값이며, 더 엄격한 정렬이 필요하면 플레이트가 더 서도록
   (스톱/링크 비) 하거나 플레이트 안쪽에 가이드 립을 두는 등 설계 변경이 필요합니다. 판 두께가 힌지-핀 선 위쪽에 있으면 한계는 더 작아집니다.
2. **커플링은 서보로 근사**: 실기는 핀-가이드 구속으로 바닥 하강 에너지가 플레이트를 밀지만, Gazebo 는 바닥 위치에 맞춰 플레이트를
   위치 서보(JointPositionController)로 움직입니다. 플레이트가 드론에 막혀도 바닥 하강이 늦춰지지 않습니다(바닥↔플레이트 반력 피드백 없음).
   핀 가이드의 수직 지지는 일정한 예압 보정(3.30 N)으로 대체했습니다(마찰 없는 수평 가이드 + 판 중앙 무게중심이면 각도와 무관).
3. 드론은 6자유도 가상 조인트 체인입니다. 스폰 시 yaw 는 항상 0 이고(직동축 정렬), 시나리오 yaw 는 공중 정렬 단계에서 맞춥니다.
   추력은 월드 Z 방향 힘이라 기체가 기울어도 수평 성분이 생기지 않습니다(모터 OFF 구간에는 영향 없음).
4. 쇼바 2개(경사 장착)를 수직 등가 스프링 1개로 표현했으며 경사에 따른 비선형 강성은 반영하지 않습니다.
5. 기구 최대 84 mm 에서 법칙은 θ = 74.4° 입니다(asin(103.3/107.25)). 안내받은 "≈76°" 와 1.6° 차이가 있으니 SolidWorks 값으로 확인하세요.
   실제 운용 범위는 스톱 60.6 mm 까지입니다.
6. 정적 검증만 된 상태입니다(이 PC 에는 ROS/Gazebo 가 없음). 처음 실행 시 확인할 것:
   * `JointStatePublisher` 의 `<update_rate>`(200 Hz) 는 버전에 따라 무시될 수 있습니다(그 경우 1 kHz 로 발행, 동작에는 문제 없음).
     `control_rate_hz` / `publish_rate_hz` 를 바꾸면 이 값도 같거나 정수배로 맞추세요(예: 250 Hz 발행 + 200 Hz 제어 → 실제 125 Hz).
   * 브리지 ROS 토픽 이름은 숫자 토큰을 허용하지 않아 플레이트 명령을 `/platform/<joint>/cmd_pos` ↔ gz `/model/landing_platform/joint/<joint>/0/cmd_pos` 로 연결합니다.
   * DART 조인트 한계(하단 스톱)는 구속식이므로 충돌 시 약간 튈 수 있습니다. 스톱 충격을 보려면 CSV 의 `floor_sink_mm` 를 확인하세요.
   * 접촉 마찰은 DART 가 두 면 중 작은 μ 를 사용합니다.

---

## 13. 로버 위 플랫폼 (2단계: 정지 로버)

바퀴 달린 로버(`spacek_rover` 패키지, 다이나믹셀 스키드 조향) 위에 이 플랫폼(REV-E4)을 고정하고 같은 4개 시나리오를 실행합니다.
설계: `ros2_ws/docs/superpowers/specs/2026-10-09-rover-platform-mount-design.md`

```bash
ros2 launch drone_landing_platform rover_landing_sim.launch.py scenario:=center gui:=false shutdown_on_done:=true
# 나머지 인자는 landing_sim.launch.py 와 같음 (= landing_sim.launch.py rover:=true)
```

| 항목 | 값 |
|---|---|
| 장착 | 플랫폼 원점(베이스판 윗면 중심) = 로버 base_link (0, 0, 0.21954) m, yaw 180° (플랫폼 +X = 로버 후방). 베이스판 바닥 = 로버 타공판 윗면 (`config/rover_mount.yaml`) |
| 월드 | `rover_landing`: 지면 z = −0.30724 m 로 내려 "월드 원점 = 플랫폼 원점" 유지 → 제어기/커플러/시나리오 변경 없음 |
| 모델 | launch 가 로버·플랫폼 runtime SDF 를 `rover_platform` 하나로 합침. 플랫폼 `frame_fixed`(world) → `platform_fixed`(base_link) |
| 프레임 질량 | 1.15 kg, CoM z 0.0446, Ixx = Iyy 0.0188, Izz 0.0323 (출력 STL 부피 × PLA 100% 추정, `platform.yaml` 의 `frame`) |
| 플랫폼 전체 | ≈ 2.34 kg (프레임 1.15 + 바닥 0.486 + 접이판 4 × 0.176) |
| 순서 | gz 기동 → `spawn_delay`(3 s) 에 로버+플랫폼 스폰 → `rover_settle_s`(2 s) 뒤 드론 스폰 |
| 감시 | `rover_watch`: 드론 스폰 순간(`/drone/joint_states` 첫 메시지)의 로버 자세를 기준으로 이동 < 5 mm, 기울기 < 1°, yaw < 0.5° → `<csv_dir>/<시나리오>_rover_watch.json`. `sw_openloop` 처럼 스폰 직후 착지해도 충격을 놓치지 않음 |

장착 x 위치: 원래 CAD(`00_플랫폼_대기12도_REV-E4.SLDASM` 안의 `spacek rover-1`)는 플랫폼 원점이 바퀴 축 중심보다 **12.25 mm 앞**
(base_link x +0.01225, y −0.00037)이지만, 2단계는 바퀴 축 중심 위(x = 0)로 정했습니다. CAD 와 맞추려면 `rover_mount.yaml` 의 `x_m: 0.01225`.

검증 (리눅스 PC):
1. `colcon build --packages-select spacek_rover drone_landing_platform && colcon test --packages-select spacek_rover drone_landing_platform && colcon test-result --verbose`
2. 시나리오 4개(center, x30, x60, yaw30)를 위 명령으로 실행 → 각 `<시나리오>_result.json` 의 `success: true`, `<시나리오>_rover_watch.json` 의 `passed: true`.
3. 같은 시나리오를 고정 플랫폼(`landing_sim.launch.py`, 결과 폴더를 나눠서)으로 돌려 착지 시각·바닥 스톱 시각·포획 x/y/yaw 를 비교.

SolidWorks 장착 확인 (`로버플랫폼프로토타입/로버_가제보/rover_platform_check.SLDASM`, `_iso.png`, `_side.png`):
바퀴 포함 로버(`rover_gazebo.SLDASM`) + E4 고정프레임·이동부를 위 장착값으로 배치 → 간섭 0, 베이스판↔타공판 면 접촉,
뒤쪽 레버 윗면 → 타워판 26.4 mm, 노브 → 베이스판 72.8 mm, 플랫폼 최고점 지면 위 452.4 mm.
