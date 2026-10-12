# spacek_rover

spacek 스키드 조향 로버(다이나믹셀 좌우 2개, 바퀴 4개)의 ROS 2 **Jazzy** + **Gazebo Harmonic** 시뮬레이션 패키지입니다.
좌우 바퀴를 같은 방향으로 돌리면 전진/후진, 반대로 돌리면 선회(제자리 선회 포함)합니다. 같은 쪽 앞뒤 바퀴는 연결되어 있어
다이나믹셀 1개가 바퀴 2개를 같은 속도로 돌립니다.

설계: `ros2_ws/docs/superpowers/specs/2026-10-09-spacek-rover-gazebo-design.md` (1단계: 로버 단독 주행)

## 1. 구조

| 파일 | 역할 |
|---|---|
| `models/spacek_rover/model.sdf` | base_link + 바퀴 4개(revolute, 축 +y), DiffDrive(좌 2 / 우 2), JointStatePublisher, OdometryPublisher, `platform_mount` 프레임 |
| `models/spacek_rover/meshes/*.stl` | SolidWorks 에서 내보낸 비주얼 메시 (mm, 7절 참고) |
| `worlds/rover_test.sdf` | 평지(μ 1.0), DART 1 ms |
| `config/rover.yaml` | 질량 예산, 무게중심, 바퀴 치수/마찰, 모터(다이나믹셀) 사양, 가속 한계 |
| `config/bridge.yaml` | ROS ↔ gz 토픽 연결 |
| `config/drive_test.yaml` | 자동 주행 시험 단계와 판정 기준 |
| `spacek_rover/rover_params.py` | rover.yaml → 질량/관성/조인트 한계/DiffDrive 한계 계산 |
| `spacek_rover/sdf_tools.py` | launch 시 SDF 를 임시 폴더에 복사하고 계산값 반영, 메시/박스 비주얼 선택 |
| `spacek_rover/drive_eval.py`, `drive_test.py` | 자동 주행 시험 판정 로직과 노드 |
| `launch/rover_sim.launch.py` | 전체 기동 |

## 2. 좌표계

- base_link: x 전방, y 왼쪽, z 위. 원점 = 바퀴 축 4개의 중심(지면 위 88.25 mm).
- SolidWorks 마스터 `spacek rover.SLDPRT` (Y-up, **전방 = −X**) → base_link [mm]: `x = −(X − 34.8713)`, `y = Z + 9.6801`, `z = Y + 209.5403`.
- 바퀴 조인트: front_left (0.235, 0.18979), front_right (0.235, −0.18979), rear_left (−0.235, 0.18979), rear_right (−0.235, −0.18979) m.
- `platform_mount` (타공판 윗면 중심) = (0.034659, 0, 0.214540) m. 2단계에서 착륙 플랫폼을 여기에 고정합니다.

## 3. 파라미터 (`config/rover.yaml`)

| 항목 | 기본값 | 비고 |
|---|---|---|
| 총질량 / 바퀴 1개 | 20.0 kg / 1.0 kg | base = 16.0 kg. 실측하면 `mass` 만 바꾸면 관성까지 다시 계산됩니다 |
| base 무게중심 | (−0.00028, 0, 0.12578) m + `com_offset_m` | 배터리 위치 등은 오프셋으로 보정 |
| base 관성 | 박스 0.415 × 0.200 × 0.123 m 근사 | Ixx 0.0735, Iyy 0.2498, Izz 0.2830 kg·m² |
| 바퀴 | r 0.0877 (실기 오도메트리 보정, CAD 0.08825), 폭 0.0876 m, 트랙 0.3796 m, 축간 0.470 m | 충돌 = 원통 |
| 마찰 전후 / 측면 | 0.9 / 0.5 (지면 1.0) | `mu_longitudinal` / `mu_lateral`. SDF 에서는 `fdir1`(바퀴 축) 방향이 `mu` = 측면, `mu2` = 전후. 선회 여유 `turn_margin` 1.45 (> 1 이어야 제자리 선회) |
| 모터 | 모델 미상 (토크 XM540-W270 추정 10.6 N·m), 펌웨어 MAX_RPM 50 (바퀴) | 바퀴당 토크 상한 5.3 N·m, 바퀴 5.24 rad/s, 최고 0.459 m/s, 2.42 rad/s |
| 가속 한계 | 0.5 m/s², 2.0 rad/s² | DiffDrive |

## 4. 설치와 빌드 (리눅스 PC: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard python3-yaml python3-pytest
mkdir -p ~/ros2_ws/src
cp -r <이 패키지 폴더> ~/ros2_ws/src/spacek_rover
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select spacek_rover
source install/setup.bash
```

## 5. 실행

```bash
# GUI + 키보드 조종 (다른 터미널에서 teleop)
ros2 launch spacek_rover rover_sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rover/cmd_vel

# 자동 주행 시험 (헤드리스, 끝나면 종료) -> ~/rover_results/drive_test_result.json
ros2 launch spacek_rover rover_sim.launch.py drive_test:=true gui:=false shutdown_on_done:=true

# 마찰 방향이 헤딩과 무관한지: 스폰 yaw 를 바꿔 같은 시험 (결과 폴더를 나눠서)
ros2 launch spacek_rover rover_sim.launch.py drive_test:=true gui:=false shutdown_on_done:=true yaw:=0.785 csv_dir:=~/rover_results/yaw0785
ros2 launch spacek_rover rover_sim.launch.py drive_test:=true gui:=false shutdown_on_done:=true yaw:=2.5 csv_dir:=~/rover_results/yaw25
```

토픽: `/rover/cmd_vel`(입력), `/rover/odom`(바퀴 오도메트리), `/rover/odom_gt`(실제 위치), `/rover/joint_states`, `/tf`(odom → base_link).

로버 위 착륙 플랫폼 시뮬레이션(`drone_landing_platform` 의 `rover_landing_sim.launch.py`)에서는 `rover_watch` 노드가
`/rover/odom_gt` 로 착륙 중 로버가 밀리거나 기울었는지 기록합니다(이동 < 5 mm, 기울기 < 1°, yaw < 0.5°).
기준 자세: `start_topic` 파라미터(launch 는 `/drone/joint_states`)의 첫 메시지 순간의 자세, `start_topic` 이 비어 있으면 첫 odom 뒤 `settle_s`(2 s).
결과: `/rover/watch/result`, `<csv_dir>/<시나리오>_rover_watch.json`.

## 6. 검증 절차와 기대 결과

1. `colcon test --packages-select spacek_rover && colcon test-result --verbose` → 실패 0.
2. 자동 주행 시험 → `drive_test_result.json` 의 `success: true`. 단계별 기준:

| 단계 | 명령 | 통과 기준 (실제 위치) |
|---|---|---|
| settle | 0, 2 s | z = 0.0877 ± 0.005 m, roll/pitch < 1°, 바퀴 속도 < 0.05 rad/s |
| forward | 0.2 m/s, 5 s | 전진 0.96 m ± 10%, 헤딩 변화 < 2°, 측면 < 0.03 m |
| spin | 0.8 rad/s, 4 s | yaw 증가 ≥ 명령 yaw(3.04 rad)의 30% (≈ 52°, 반시계), 위치 이탈 < 0.10 m. `yaw_ratio_gt_over_odom` = 스키드 미끄럼 지표 |
| arc | 0.15 m/s, 0.3 rad/s, 4 s | yaw 증가 ≥ 명령 yaw 의 30%, 전진 ≥ 명령 거리의 30% |

`success` 만 보지 말고 단계별 `measured`(특히 spin 의 `yaw_change_deg`, `yaw_ratio_gt_over_odom`)와 `drive_test.csv` 도 확인하세요.
기준 비율은 `config/drive_test.yaml` 의 `min_motion_fraction` 입니다.
| stop | 0, 2 s | 마지막 0.5 s 속도 < 0.01 m/s |

3. GUI 에서 바퀴의 흰 표시(`*_marker`)가 돌아가는지, 메시가 충돌 형상과 겹쳐 보이는지 확인합니다.

실패 시: settle 의 z 가 낮으면 질량/접촉, forward 거리가 짧으면 토크 상한(`motor`)이나 `mu_longitudinal`,
spin 이 안 돌면(바퀴는 도는데 차체가 안 돎) `mu_lateral` 을 낮추세요(`turn_margin` 이 커짐).
yaw 에 따라 결과가 크게 다르면 마찰 방향(`fdir1`)이 바퀴 축을 따르지 않는 것이니 `mu`/`mu2` 를 바꿔 넣어 비교해 보세요.

## 7. SolidWorks 원본과 다시 만들기

- 마스터: `로버플랫폼프로토타입/spacek rover.SLDPRT` (수정하지 않음).
- `로버플랫폼프로토타입/로버_가제보/rover_base.SLDPRT`, `rover_wheel.SLDPRT` 는 마스터 **사본**을 `keep_inside` 컷으로 자른 것입니다.
  마스터와 연결되어 있지 않으므로 **마스터를 고치면 다시 만들어야 합니다**:
  1. 마스터를 두 이름으로 복사.
  2. base 사본: Top 평면에 사각형 (X −400..400, Z −155.6701..136.3199) 을 `keep_inside` 컷 → 바디 3개.
     (Z −155.6801 그대로 자르면 바퀴 축 끝이 닿는 면에 두께 0 조각 바디가 생겨서 0.01 mm 안쪽을 경계로 씁니다.)
  3. wheel 사본: Top 평면에 사각형 (X −400..34.8713, Z −400..−155.6901) 을 `keep_inside` 컷 → 바디 1개(front_right 바퀴).
  4. 두 파트를 STL(mm) 로 `models/spacek_rover/meshes/base.stl`, `wheel.stl` 에 내보냅니다.
     SolidWorks 는 확장자를 대문자 `.STL` 로 저장하므로 **소문자 `.stl` 로 이름을 바꿉니다** (리눅스는 대소문자를 구분,
     `test_mesh_uris_match_files_case_sensitively` 가 검사).
     **SolidWorks STL 내보내기는 형상을 "양의 공간"으로 평행이동합니다** (STL = 마스터 + 오프셋).
     현재 오프셋: base (266.8977, 233.6776, 155.6701) mm, wheel (288.3787, 297.7903, 243.2645) mm
     (= STL 바운딩 박스 최솟값 − SolidWorks 바디 박스 최솟값). 오프셋이 바뀌면 `model.sdf` 의 `*_mesh_visual` pose 를
     `R·(−오프셋 − 링크원점) / 1000` 으로 다시 계산합니다 (model.sdf 머리 주석 참고).
  5. `python3 -m pytest test/test_sdf_wellformed.py` 의 메시 정렬 시험(`test_base_mesh_aligned_with_link`,
     `test_wheel_mesh_centered_on_axle`)이 통과하는지 확인하고 `colcon build`.
- `rover_gazebo.SLDASM`: base + 바퀴 4개 배치 확인용 어셈블리 (base 에 축 원통 면이 없어 메이트 없이 배치만 함, `rover_gazebo_iso.png`).

## 8. 알려진 한계

1. 같은 쪽 바퀴 2개는 개별 조인트에 같은 속도 명령을 주고 토크 상한을 반씩 나눴습니다. 실기는 한 바퀴가 미끄러지면 다른 바퀴로 토크가 몰립니다.
2. 토크 상한이 일정합니다(다이나믹셀 토크-속도 곡선 미반영). DART 에서 속도 명령은 서보로 동작하고 SDF `effort` 가 힘 상한이 됩니다.
3. DiffDrive 오도메트리는 이상적 차동구동 기구학이라 스키드 선회에서 yaw 를 과대 추정합니다 → `/rover/odom_gt` 를 쓰세요.
4. 모터 사양(XM540-W270, 1:1)과 질량 배분은 추정값입니다. 실측 시 `rover.yaml` 만 바꾸면 됩니다.
5. 사이드 암은 섀시에 고정으로 가정했습니다(로커 아님). 암 충돌 박스는 정면 스크린숏 측정(±5 mm)입니다.
