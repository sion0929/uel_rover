# SLAM 맵 겹침/전면-occupied 문제 — 디버깅 핸드오프

작성: 2026-07-24 · 방식: systematic-debugging (근본원인 확정 전, 증거 수집 단계)

## 증상
- AUTO 모드에서 rviz2는 정상 실행/연결됨.
- SLAM 맵은 확장되고 로버도 맵 위에서 움직임.
- 그런데 로버가 움직이면 새 스캔이 실제 위치와 어긋나 **겹쳐 찍히고**, 결국 전 영역이 occupied("다 막힌 맵")로 채워짐.

## 오판 금지 (원인 아님)
- rviz2 GLSL 에러 `active samplers ... same texture image unit` = **양성 경고**.
  Tegra/Jetson GLSL120 드라이버 + Ogre `indexed_8bit_image` 셰이더(OccupancyGrid 렌더용)에서 상시 발생.
  렌더링·맵 표시에 영향 없음. 맵 문제와 무관. 무시.

## 파이프라인 상태 (확인 완료 — 연결은 정상)
- MCU `/openrb150` → `/wheel_velocity` (`std_msgs/Float32MultiArray [vL,vR] rpm`) 발행. 정지 시 `[0.0, -0.0]`, 부호 있음(방향 정보 존재).
- `odom_publisher`(src/odom_publisher.cpp): 바퀴 rpm 적분 → `odom→base_link` TF + `/odom`.
- slam_toolbox: `/scan` + `odom→base_link` → `map→odom` TF, `/map`.
- `/scan` 출처: D455 depth → `pointcloud_to_laserscan` (target_frame=base_link, FOV ±0.76rad ~87° 전방 콘).
- 프레임 일관성 OK: odom·slam·urdf 모두 `base_link` 사용(과거 base_footprint 혼선은 해소됨).
- rviz Fixed Frame = `map` (정상).
- 즉 "얼마나 움직였나" 경로는 살아있음. 문제는 **값의 정확도** = 포즈 정합(pose registration) 실패.

## 유력 근본원인 (우선순위)
### (A) 오도메트리 스케일/기하가 실측이 아니라 추정값 ← 1순위
- `urdf/uel_rover.urdf` + `config/rover_params.yaml`: `wheel_radius=0.065`, `wheel_base=0.30`.
- URDF 주석에 명시: **"※ 아래 치수/카메라 위치는 실측 후 수정"** (urdf/uel_rover.urdf:11) → 미측정 placeholder.
- radius 오차 → 직진 거리 스케일 전체 틀어짐(스캔이 앞으로 덜/더 밀려 벽 겹침).
- wheel_base 오차 → 회전각 틀어짐(맵이 회전 방향으로 찢어지며 겹침).

### (B) 전방 좁은 FOV라 scan matching이 odom 오차를 못 잡음 ← 2순위(증폭 요인)
- slam_toolbox(Karto)는 360° 라이다 전제. ~87° 전방 콘으로는 회전·횡방향 구속 거의 못 함 → 사실상 odom을 그대로 신뢰 → (A) 오차가 교정 없이 맵에 그대로 드러남.

## 다음 작업: fix 전 반드시 실행할 확정 테스트 (추측 금지)
bringup/AUTO 띄운 상태에서:
```bash
# 맵 기준 실제 포즈
ros2 run tf2_ros tf2_echo map base_link
# 또는 odom만
ros2 topic echo /odom --field pose.pose.position
```
- **테스트1 (직진 스케일):** 정확히 1.0 m 직진 → odom x 증가분 확인.
  - ≠1.0 이면: `실제_radius = 0.065 × (실제이동 / odom보고값)`
- **테스트2 (회전 스케일):** 제자리 정확히 360°(2π rad) 1회전 → odom yaw 증가분 확인.
  - ≠2π 이면: `실제_base = 0.30 × (odom회전 / 실제회전)`

## 결과별 분기
- (A)로 확정 → 실측값을 `rover_params.yaml`(+URDF 치수)에 반영 → 맵 겹침 대부분 해소 기대.
- 보정 후에도 회전 시 뒤틀림 남으면 → (B) 구조 한계. 대책 후보: 회전속도 제한 / 더 넓은 FOV·실제 2D 라이다 / depth 기반 RTAB-Map 계열 전환.

## 미해결/열린 항목
- odom_publisher에 "실이동 대비 로그 계측" 추가해 테스트 자동화 옵션 있음(미착수).
- rpm→속도 부호/좌우 매핑이 실기와 일치하는지 회전 테스트에서 함께 검증 필요.
- 과거 odom 발산 사례(-71km 등, cpp 주석)와 이번 겹침 문제의 연관성 미확인(재현 실패 이력).
