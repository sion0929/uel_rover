# mode_manager 제거 — 드라이브 모드 게이팅을 MCU로 일원화

**작성일:** 2026-07-22
**대상:** `uel_rover_control` 패키지 (Jetson Orin 배포)

## 1. 배경 / 문제

현재 드라이브 모드(RC/AUTO/STOP) 게이팅이 두 곳에서 중복으로 일어난다.

1. **OpenRB-150 (MCU, `firmware/openrb150_motor/openrb150_motor.ino`)** — RC 채널(ch5)을
   인터럽트로 직접 읽어 `getDriveMode()`로 판정하고, `loop()`에서 `MODE_AUTO`일 때만
   `/cmd_vel_out`으로 받은 목표 RPM을 모터에 적용한다. RC/STOP일 땐 `/cmd_vel_out` 내용과
   무관하게 무시하고 직접 RC 값 또는 정지를 적용한다. **MCU 자체가 이미 최종 권위 있는 게이트다.**
2. **`mode_manager` (Jetson, `src/mode_manager.cpp`)** — MCU가 발행한 `/drive_mode`를
   구독해 같은 판정을 다시 내리고, `AUTO`일 때만 Nav2의 `/cmd_vel`을 `/cmd_vel_out`으로
   중계한다.

2번은 1번이 이미 보장하는 안전 속성을 ROS 왕복(구독→재발행) 한 단계 거쳐 다시 확인하는
것뿐이라 실질적인 안전 이득이 없다. 오히려 프로세스 하나·DDS 참여자 하나가 추가로 젯슨의
CPU/디스커버리 자원을 쓰고, 지연을 더한다. **원 피드백(사용자 보고):** MCU가 자기 RC 스위치를
이미 알고 있으니 AUTO가 아닐 때 돌아온 `/cmd_vel_out`을 그냥 무시하면 되고, `mode_manager`는
없애도 된다.

## 2. 변경 사항

- **삭제:** `src/uel_rover_control/src/mode_manager.cpp`
- **삭제:** `src/uel_rover_control/launch/rover.launch.py` — `mode_manager`+`odom_publisher`를
  묶어 실행하던 파일인데, 원래 유일한 참조자였던 `nav2_bringup.launch.py`가 이미 삭제되어
  현재 시점에 이 파일을 include하는 곳이 저장소 어디에도 없다(고아 파일). 같이 정리.
- **`CMakeLists.txt`:** `mode_manager` 빌드 타깃(`add_executable`, `ament_target_dependencies`,
  `install(TARGETS ...)`) 제거. `odom_publisher`는 그대로 유지.
- **`launch/bringup_common.launch.py`:**
  - `mode_manager` `Node()` 항목 제거
  - `controller_server` `Node()`에 `remappings=[('cmd_vel', '/cmd_vel_out')]` 추가 →
    Nav2 컨트롤러 출력이 곧바로 MCU가 구독하는 토픽으로 감
- **`docs/system-overview.md`:** 노드 목록에서 `mode_manager` 행 제거, 토픽 표의
  `/cmd_vel`/`/cmd_vel_out` 행을 "controller_server → (remap) → OpenRB-150"으로 갱신,
  §6(드라이브 모드 게이팅)을 "2단 게이팅"에서 "MCU 단일 게이팅"으로 재작성.

## 3. 영향 없음

- **펌웨어(`openrb150_motor.ino`):** 변경 없음. 이미 올바르게 게이팅하고 있음이 이번
  실기 조사에서 코드로 확인됨.
- **`odom_publisher`, `rover_params.yaml`:** `mode_manager` 전용 파라미터가 없어 영향 없음.
- **`/drive_mode` 토픽:** MCU가 계속 발행. 구독자가 없어져도(mode_manager 삭제) 관측용으로
  rviz2/rqt에서 직접 구독 가능하므로 가시성은 유지됨.

## 4. 검증 체크리스트

1. `colcon build --packages-select uel_rover_control` — `mode_manager` 없이 빌드 성공
2. `ros2 node list`에 `mode_manager` 없음, `controller_server`는 그대로 있음
3. AUTO 모드에서 `ros2 topic echo /cmd_vel_out`으로 Nav2 명령이 직접 나오는지 확인
4. RC 모드로 전환 시 `/cmd_vel_out`에 값이 와도 실제 모터는 RC 입력을 따르는지(펌웨어
   게이팅 확인, 회귀 없음)
5. STOP 모드 전환 시 즉시 정지 확인 (데드맨 스위치 회귀 없음)

## 5. 범위 밖 (별도 스펙)

- 노드 컴포지션(ComposableNodeContainer로 여러 노드를 한 프로세스에 묶기)은 이번 스펙에
  포함하지 않는다. mode_manager 제거로 노드 구성이 먼저 바뀐 뒤, 최종 컴포지션 대상
  노드 목록을 정하는 게 순서상 맞아서 별도 스펙(`YYYY-MM-DD-node-composition-design.md`)
  으로 분리한다.
