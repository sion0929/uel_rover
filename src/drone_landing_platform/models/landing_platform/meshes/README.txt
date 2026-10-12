landing_platform 메시 폴더 (비어 있어도 시뮬레이션은 박스 형상으로 동작합니다)

여기에 SolidWorks 에서 내보낸 STL 6개를 넣으세요:
  frame.stl     - 고정 프레임 전체 (베이스, 코너 블록, 타워 벽, 코너 데크, 핀 가이드)
  floor.stl     - 가동 바닥 (기어/힌지 블록 포함)
  plate_px.stl  - +X 쪽 플레이트 (대기 자세 10.37 deg)
  plate_mx.stl  - -X 쪽 플레이트
  plate_py.stl  - +Y 쪽 플레이트
  plate_my.stl  - -Y 쪽 플레이트

내보내기 규칙 (중요):
  * 단위 mm (SDF 에서 scale 0.001 적용), Z 위쪽
  * 모든 파일을 "플랫폼 좌표계" 그대로 내보낼 것: 원점 = 베이스 플레이트 윗면 중심,
    어셈블리를 대기(standby) 자세로 둔 상태에서 이동/센터링 없이 저장
    (SolidWorks: 다른 이름으로 저장 > STL > 옵션 > "출력 좌표계" 에 플랫폼 좌표계 지정,
     "양수 공간으로 이동 안 함" 체크)
  * 각 링크 좌표계 = 대기 자세의 플랫폼 원점이므로 메시 pose 는 항등(0 0 0 0 0 0)입니다.
  * 충돌 형상은 SDF 의 박스를 사용합니다 (메시 충돌 사용 안 함).

STL 을 추가한 뒤에는 반드시 다시 빌드하세요:  colcon build --packages-select drone_landing_platform
메시가 6개 모두 있으면 launch 의 visual:=auto 가 자동으로 메시 비주얼을 사용합니다.
