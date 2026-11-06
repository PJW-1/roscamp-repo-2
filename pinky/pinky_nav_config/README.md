# pinky_nav_config (자리표시 launch뿐)

핑키 Pi에서 도는 Nav2와 AMCL의 설정, 지도가 들어갑니다. 담당(제안): SLAM·Nav2.

| 넣을 것 | 메모 |
|---|---|
| Nav2 파라미터 | FollowPath 컨트롤러는 RPP, 충돌 검사로 멈추기만 하고 돌아가지 않음. Spin·BackUp 복구 없음(전달사항 R14) |
| 점유 지도 | 실제 배치가 정해진 뒤 SLAM으로. 좌표는 `pc/fleet_core/fleet_core/data/lane_map.yaml`과 같은 원점(전달사항 R2) |
| AMCL용 1 cm 지도 | costmap은 5 cm 그대로 두고 AMCL에만 따로 줌(전달사항 R15) |

시작값: roscamp_ws `sim/pinky_multi_sim/nav2_overrides.yaml`(lookahead 0.15 m, `xy_goal_tolerance` 0.05 m, `rotate_to_heading_min_angle` 0.25 등). 0.33 m 폭 통로에 맞춘 값이라 실제 지도에서 다시 맞춰야 합니다.

레인 지도는 FMS 쪽(`fleet_core/data`)에 둡니다. 둘이 맞는지는 레인이 벽과 로봇·트레이 폭만큼 떨어져 있는지 보는 시험으로 확인합니다(아직 없음).
