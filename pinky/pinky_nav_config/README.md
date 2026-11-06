# pinky_nav_config

핑키 Pi의 Nav2·AMCL 설정과 지도.

- 컨트롤러는 RPP. 막히면 멈추기만 하고 돌아가지 않는다. Spin·BackUp 복구는 쓰지 않는다.
- 점유 지도의 원점은 `pc/fleet_core/fleet_core/data/lane_map.yaml`과 같게 한다.
