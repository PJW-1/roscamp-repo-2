# fleet_gz_sim

Gazebo에서 핑키 3대를 돌린다. `fleet_manager`·`pinky_fsm`·Nav2는 진짜, central·팔은 가짜(`fake_robots`). CI 밖에서 손으로 돌린다. 지금은 격리 검사만 있다.

실물과 같은 `/pinkyN/...` 이름을 쓰므로 다음이 아니면 시작하지 않는다(`fleet_gz_sim/isolation.py`).

- `ROS_DOMAIN_ID` 43~101
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS` 없음
- RMW는 `rmw_fastrtps_cpp`, Fast DDS XML 프로파일과 discovery server 없음

```bash
export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch iponic_bringup sim_gz.launch.py
```
