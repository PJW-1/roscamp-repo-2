# fleet_gz_sim (자리만 있음)

Gazebo 안에서 핑키 3대를 돌리는 시험 패키지입니다. 가짜 로봇(`fake_robots`)과 달리 진짜 `fleet_manager`, 진짜 `pinky_fsm`, 진짜 Nav2를 그대로 띄웁니다. central과 팔은 `fake_robots`의 가짜 노드를 씁니다. CI에는 넣지 않고 손으로 돌립니다.

지금 있는 것은 격리 검사(`fleet_gz_sim/isolation.py`)와 그것만 하는 `launch/gz.launch.py`뿐입니다. 시뮬은 실물과 같은 `/pinkyN/...` 이름을 쓰므로, 다음이 아니면 시작하지 않습니다.

- 도메인 43~101. ROS 문서의 안전 범위는 0~101이고, 102 이상은 리눅스 기본 임시 포트와 겹치거나 띄울 수 있는 프로세스 수가 적습니다.
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `ROS_STATIC_PEERS` 없음.
- 실제로 불러온 RMW가 Fast DDS(`rmw_fastrtps_cpp`).
- Fast DDS가 따로 읽는 설정이 없음: `ROS_DISCOVERY_SERVER`, `FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE`, `FASTDDS_ENVIRONMENT_FILE`, `FASTDDS_BUILTIN_TRANSPORTS`, 작업 폴더의 `DEFAULT_FASTRTPS_PROFILES.xml`·`DEFAULT_FASTDDS_PROFILES.xml`.

```bash
export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch iponic_bringup sim_gz.launch.py
```

## 채울 것

| 순서 | 내용 | 출처 |
|---|---|---|
| 1 | 월드: 실제 배치가 정해지면 `lane_map.yaml`과 점유 지도에서 만든다 | 새로 만듦. roscamp_ws 시뮬의 월드는 옛 12×7 격자라 쓰지 않는다 |
| 2 | 핑키 모델, 모터만 Gazebo로 바꾸는 브링업, 스캔 필터 | roscamp_ws `sim/pinky_multi_sim`의 `pinky_sim.urdf.xacro`, `bringup_sim.py`, `sim_motor.py`, `scan_filter.py` |
| 3 | 로봇별 Nav2를 하나씩 띄우고 안 뜨면 다시 띄우기 | 같은 곳의 `nav2_supervisor.py`, `nav2_unit.launch.py`. 그 시뮬의 월드와 launch에 묶여 있어 고쳐서 쓴다 |
| 4 | 격리 검사 나머지(Fast DDS 프로파일 등) | 같은 곳의 `sim_config.py` |
| 5 | 진짜 `fleet_manager` + `pinky_fsm`, 가짜 central·팔, 심판 | `pc.launch.py`와 `fakes.launch.py`. 시계는 Gazebo `/clock`이므로 `sim_clock`을 띄우지 않는다 |
| 6 | 상자 시험 같은 시험 도구 | 같은 곳의 상자 시험 스크립트 |

가져오지 않는 것: `traffic.py`, `missions.py`, `free_path.py`(칸 단위로 바닥을 잡는 별개 관제), `obstacle_watch.py`, `obstacle_map.py`(감지는 로봇이 한다), `navigate_lane.xml`(후진 복구가 들어 있다).

이름: 제조사 `pinky_pro`에 `pinky_gz_sim`이 이미 있어서 `fleet_gz_sim`으로 했습니다.
