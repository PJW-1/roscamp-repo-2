# 버전 2 맞추기 트랙 B: 잠금 경계 이동의 둘째 RPP와 launch 인자 (2026-11-10)

지시서 "FMS 버전2 맞추기 지시서 2026-11-09" 2단계 1·2행. 자유바닥 실험 레포(`roscamp-freefloor`,
`origin/freefloor-route` 72d8edb)의 레인 쪽 개선을 버전 2(`v2-parity` 0688a6d)에 옮겼다.
브랜치 `par-b`, 중간에 `origin/v2-parity` 461c63e, 926d2f4를 합쳤다.

## 커밋 대응

| 자유바닥 | par-b | 내용 |
|---|---|---|
| 05a09ae, 0480736 | 5fab8fe | `pinky_nav_config`가 `FollowPathThrough`(FollowPath에서 `approach_velocity_scaling_dist`만 0.05)를 함께 띄운다 |
| 1ee7f99(pinky_fsm 부분), ae4eb4c | 158529c | `SendPath.final_goal`, 노드 파라미터 `through_controller_id`(기본 ""), launch 기본 `FollowPathThrough` |
| be13fe3, 6492f5d | ad5a499 | 속도 제한에서 지연 여유 `horizon_cap_mps × horizon_latency_s`(0.3 s, 추정)를 뺀다. `horizon_cap_mps` 0.3 → 0.25(Nav2 desired_linear_vel) |
| a7d1680(pinky_fsm 부분) | d569019 | `pinky_fsm.launch.py`가 `horizon_cap_mps`를 받는다 |
| 1ee7f99·e17b076(시뮬 부분), 5c41979, a7d1680, 92a87fe, 1d5eaab | 84903cc | `gz.launch.py`: `nav_pass_through`, `nav_max_speed`, `nav_collision_check`, `fleet_params_file` 이름 해석 |
| (새로) | fbd8e38, 3ec7d85, 이 커밋들 | real_arena 명령 줄 시험, README, Codex 지적 고침 |

## 버전 2에 맞춘 것

- final_goal 배관은 가져오지 않았다. 버전 2는 이미 관제 `mark_final_goal`(계약 0.8-draft `MoveTo.final_goal`)로
  목표마다 보내고, pinky_fsm이 max(horizon_min_mps, √(2·a·d))로 제한한다. 자유바닥 8ce2ded의 fleet_manager
  어댑터, freefloor 사본의 `mark_final_goal` 기본값 등은 버전 2에 없는 traffic_mode 사본 기능이라 뺐다.
  더한 것은 controller 고르기, 지연 여유, 상한 0.25, launch 인자뿐이다.
- 버전 2의 바닥(0.05 m/s)과 내림 반올림은 그대로 두고 그 앞에서 지연 여유만 뺀다.
- 버전 2 `gz.launch.py`에는 `params_for`(traffic_mode 사본)가 없다. `fleet_params_file`은 기존 인자를 넓혀
  `params_file_of`로 이름을 풀고 그 결과를 `fleet()`에 그대로 넘긴다(인자를 새로 만들지 않았다).
- 자유바닥 시험의 freefloor 전용 부분(`traffic_mode`, 사본 경로 검사)은 옮기지 않았다.
- `nav_max_speed`가 로봇 설정 속도(0.25)보다 크면 `sim_nav_params`가 거절한다(자유바닥과 같다). gz 인자 시험에
  0.3과 inf를 더했다.
- 시나리오 키는 launch가 읽지 않는다. `gz_real_arena.yaml`(트랙 C)은 주석의 실행 줄에 launch 인자를 적을 뿐이다.

## 시험(WSL Jazzy, 도메인 44, `rec_test_pkgs.sh`로 빌드 후 pytest)

- 926d2f4 합친 뒤(fbd8e38): pinky_fsm 171 passed, pinky_nav_config 17 passed, fleet_gz_sim 366 passed.
- 같은 빌드의 fleet_core: 2 failed, 1303 passed. 실패 둘은 트랙 C의 `sim/fake_robots/maps/real_arena.yaml`이다
  (`test_shipped_maps`: `shortest_exit_lane()`가 0.174 m로 기본 release 0.2 m보다 짧아 실패, `test_release_floor` 같은 지도).
  이 트랙이 건드리지 않은 파일이라 고치지 않았다.
- Codex 고침 뒤: fleet_gz_sim 366 passed(먼저 바꾼 시험이 2 failed로 빨갛게 나온 것을 봤다).
- 행동을 바꾼 단계마다 시험을 먼저 써서 빨간 것을 봤다: pinky_nav_config 6 failed(빌드 전 nav2 launch 4건 포함),
  pinky_fsm test_core 3 failed(SendPath.final_goal), 12 failed(지연 여유·상한), test_launch 1 failed(horizon_cap_mps),
  fleet_gz_sim 36 failed. Windows `ruff check`는 모두 통과.

## Codex 검수(0688a6d..fbd8e38, gpt-6.1-terra high)

1. [확정] `fleet_params_file:=fleet_params_real_arena.yaml`처럼 폴더 없이 `.yaml`이 붙은 이름은 그대로 넘어가
   launch 폴더에서 찾았다. 맞다(지시서의 예가 바로 이 꼴이다). 시험을 먼저 고쳐 빨간 것을 보고 폴더 없는 값은
   모두 data 이름으로 풀게 고쳤다.
2. [확정] FakeNav는 controller_id와 speed_limit을 적기만 하므로 Nav2가 실제로 controller를 바꾸는지, 모든
   controller에 제한이 가는지는 이 시험으로 보이지 않는다. 맞다. 코드로 고칠 것이 아니라 Gazebo에서 볼 일이다(남은 일).
- 그 밖에 R13 재전송, 자가 복구 재전송, 마지막 목표로 바꿀 때의 controller, 두 RPP 차이, 속도 상한, 인자 검사에서는
  문제를 찾지 못했다고 했다.

재검수(fbd8e38..7e2b1c2): 이름 해석은 맞다고 했고 두 가지를 짚었다. (1) [확정] 작업 기록의 부등호가 틀렸다 →
고쳤다. (2) `configs\custom.yaml` 같은 Windows 경로를 이름으로 본다 → launch는 WSL/Linux에서만 돌고 거기서
`\`는 폴더 구분자가 아니므로 고치지 않았다.

## 남은 일

- Gazebo에서 확인: 둘째 RPP로 바뀌는지, 경계 앞에서 서는지, 평균 이동 속도(자유바닥 창고 0.133 → 0.162 m/s)가
  레인 관제에서도 나오는지. Gazebo는 메인 세션 담당이라 돌리지 않았다.
- `horizon_latency_s` 0.3 s와 `horizon_decel_mps2` 0.25는 추정이다. 실물에서 재야 한다.
- 실물 핑키도 `pinky_fsm.launch.py`를 쓰면 기본으로 `FollowPathThrough`를 쓴다(실물 Nav2 설정에 함께 들어 있다).
  실물에서 끄려면 `through_controller_id:=""`.
- 트랙 C의 real_arena 지도가 fleet_core 지도 시험 두 건에 걸린다(위). `gz_real_arena.yaml` 11행 주석의 실행 줄이
  줄바꿈 없이 한 줄로 붙어 있다.
