# 작업 기록: 버전 2 맞추기 (2026-11-10)

브랜치 `v2-parity`(PJW-1/roscamp-repo-2). 출발점은 `origin/main` 0858a61(버전 2 마감)이다. 지시서는 Drive의 `FMS_버전2_맞추기_지시서_2026-11-09.md`이고, 참고 레포는 `PJW-1/roscamp-freefloor` 브랜치 `freefloor-route`(72d8edb)다.

목표: 자유바닥 실험 레포가 11-09에 받은 것 중 레인에도 맞는 것(Gazebo 창고 틀, 통과용 RPP, 런치 인자, 실제 경기장, 같은 측정 세트)을 버전 2 레인 관제에 가져와, 두 방식을 같은 조건에서 잰다. 사용자 지시로 main을 이 브랜치로 빨리 감기(fast-forward)했고, 브랜치를 정리하면서 `v2-parity` 브랜치는 같은 이름의 태그로 남겼다. 팀 레포에는 push하지 않았고, 실물 로봇에는 접속하지 않았다.

트랙별 자세한 기록은 `docs/worklog_v2_parity_b_2026-11-10.md`(통과용 RPP와 런치 인자), `docs/worklog_v2_parity_c_2026-11-10.md`(실제 경기장), `docs/worklog_v2_parity_d_2026-11-10.md`(측정 도구)에 있다. 세 트랙은 각자 worktree와 브랜치(`par-b`, `par-c`, `par-d`)에서 0688a6d를 출발점으로 일했고, 메인 세션이 v2-parity에 합쳤다.

## 가져온 것 (커밋 대응표)

| 지시서 | 가져온 곳 | v2-parity 커밋 | 비고 |
|---|---|---|---|
| 1. Gazebo 창고 틀(sites, arena.py, warehouse 지도·월드, warehouse_rush, gz_warehouse) | 이 레포 `gz-portfolio-rec` | 0688a6d(병합), 461c63e, 0e0c8c8 | 충돌은 버전 2 쪽 동작을 지켜 풀었다(아래 '병합에서 지킨 버전 2 동작') |
| 1. `gz_nonstop`, `gz_warehouse_nonstop` | c0d5e9d, d57a21e | eeefa69 | 실행 줄의 `traffic_mode:=freefloor`를 뺐다 |
| 1. Gazebo physics 4 ms | 37eb6e5 | (이미 있음) | `gz-portfolio-rec`의 32fef0a가 같은 변경이다 |
| 2. 둘째 RPP `FollowPathThrough`(접근 감속 없음) | 05a09ae, 0480736 | 5fab8fe | `approach_velocity_scaling_dist`만 0.05 |
| 2. 잠금 경계 경로(`final_goal` 거짓)를 둘째 controller로 | 1ee7f99(pinky_fsm 부분), ae4eb4c | 158529c | 노드 파라미터 `through_controller_id`(노드 기본 "", launch 기본 `FollowPathThrough`) |
| 2. 속도 상한의 지연 여유, 상한 0.3 → 0.25 m/s | be13fe3, 6492f5d | ad5a499 | `horizon_latency_s` 0.3 s(추정) × 상한만큼 d에서 뺀다 |
| 2. `pinky_fsm.launch.py`의 `horizon_cap_mps` | a7d1680(pinky_fsm 부분) | d569019 | |
| 2. gz.launch 인자 `nav_pass_through`, `nav_max_speed`, `nav_collision_check`, `fleet_params_file` 이름 해석 | 1ee7f99·e17b076(sim 부분), 5c41979, a7d1680, 92a87fe, 1d5eaab | 84903cc, 618f9cf | `nav_max_speed`는 숫자가 아니거나, 로봇 파일 속도보다 크거나, FSM 후진 속도(0.08)·감속 바닥(0.05)보다 작으면 거절 |
| 2. site별 `lane_clearance_m`, 숫자만 받기 | 6c63fd0, 12f8f09(arena.py 부분) | 536f966 | 12f8f09의 floor_map.py 부분은 자유바닥 전용 |
| 2. 측정 도구 | `tools/compare_floor.py`의 열 | 44c183b, c7ad3fb, 07e114c, 7659955, 3eec328, 92a9235 | 새 도구 대신 `tools/baseline.py`를 늘렸다. `--speed`, `--noise`, `--params` |
| 2. `long_orders_100` | 753c956 | 1277967 | 그대로 복사 |
| 3. 실제 경기장 site·월드·지도·시나리오 | 2d335a1, 9a5e83a | cdcd761, ed78505 | 레인 지도와 `fleet_params_real_arena.yaml`은 지시서대로 새로 만들었다 |
| 6. 지도 시험이 지도별 파라미터 파일을 씀 | - | 12c3977 | 아래 '지시서와 다르게 한 것' |

합치기 커밋: 926d2f4(par-c), f4dbe1c(par-d), d8d6735(par-b). 그 밖에 e1eb457(경기장 실행 줄 주석), 6e69e0e(공백)이 있다.

### 병합에서 지킨 버전 2 동작

- `gz-portfolio-rec`의 `arena_for`는 site가 없는 지도를 거절했다. 그대로면 버전 2 F-2의 `free_area_crossing`을 Gazebo에서 돌릴 수 없다. site가 없으면 lane_arena 월드에 그 레인 지도를 쓰도록 되돌렸다(461c63e).
- 시나리오 파일 옆 `maps` 폴더를 먼저 찾는 버전 2 규칙, site의 `lane_map` 키, 빈 `lane_map`은 기본 지도라는 뜻을 함께 지켰다(461c63e, 0e0c8c8).
- 자유바닥의 `final_goal` 배선(8ce2ded 병합: fleet_manager 어댑터, `traffic_mode`·`params_for` 복사, `mark_final_goal` 기본값)은 가져오지 않았다. 통과용 RPP는 버전 2의 `mark_final_goal`이 채우는 `MoveTo.final_goal`을 그대로 쓴다. √(2ad) 상한과 모서리 둥글리기도 버전 2 그대로이고, 지연 여유만 더했다.

## 지시서와 다르게 한 것

- 실제 경기장 site의 `lane_clearance_m`은 자유바닥의 0.12가 아니라 0.10이다. 지시서의 레인 지도에서 G→PS 레인이 block_long 모서리를 0.106 m로 지나가므로, 0.12로는 월드 생성이 거부된다.
- 실제 경기장 레인 지도는 지시서의 YAML 그대로다. 자유바닥 레포의 같은 이름 파일(양방향 CORR·BAY, QO→PS 출구, PS 위치 (0.90, 0.575))과 다르다.
- `fleet_params_file`은 새 인자를 만들지 않고 기존 인자를 늘렸다. 폴더 없이 이름만 주면(`.yaml`은 있어도 되고 없어도 된다) `fleet_core/data/<이름>.yaml`이다.
- fleet_core의 지도 시험 두 개(`test_shipped_maps.py`, `test_release_floor.py`)는 `sim/fake_robots/maps`의 모든 지도를 기본 해제 거리 0.2 m로 검사했다. 경기장 지도(가장 짧은 레인 0.174 m)가 그 둘에서 실패했다. 이제 지도마다 `fleet_core/data/fleet_params_<지도 이름>.yaml`이 있으면 그 파일로 검사한다. 지금 그런 파일은 `fleet_params_real_arena.yaml` 하나이고, 이 규칙을 시험으로 고정했다(12c3977).

## 측정

### 머리 없는 가짜 세계

- 레인 v2: v2-parity f4dbe1c의 git archive에서 `tools/baseline.py --modes path`(SimWorld). 출하 `fleet_params.yaml`(real_retreat, mark_final_goal, corner_radius_m 0.12, free_cells 켬, borrow_enabled 끔). 경기장 줄은 `--speed 0.12 --noise 0.03 --params fleet_params_real_arena`와 실제 경기장 레인 지도다. f4dbe1c 뒤에는 관제 코드가 바뀌지 않았다(요약 함수만 바뀜).
- 자유바닥: freefloor-route 72d8edb의 git archive에서 `tools/compare_floor.py --modes freefloor`. 출하 설정(`floor_dock_schedule: false`, 자유바닥 작업 기록의 '끔' 열)이다. 경기장 줄은 자유바닥 기록의 경기장 잠금 셋(`floor_lock_ahead_m` 0.25, `floor_extend_step_m` 0.05, `release_distance_m` 0.17), 0.12 m/s, 잡음 0.03 m다.
- 횟수: 씨드마다 한 번. 한 쪽에 88번(lane_map 3개 × 20, 창고 15, 주문 100건 3, 경기장 10)이다. 두 쪽을 같은 시각(00:52~00:57)에 12개 프로세스로 함께 돌렸다(16코어, nice 없음). 그래서 tick은 같은 부하에서 잰 값이다.
- 확인: D 트랙이 먼저 잰 값(`docs/v2_parity/`, 54fa99d)과 tick을 뺀 모든 칸이 같다. 자유바닥 쪽은 자유바닥 작업 기록 521661b 표의 '끔' 줄(끝난 시각, 교착 감지, 못 풂, 최소 간격)과 같고, 경기장 9/10, 292.2 s도 같다.
- 끝난 시각과 평균은 끝난 실행만, 사건 수는 모든 실행의 합, 최소 간격은 끝난 실행의 최솟값이다.

| 시나리오 (씨드) | 방식 | 끝남 | 마지막 주문 보고(s) 평균 (최소~최대) | 기다린 시간(s) | 달린 거리(m) | 최소 간격(m) | tick ms 평균 / p99 중앙값 / p99 최대 | 교착 감지 | 못 풂 | 격리 | 위반 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| basic_orders (1~20) | 레인 v2 | 20/20 | 177.4 (176.3~183.6) | 104.1 | 55.9 | 0.400 | 0.18 / 1.54 / 2.13 | 0 | 0 | 0 | 0 |
| basic_orders (1~20) | 자유바닥 | 20/20 | 185.7 (184.1~189.0) | 36.7 | 49.8 | 0.313 | 1.56 / 7.08 / 10.41 | 50 | 1 | 0 | 0 |
| order_burst (1~20) | 레인 v2 | 20/20 | 264.8 (264.2~265.4) | 137.9 | 82.7 | 0.400 | 0.17 / 1.52 / 2.12 | 0 | 0 | 0 | 0 |
| order_burst (1~20) | 자유바닥 | 20/20 | 276.6 (271.3~289.6) | 52.4 | 78.2 | 0.300 | 1.36 / 5.37 / 6.07 | 41 | 1 | 0 | 0 |
| blocked_lane (1~20) | 레인 v2 | 20/20 | 195.2 (194.6~195.8) | 137.6 | 55.9 | 0.307 | 0.17 / 1.13 / 1.26 | 0 | 0 | 0 | 0 |
| blocked_lane (1~20) | 자유바닥 | 20/20 | 206.0 (201.5~222.8) | 43.2 | 50.8 | 0.296 | 1.20 / 5.77 / 7.00 | 50 | 0 | 0 | 0 |
| warehouse_rush (1~15) | 레인 v2 | 15/15 | 1301.6 (1300.6~1303.1) | 342.8 | 611.0 | 0.350 | 0.18 / 0.36 / 0.47 | 0 | 0 | 0 | 0 |
| warehouse_rush (1~15) | 자유바닥 | 14/15 | 1308.5 (1269.3~1358.3) | 299.6 | 612.6 | 0.286 | 2.32 / 8.90 / 11.20 | 442 | 5 | 0 | 1 |
| long_orders_100 (1~3) | 레인 v2 | 3/3 | 2162.0 (2161.0~2163.4) | 489.1 | 685.2 | 0.400 | 0.17 / 2.13 / 2.14 | 0 | 0 | 0 | 0 |
| long_orders_100 (1~3) | 자유바닥 | 3/3 | 2291.1 (2281.3~2306.3) | 487.3 | 724.1 | 0.300 | 1.59 / 5.34 / 6.32 | 40 | 0 | 0 | 0 |
| gz_real_arena (1~10) | 레인 v2 | 10/10 | 292.0 (289.5~297.8) | 33.9 | 43.8 | 0.305 | 0.10 / 0.23 / 0.26 | 0 | 0 | 0 | 0 |
| gz_real_arena (1~10) | 자유바닥 | 9/10 | 292.2 (291.1~293.9) | 3.6 | 42.2 | 0.292 | 2.70 / 12.05 / 15.53 | 1 | 0 | 0 | 1 |

읽는 법:

- 레인 v2가 모든 줄에서 먼저 끝나거나 같다. 마지막 주문 보고 차이는 basic_orders −4.5%, order_burst −4.3%, blocked_lane −5.2%, 창고 −0.5%, 주문 100건 −5.6%, 경기장 −0.1%다. 끝남은 창고(15/15 대 14/15)와 경기장(10/10 대 9/10)에서 레인이 하나씩 많다. 자유바닥이 못 끝낸 두 실행은 시작 0.1 s의 배치 위반(창고 씨드 2, 경기장 씨드 8)이고, 자유바닥 기록에도 있다.
- 레인 v2의 교착 감지·못 풂·격리는 모든 줄에서 0이다. 자유바닥은 대기 그래프 순환을 수십~수백 번 감지하고 재계획으로 푼다.
- 자유바닥은 기다린 시간이 짧고 lane_map에서 달린 거리가 짧다. 창고와 주문 100건에서는 거리가 같거나 길다.
- 최소 간격(로봇 중심 사이)은 레인 v2가 모든 줄에서 크다(0.305~0.400 m 대 0.286~0.313 m).
- tick: 레인 v2는 평균 0.10~0.18 ms, p99 최대 2.14 ms다. 자유바닥은 평균 1.2~2.7 ms, p99 최대 15.5 ms다. 둘 다 관제 주기 100 ms에 한참 못 미친다.
- 같은 열 이름이라도 뜻이 다른 것: 기다린 시간은 레인은 구역 대기, 자유바닥은 칸 대기다. 위반은 레인은 SimWorld 레인 불변식, 자유바닥은 FloorWorld 칸 불변식이다. 교착 감지는 레인은 구역 대기 순환, 자유바닥은 칸 대기 순환이다.
- 레인 v2 경기장 10/10, 292.0 s는 지시서의 레인 참고값과 같다. D 트랙 확인으로는, 이 참고값은 자유바닥 문서에 없고 자유바닥 레포의 경기장 레인 그림을 compare_floor lanes로 돌리면 0/10이다(두 로봇이 BAY와 PS를 서로 기다려 격리). 지시서의 레인 지도로 돌린 이번 값이 레인 기준이다.

### Gazebo

레인 v2 Gazebo는 **완주한 실행이 없다.** 세 번 시작했고 셋 다 도중에 멈췄다. 모두 e1eb457 스냅숏, 도메인 60, `sim_dock_stub:=true`로 한 번씩 돌렸다. 경기장은 지시서대로 `fleet_params_file:=fleet_params_real_arena nav_max_speed:=0.12 nav_collision_check:=false`다.

| 실행 | 렌더링 | 멈춘 시뮬레이션 시각 | 멈춘 까닭 | 주문 배정 / 도킹 | 로봇 중심 최소 거리 | 정지(이동 중) | 실시간 비율 | 교착·격리 |
|---|---|---|---|---|---|---|---|---|
| gz_warehouse_nonstop 1회차(화면 없이 시작, 도중에 GUI·RViz를 붙임) | CPU(llvmpipe) | 233.3 s | 사용자가 경기장을 먼저 보려고 함 | 6 / 11 | 0.35 m | 18 (0) | 0.41 | 0 / 0 |
| gz_real_arena(GUI·RViz) | CPU(llvmpipe) | 105.7 s | GPU로 다시 띄움 | 2 / 3 | 0.256 m | 5 (0) | 0.56 | 0 / 1 |
| gz_real_arena(GUI·RViz) | GPU(D3D12) | 116.0 s | 사용자가 자유바닥 최신판을 보려고 함 | 2 / 3 | 0.245 m | 6 (4) | 0.98 | 0 / 1 |

창고 1회차의 `[ERROR]` 줄은 모두 멈춤 신호(SIGINT) 뒤 노드가 내려가며 찍힌 것이다. 경기장 두 번의 격리는 멈춤 신호 **전**에 났다.

경기장 격리(두 번 똑같이 났다):

- pinky1이 OUTBOUND 도크 O에서 일을 마치고 S1로 갔다. 목표는 SUCCEEDED로 끝났지만 O와 S1을 쥔 채 다음 목표도 다음 구역 요청도 없이 10 s 서 있었다(`ROBOT_IDLE TO_WORK at S1 ... holding ['O', 'S1'], last goal SUCCEEDED`). 관제가 S1을 세 번 다시 보낸 뒤 격리했다(`GOAL_FAILED gave up after 3 attempts: ROBOT_IDLE`).
- 원인(추정, 로그와 숫자로 맞춰 봄): O→S1이 0.174 m로 지도에서 가장 짧은 나가는 레인이다. Nav2 목표 허용치(`xy_goal_tolerance` 0.05 m) 안에서 서면 로봇은 레인을 0.124~0.174 m 간다. 해제는 0.17 m를 넘어야 하므로 S1에서 4 mm 안쪽에 서야 O를 놓는다. 버전 2의 `test_the_crossing_map_leaves_room_to_stop_short_on_every_exit_lane`이 다른 지도에서 막던 바로 그 경우다(나가는 레인이 해제 거리 + 0.15 m보다 짧으면 끝에 선 로봇이 앞 구역을 못 놓는다).
- 머리 없는 측정(10/10)에서 안 보인 까닭: 가짜 로봇은 지점에 정확히 선다.
- 이 지도에서 해제 거리 + 0.15 m(0.32 m)보다 짧은 나가는 레인: O→S1 0.174, S3→G 0.226, S2→S3 0.26, QW→W 0.30, G→PS 0.316 m. Gazebo에서 본 것은 O→S1뿐이다. 다른 넷은 앞 구역이 막혀 끝에 설 때 같은 일이 날 수 있다(안 봄).
- 지시서가 지도를 그대로 쓰라고 했고 지도 변경은 사용자 결정이라 고치지 않았다. 아래 '남은 일' 1번.

그 밖에 본 것:

- 경기장 Gazebo에서 처음 30 s 안에 두 로봇 중심이 0.245~0.256 m까지 가까워졌다(머리 없는 최소 0.305 m). 어디서인지는 보지 않았다.
- 실행이 완주하지 못해 `FollowPathThrough` 전환이 실제 Nav2에서 일어나는지와 속도 효과(자유바닥 0.133 → 0.162 m/s)는 확인하지 못했다. launch 로그에는 controller 이름이 찍히지 않는다.
- WSLg는 기본으로 OpenGL을 CPU(llvmpipe)로 그린다. `GALLIUM_DRIVER=d3d12`를 주면 GPU(RTX 3080)로 그리고, 같은 경기장 실행의 실시간 비율이 0.51~0.56에서 0.98로 올랐다. 결과는 시뮬레이션 시계로 재므로 숫자에는 영향이 없다.

자유바닥 쪽 Gazebo 참고값(자유바닥 문서): 창고 nonstop은 같은 코드 두 번에 마지막 출고 738 s와 878 s(146 s 차이), 움직일 때 평균 속도 0.163~0.166 m/s다(`docs/freefloor/README.md`). 경기장은 `freefloor-arena3`(c2517cb)에서 핑키 3대, 0.15 m/s로 돌렸다(그 브랜치의 `docs/worklog_freefloor_arena3_2026-11-09.md`).

## 창고 대기 칸과 도크 앞 대기 (4단계 확인)

대기 칸 P-1~P-3(0.35 m 간격)에서 레인도 막히는가:

- 막히지 않는다. 레인 v2 창고(warehouse_rush 15씨드, 머리 없음)는 15/15로 끝났고 위반·교착이 0이다. 최소 간격 0.350 m는 대기 칸에 나란히 선 두 로봇의 중심 간격, 곧 칸 간격 그대로다.
- 15씨드 모두에서 로봇이 대기 칸 구역(P-1, P-2, P-3)을 기다린 시간은 0.0 s다(탐침: 매 단계 `zones.waiting_for`를 구역별로 더함, f4dbe1c). 레인에서는 지점 하나가 구역 하나라 옆 칸에 선 로봇이 서로의 구역을 잡지 않는다. 자유바닥은 몸 칸(반지름 + 여유)과 위치 잡음이 0.35 m에 빠듯해서, 씨드 2가 시작 0.1 s에 이웃 로봇의 칸을 잡아 멈췄다.

도크 앞 대기가 큰가:

- 크다. 15씨드의 구역 대기 합 5,141 s 중 도크 구역(W, O)을 기다린 시간이 2,646 s(51%)다. OUTBOUND 도크 O가 씨드마다 약 104~110 s, WORK 도크 W가 약 68~74 s다. 도크 앞 대기 지점(QS, QOS)은 17 s(0%)이고, 나머지 2,479 s(48%)는 통로다. 통로 중에서는 TUN(A2·C2를 묶은 구역)이 씨드마다 약 84~93 s(약 26%), A1이 약 47~50 s(약 14%)다.
- D 트랙 탐침에서는 order_burst 씨드 1이 138.7 s 중 W 59.9 s(43%), 주문 100건 씨드 1이 493.4 s 중 W 95.1 s(19%, AVENUE가 395.5 s)였다.

제안(구현하지 않음, 지시서 4단계):

- 도크 시간창의 레인판: 도크를 쓰는 로봇이 끝날 시각(품목 수 × 4 s, 교체 8 s, 도킹 3 s)을 관제가 알고 있으니, 다음 로봇이 도크 앞에 그보다 일찍 닿지 않게 출발을 늦추거나 그동안 다른 도크의 일을 먼저 준다.
- 다만 레인에서 도크 구역을 기다리는 것 자체는 다른 로봇을 막지 않는 한 손해가 아니다(도크가 비면 바로 들어간다). 줄일 몫은 도크를 기다리는 로봇이 통로(TUN, A1)를 막아서 생기는 2차 대기다. 시간창을 만들기 전에, 도크 대기 중인 로봇 때문에 다른 로봇이 통로에서 기다린 시간을 따로 재는 것이 순서다.

## 시험

e1eb457의 git archive를 colcon으로 빌드하고 패키지마다 pytest를 한 번 돌렸다(WSL, 도메인 61, fleet_manager는 고정 도메인 50~59와 잠금 파일). 2,435개가 모두 통과했다.

| 패키지 | 결과 | 걸린 시간 |
|---|---|---|
| fleet_core | 1320 passed | 214 s |
| pinky_fsm | 171 passed | 82 s |
| pinky_nav_config | 17 passed | 1 s |
| fake_robots | 457 passed | 280 s |
| fleet_gz_sim | 366 passed | 6 s |
| fleet_manager | 104 passed | 761 s |

- 그 뒤 커밋: 6e69e0e는 공백 두 칸이다. 92a9235와 e101ae1(요약과 표)은 `test_baseline_tool.py` + `test_eta_error_tool.py` 68 passed로 확인했다(소스에서 pytest).
- 시험을 먼저 쓰고 빨강을 본 것: 0e0c8c8(ScenarioError), 12c3977(지도 시험 2 failed, 도우미 시험 ImportError), 92a9235(`(1, 0.0, 0.0) == (2, 3, 1.5)`), e101ae1(`violations` 열 없음).
- ruff: `pc/fleet_core`, `sim/fleet_gz_sim`, `pinky`, `tools` 통과. `sim/fake_robots/fake_robots/planner.py:5` E501은 전부터 있던 것이다.

## 검수 (Codex, gpt-6.1-terra high)

| 범위 | 결과 | 처리 |
|---|---|---|
| 0858a61..eeefa69(창고 병합, nonstop) | 확정 3: 시나리오 옆 maps 폴더를 안 씀, 버전 2 launch 배선 시험 4개 실패, site의 `lane_map`을 무시 | 461c63e(시험 먼저) |
| eeefa69..461c63e | 확정 1: 빈 site `lane_map`이 load에서 시나리오 지도로 바뀜 | 0e0c8c8(시험 먼저) |
| 461c63e..0e0c8c8 | 결함 없음 | |
| par-b(두 번) | 확정: `.yaml`이 붙은 이름이 상대 경로로 넘어감, 기록의 부등호 | 618f9cf, a2c2782. FakeNav로는 실제 Nav2의 controller 전환을 보일 수 없다는 지적은 Gazebo 확인으로 넘김 |
| par-c | 확정 P3 3건: 경기장 고리 시험이 점 하나만 봄, 시험 이름, `lane_clearance_m`이 1e-10처럼 아주 작아도 통과 | 앞의 둘은 ed78505. 마지막은 자유바닥 규칙과 같고 0.10에서는 영향이 없어 남김 |
| par-d | 확정 3건: 불변식이 아닌 AssertionError도 위반, 깨진 단계는 간격·거리에서 빠짐, 사건 수 시험이 0만 봄 | 앞의 둘은 compare_floor와 같아 문서로 남김, 마지막은 7659955. 재검수는 Codex 용량 부족으로 D가 직접 하고 3eec328 |
| 0858a61..e1eb457(브랜치 전체) | 확정 1: 요약이 끝나지 못한 실행의 교착·격리·위반을 뺌 | 92a9235(시험 먼저) |
| 6e69e0e..92a9235(요약 고침) | 확정 2: 표에 `violations` 열이 없음, 도움말의 집계 기준이 예전 것 | e101ae1(시험 먼저) |
| 92a9235..e101ae1 | 결함 없음 | |

## 남은 일

1. **실제 경기장 레인 지도의 O→S1(0.174 m) 고장 (결정 필요).** Gazebo 경기장 두 번 모두 OUTBOUND를 마친 로봇이 S1에서 O를 놓지 못하고 격리됐다(위 'Gazebo'). 고치는 길은 셋이다: S1을 옮겨 O→S1을 0.32 m 이상으로, O와 S1을 한 구역으로 묶기(구역 안 레인은 해제를 따지지 않아 나가는 레인이 S1→S2 0.39 m가 된다), 경기장 해제 거리를 더 줄이기(로봇 사이 여유도 준다). 정하기 전에는 실물 경기장에서 레인 v2를 쓰지 않는다. 같은 조건의 다른 짧은 레인 넷(S3→G, S2→S3, QW→W, G→PS)도 함께 본다.
2. **레인 v2 Gazebo 측정을 끝까지 돌리기.** 지시서의 창고 nonstop 두 번과 경기장 한 번 중 완주한 것이 없다. 경기장은 1번을 정한 뒤에 다시 잰다. GPU(`GALLIUM_DRIVER=d3d12`)로 돌리면 실시간 비율이 약 두 배다.
3. 실물 기본값 확인(사용자): `pinky_fsm.launch.py`의 `through_controller_id` 기본이 `FollowPathThrough`라 실물 핑키도 잠금 경계 경로에 접근 감속 없는 RPP를 쓴다(`through_controller_id:=""`로 끈다). `horizon_cap_mps`는 0.3에서 0.25 m/s(Nav2 `desired_linear_vel`)로 내렸다.
4. 추정값 실측: `horizon_latency_s` 0.3 s, `horizon_decel_mps2` 0.25 m/s².
5. `FollowPathThrough` 전환이 실제 Nav2에서 일어나는지 Gazebo에서 확인(FakeNav 시험은 보낸 controller 이름만 본다).
6. 경기장 Gazebo의 로봇 중심 최소 거리 0.245~0.256 m가 어디서 났는지.
7. 실제 경기장 G→PS가 block_long 모서리를 0.106 m로 지난다(몸 반지름 0.08 m면 가장자리 2.6 cm). 장애물 위치는 ±5 cm 추정이고, RPP 충돌 검사를 끈다. 실물 전에 자리를 재야 한다.
8. `lane_clearance_m`이 1e-10처럼 아주 작아도 통과한다(자유바닥 규칙과 같음). 최솟값을 둘지.
9. 도크 시간창은 제안만 했다. 먼저 도크를 기다리는 로봇 때문에 다른 로봇이 통로에서 기다린 2차 대기를 따로 재는 열이 필요하다.
10. tick은 두 방식을 같은 시각, 같은 부하에서 잰 값이다. 한가한 기계에서 따로 재면 절댓값은 달라질 수 있다.
11. `fleet_params_file`에 Windows 역슬래시 경로를 주면 이름으로 본다. launch는 WSL에서만 돌려 남겼다.
12. 합치지 않은 원격 브랜치 둘: `pinky-fsm`(팀 main 스냅숏 위의 pinky_fsm 1차판, 9커밋), `feat/repo-layout-preview`(11-06 문서·CI 정리 미리보기, 18커밋). 지울지 사용자가 정한다.
13. `sim/fake_robots/fake_robots/planner.py:5` E501(전부터 있던 것).
14. **RPP 비용 감속 (해결함, 사용자 결정).** `pinky_nav_config`의 두 RPP는 `cost_scaling_dist` 0.6, `cost_scaling_gain` 1.5, 바닥 `regulated_linear_scaling_min_speed` 0.05였고, 코스트맵 `inflation_radius`는 0.15 m다. 비용이 매겨지는 가장 먼 곳(0.15 m)에서도 0.15 × 1.5 × 0.15 / 0.6 = 0.056 m/s라, 장애물 0.15 m 안에 들면 바로 바닥에 붙었다. 자유바닥 최신판(c2517cb) Gazebo 경기장에서 사용자가 "장애물 옆에서 느려진다"고 봤고, 재 보니 pinky1이 pinky_fsm 속도 제한 없이 0.050 m/s로 달렸다. 사용자가 0.05로 내리라고 해서 두 레포를 같이 바꿨다(자유바닥 585bcf8, 이 레포는 다음 커밋). Jazzy RPP는 거리를 `내접 반지름 + ln(253/비용)/3`으로 읽어 닿기 전에는 0.06 m 이상이므로, 1.5 × 0.06 / 0.05 = 1.8 > 1이 되어 이 감속은 사실상 꺼진다. 자유바닥 Gazebo 경기장(같은 시나리오, 한 번씩): 이전 8/8 출고 361.5 s, 바꾼 뒤 8/8 출고 224.1 s, 로봇 최소 거리 0.094 → 0.093 m. 레인 v2 Gazebo로는 재지 않았다. 실물 핑키도 이 파일을 쓰므로 실물에서는 상자 옆에서도 최고 속도로 달린다.
