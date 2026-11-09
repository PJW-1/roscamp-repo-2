# 버전 2 맞추기 C: 실제 경기장 (2026-11-10)

브랜치 `par-c` (기준 0688a6d, 이후 `origin/v2-parity` 461c63e를 합쳤다).

## 커밋 대응

| 자유바닥 | par-c | 내용 |
|---|---|---|
| 6c63fd0 + 12f8f09(arena.py, test_arena.py) | 536f966 | 사이트의 `lane_clearance_m`(숫자만). floor_map.py 부분은 가져오지 않음 |
| 2d335a1 + 9a5e83a(사이트, 지도, 월드, 시나리오) | 다음 커밋 | 실제 경기장 파일 일체. 지도 PGM·YAML 네 개는 자유바닥과 바이트 동일 |

가져오지 않은 것(자유바닥 전용): floor_map_real_arena.yaml, fleet_core data의 real_arena pgm, test_floor_real_arena.py.

## 바뀐 것과 이유

- `sim/fake_robots/maps/real_arena.yaml`: 지시서의 한 방향 고리 그대로(들어갈 때 북쪽 벽과 box_small 북쪽 틈, 나올 때 table_2 남쪽과 block_long-box_small 틈). 자유바닥 쪽 파일과 다른 점: 양방향 쌍(CORR, BAY)과 `floor_map`
  키가 없고, QO->EN, EN->O가 한 방향이며, S1, S2, S3, G를 지나 PS(0.93, 0.62)로 들어온다(자유바닥은 QO->PS, PS 0.90/0.575).
- `sites/real_arena.yaml`: `lane_clearance_m`을 0.12에서 **0.10**으로. 지시서 레인의 G->PS가 block_long 모서리를 0.106 m로 지나 0.12면 월드를 못 만든다.
- `worlds/real_arena.sdf`: 새 레인 지도로 다시 만들었다(`python3 -m fleet_gz_sim.arena --site real_arena`). 물리 스텝 4 ms.
- `fleet_params_real_arena.yaml`: 버전 2 `fleet_params.yaml`에서 `release_distance_m` 0.2 -> 0.17만 바꿨다(가장 짧은 레인 0.174 m). 다른 값은 바꾸지 않았다. `package_data data/*.yaml`이 이미 싣는다.
- `scenarios/gz_real_arena.yaml`: 키는 우리 로더가 읽는 것뿐이다(`map`, seed, stock, orders, robots, arm, dock, faults, expect). 주석의 실행 줄에 `fleet_params_file`, `nav_max_speed`, `nav_collision_check`가 나온다(트랙 B가 붙이는 launch 인자, 시나리오 키가 아니다). `traffic_mode:=freefloor`는 뺐다.

## 가장 빠듯한 곳

G->PS와 block_long 모서리 0.106 m(몸체 반경 0.08 m면 가장자리 2.6 cm). 상자 위치는 ±5 cm 추정이고 0.08 m에 트레이가 빠졌다. 다음은 테스트가 레인-벽/상자 최소 0.10 m를 지키는 것까지만 본다.

## 시험

- 새 `test_real_arena_site.py` 15개(레인 길이 > 0.17, 한 방향 단일 고리 13칸, 레인-상자/벽 거리와 최소 지점, 생성기 산출물 바이트 일치, 4 ms, 파라미터 차이는 한 줄, 시나리오가 real_arena 월드를 고름, 시작 자세, 주문 8건과 재고). 지도 파일을 넣기 전 수집 오류로 빨강을 확인했다.
- `test_arena.py`: lane_clearance_m 시험 추가, 먼저 빨강(AttributeError) 후 초록.
- fleet_gz_sim 전체(빌드 후, 합친 뒤): 332 passed. 합치기 전에는 test_launch_wiring 4개가 실패했다(v2-parity 쪽 수정 전의 기존 실패).
- fake_robots 전체(빌드 후, 합치기 전 트리): 457 passed(280 s). 합친 뒤에는 비 ROS 시험만 다시 돌려 329 passed. gz 시나리오 목록은 fake_robots 시험에 없다.
- SimWorld(레인 모드, 0.12 m/s, 잡음 0.03)로 gz_real_arena 시드 1-3: 모두 완료, 8건 SHIPPED, 253/256/254 s, 안전 불변식 위반 없음, 뒤 구역 놓는 최소 거리 0.118/0.118/0.124.

## 열린 것

- Gazebo 실주행과 전체 측정은 메인 세션.
- 0.106 m 지점은 실측으로 확인 필요(RPP 충돌 검사를 끄는 이유).

## Codex 검토 (gpt-6.1-terra, 0688a6d..1030d30)

확정 3건, 모두 P3. (1) 아주 작은 양수 lane_clearance_m(1e-10)이 고정 오차 1e-9에 걸려 접촉을 허용: 자유바닥에서 온 규칙이고 실제 값 0.10에는 영향 없어 고치지 않음. (2) 고리 위치 시험이 점 하나만 봄: 들어가는 길 전체와 나오는 길 앞쪽 전체, G의 틈 위치를 보도록 보강. (3) 시나리오 시험 이름이 runs인데 실행을 안 봄: 이름을 picks_the_real_arena_world로 바꿈. 고친 뒤 52 passed(test_real_arena_site + test_arena). 레인, 파라미터, 시나리오 점검은 이상 없음.