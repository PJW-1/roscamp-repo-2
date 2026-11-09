# 버전 2 맞추기 D: 레인 버전 2 측정 도구와 자유바닥 비교 (2026-11-10)

지시서 "FMS 버전2 맞추기 지시서 2026-11-09" 2단계 마지막 줄과 5단계. 브랜치 `par-d`(0688a6d에서, 926d2f4 병합).

## 자유바닥 → 이 브랜치

| 자유바닥(freefloor-route 72d8edb) | 여기 |
|---|---|
| `tools/compare_floor.py`의 추가 열(min_gap_m, path_m, tick_ms_*, deadlocks, unresolved, isolations, violations) | `tools/baseline.py`에 같은 열(44c183b). 새 도구를 만들지 않고 baseline에 넣었다 |
| `tools/scenarios/long_orders_100.yaml` (753c956) | 그대로 복사(1277967). 버전 2 시나리오 파서가 고치지 않고 받는다 |
| 경기장 측정(시험 코드에서 로봇 속도·잡음을 바꿈, `--set` 잠금 셋) | `--speed`, `--noise`, `--params`(44c183b). 시나리오 이름은 `sim/fleet_gz_sim/scenarios`, `tools/scenarios`에서도 찾는다 |
| - | 표의 tick·간격 소수 셋째 자리(c7ad3fb), 경기장 줄 시험(07e114c) |

## 열의 뜻

compare_floor와 같은 방식으로 잰다: 간격은 매 단계 뒤(시작 자리 제외), 바닥에 없는 로봇은 뺀다. p99는 `sorted[int(0.99*(n-1))]`. 교착 감지·못 풂·격리는 `DEADLOCK_DETECTED`·`DEADLOCK_UNRESOLVED`·`ROBOT_ISOLATED` 사건 수. 다른 점:

- 위반: 레인은 SimWorld의 레인 불변식, 자유바닥은 FloorWorld의 칸 불변식. 같은 0이라도 검사 내용이 다르다.
- 정지: 레인은 잠금 끝에서 서는 일이 적고, 자유바닥은 굴러가는 잠금이 앞 칸을 못 잡아 서는 일이 들어간다(가짜 로봇은 감속이 없다).
- 교착 감지: 자유바닥은 대기 그래프 순환을 수십 번 감지하고 재계획으로 푼다. 레인 버전 2는 같은 사건 이름이지만 구역 대기 순환이다.
- tick: 벽시계라 같은 부하에서 연달아 돈 실행끼리만 비교된다. 이번 두 쪽은 같은 기계에서 겹쳐 돌렸다(nice 10, 다른 에이전트 작업과 함께).
- 달린 거리: 다시 넣기(reinstate)로 시작 자리에 옮긴 것도 더한다(compare_floor와 같다). 이번 측정에는 없다.
- 로봇 설정에 speed_mps·pose_noise_m가 없으면 가짜 로봇 기본값(0.2 m/s, 잡음 0)을 쓴다. 예전에는 만들지 못하고 멈췄다.

## 측정: 레인 버전 2 대 자유바닥

모두 headless. 레인 버전 2: SimWorld, path 모드, 배포 `fleet_params.yaml`(real_retreat, mark_final_goal, corner_radius_m 0.12, free_cells 켬, borrow_enabled 끔). 자유바닥: freefloor-route 72d8edb를 `git archive`로 꺼내 `compare_floor.py --modes freefloor`로 같은 기계에서 다시 돌렸다. 기본값 `floor_dock_schedule: false`(작업 기록의 '끔' 열). 다시 돈 결과는 끝난 시각·교착 감지·못 풂·최소 간격이 작업 기록 521661b 표의 '끔' 줄과 모두 같다. 경기장 줄은 경기장 잠금 셋(`floor_lock_ahead_m` 0.25, `floor_extend_step_m` 0.05, `release_distance_m` 0.17)이다. 레인 경기장은 track C의 `real_arena` 레인 지도와 `fleet_params_real_arena`(release_distance_m 0.17). 끝난 시각과 다른 평균은 끝난 실행만, 사건 수는 모든 실행의 합, 최소 간격은 끝난 실행의 최솟값이다.

| 시나리오 | 방식 | 끝남 | 마지막 주문 보고(s) 평균 (최소~최대) | 기다린 시간 합(s) | 달린 거리(m) | 정지 | 최소 간격(m) | tick ms 평균 / p99 중앙값 / p99 최대 | 교착 감지 | 못 풂 | 격리 | 위반 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| basic_orders | 레인 v2 | 20/20 | 177.4 (176.3~183.6) | 104.1 | 55.9 | 8.2 | 0.400 | 0.16 / 1.17 / 2.02 | 0 | 0 | 0 | 0 |
| basic_orders | 자유바닥 | 20/20 | 185.7 (184.1~189.0) | 36.7 | 49.8 | 9.9 | 0.313 | 1.29 / 5.97 / 6.42 | 50 | 1 | 0 | 0 |
| order_burst | 레인 v2 | 20/20 | 264.8 (264.2~265.4) | 137.9 | 82.7 | 12.2 | 0.400 | 0.16 / 1.21 / 2.25 | 0 | 0 | 0 | 0 |
| order_burst | 자유바닥 | 20/20 | 276.6 (271.3~289.6) | 52.4 | 78.2 | 17.4 | 0.300 | 1.53 / 6.01 / 7.35 | 41 | 1 | 0 | 0 |
| blocked_lane | 레인 v2 | 20/20 | 195.2 (194.6~195.8) | 137.6 | 55.9 | 10.2 | 0.307 | 0.16 / 1.16 / 1.25 | 0 | 0 | 0 | 0 |
| blocked_lane | 자유바닥 | 20/20 | 206.0 (201.5~222.8) | 43.2 | 50.8 | 13.8 | 0.296 | 1.28 / 5.97 / 8.44 | 50 | 0 | 0 | 0 |
| warehouse_rush (씨드 1~15) | 레인 v2 | 15/15 | 1301.6 (1300.6~1303.1) | 342.8 | 611.0 | 55.7 | 0.350 | 0.18 / 0.39 / 0.44 | 0 | 0 | 0 | 0 |
| warehouse_rush (씨드 1~15) | 자유바닥 | 14/15 | 1308.5 (1269.3~1358.3) | 299.6 | 612.6 | 84.6 | 0.286 | 2.37 / 9.23 / 11.93 | 442 | 5 | 0 | 1 |
| long_orders_100 (씨드 1~3) | 레인 v2 | 3/3 | 2162.0 (2161.0~2163.4) | 489.1 | 685.2 | 18.0 | 0.400 | 0.16 / 1.91 / 2.08 | 0 | 0 | 0 | 0 |
| long_orders_100 (씨드 1~3) | 자유바닥 | 3/3 | 2291.1 (2281.3~2306.3) | 487.3 | 724.1 | 145.0 | 0.300 | 1.63 / 5.50 / 6.31 | 40 | 0 | 0 | 0 |
| gz_real_arena (씨드 1~10, 0.12 m/s, 잡음 0.03 m) | 레인 v2 | 10/10 | 292.0 (289.5~297.8) | 33.9 | 43.8 | 14.7 | 0.305 | 0.07 / 0.16 / 0.17 | 0 | 0 | 0 | 0 |
| gz_real_arena (씨드 1~10, 0.12 m/s, 잡음 0.03 m) | 자유바닥 | 9/10 | 292.2 (291.1~293.9) | 3.6 | 42.2 | 4.4 | 0.292 | 2.11 / 8.96 / 10.17 | 1 | 0 | 0 | 1 |

lane_map 세 시나리오는 씨드 1~20. 실행 수: 레인 60 + 15 + 3 + 10 = 88, 자유바닥 88. 원자료 `docs/v2_parity/*.csv`(레인), `docs/v2_parity/freefloor/*.csv`(자유바닥).

- 위반 2건은 둘 다 자유바닥 시작 0.1 s의 배치 문제다: 창고 씨드 2(대기 칸 0.35 m 옆 로봇의 칸을 잡음), 경기장 씨드 8(첫 MoveCmd가 쥐지 않은 3 cm 칸을 지남). 자유바닥 기록에도 적혀 있다.
- 레인 v2가 모든 줄에서 일찍 끝나거나 같다(경기장 292.0 대 292.2 s). 자유바닥은 기다림이 짧고 길이 조금 짧지만(lane_map) 도크 접근이 느리다. 긴 실행과 창고에서는 자유바닥 길이 같거나 길다.
- tick: 레인 v2 평균 0.07~0.18 ms, p99 최대 2.25 ms. 자유바닥 평균 1.3~2.4 ms, p99 최대 11.9 ms. 둘 다 관제 주기 100 ms에 한참 못 미친다.
- 레인 v2 경기장 10/10, 292.0 s는 지시서의 레인 기준과 같다. 지시서의 이 레인 값은 자유바닥 문서에 없다: freefloor-route에서 compare_floor lanes로 자유바닥의 경기장 레인 그림을 돌리면 0/10이다(두 로봇이 BAY와 PS를 서로 기다려 격리). 그 지도는 자유바닥 길을 그린 것이라 레인용이 아니다. track C 지도로 돌린 이번 값이 레인 기준이다.

## 창고 대기 칸(간격 0.35 m)과 도크 앞 기다림

- 레인도 막히는가(지시서 4단계): 아니다. 레인 v2 창고 15/15, 최소 간격은 정확히 0.350 m(대기 칸 P-1~P-3에 나란히 선 로봇)이고 위반·교착 0. 구역 대기 시간을 구역별로 나눠 보면(씨드 1, 2, 탐침 스크립트) 대기 칸 구역(P-*)은 상위 8개 구역에 없다(각 1.3 s 미만). 자유바닥이 막히는 까닭은 몸 칸(반지름 + 여유 0.13 m, 0.1 m 칸)과 잡음(σ 0.03 m)이 0.35 m에 빠듯하기 때문이고, 레인은 지점 하나가 구역 하나라 몸 칸 겹침이 없다.
- 도크 앞 기다림이 큰가: 창고에서는 크다. 씨드 1: 구역 대기 350.9 s 중 OUTBOUND 도크 O 109.4 s, WORK 도크 W 72.6 s(합 52%), 나머지는 통로 TUN 91.7 s, A1 49.8 s 등. order_burst 씨드 1: 138.7 s 중 W 59.9 s(43%). 긴 실행 씨드 1: 493.4 s 중 W 95.1 s(19%), AVENUE 395.5 s.
- 제안(구현하지 않음): 도크 시간창. 도크를 쓰는 로봇이 끝날 예상 시각(작업 시간 = 품목 수 × 4 s, 교체 8 s, 도킹 3 s)을 알고 있으니, 다음 로봇이 대기 지점에 그보다 일찍 닿지 않게 출발을 늦추거나, 그 사이 다른 일(먼 쪽 도크)을 먼저 주는 방식. 다만 레인에서 대기 지점 기다림은 다른 로봇을 막지 않는 한 시간 손실이 아니다(도크가 비는 순간 바로 들어간다). 줄일 몫은 기다리는 로봇이 통로(TUN, AVENUE)를 막아 생기는 2차 대기다. 먼저 이 2차 대기를 따로 재 보는 것이 순서다.

## 실행 명령

WSL, Jazzy를 source한 뒤 저장소 뿌리에서:

```bash
python3 tools/baseline.py --modes path --maps lane_map --scenarios basic_orders order_burst blocked_lane --seeds $(seq 1 20) --out docs/v2_parity/lane_map.csv
python3 tools/baseline.py --modes path --maps warehouse --scenarios warehouse_rush --seeds $(seq 1 15) --out docs/v2_parity/warehouse.csv
python3 tools/baseline.py --modes path --maps lane_map --scenarios long_orders_100 --seeds 1 2 3 --out docs/v2_parity/long_orders_100.csv
python3 tools/baseline.py --modes path --maps real_arena --scenarios gz_real_arena --seeds $(seq 1 10) --speed 0.12 --noise 0.03 --params fleet_params_real_arena --out docs/v2_parity/real_arena.csv
```

레인 쪽은 전부 합쳐 몇 분이다(창고 15씨드가 한 프로세스로 약 2분).

## 시험

먼저 쓴 시험 11개가 빨강(11 failed, 2 passed)이었고, 구현 뒤 `test_baseline_tool.py` 55 passed. 같은 씨드 같은 행 시험은 tick 열(벽시계)을 빼고 비교하도록 고쳤다. 표 형식 시험은 빨강 1개 뒤 초록. 병합 뒤 경기장 줄 시험을 더해 `test_baseline_tool.py` + `test_eta_error_tool.py` 65 passed. 검토 뒤 사건 수 시험(빨강 1 → 초록)을 더해 66 passed. ruff 통과.

## Codex 검토

0688a6d..HEAD(07e114c까지), gpt-6.1-terra high:

- [확정] 불변식이 아닌 AssertionError도 위반 1로 센다: 맞다. compare_floor도 같은 식이라 맞춤을 위해 두고 docstring에 적었다(7659955).
- [확정] 깨진 단계는 최소 간격·거리에 들지 않는다: 맞다. compare_floor도 같다. docstring에 적었다.
- [확정] 사건 수 시험이 0만 본다: 맞다. `event_counts()`로 빼고 만든 사건으로 시험(7659955, 빨강 뒤 초록).
- 시나리오 폴더 사이 같은 이름은 지금 없음, 속도·잡음 채우기는 기존 시나리오를 바꾸지 않음, `--params`는 모드와 SCENARIO_PARAMS를 유지, p99 식 일치: 결함 없음.

짧은 재검토는 Codex가 "model at capacity"로 끝나지 못해 직접 했다: `event_counts`는 예전 네 식과 같은 코드를 센다. 시험의 detours와 deadlocks 수가 같아 둘을 바꿔도 통과했으므로, 수를 모두 다르게 고쳤다.

## 남은 일

- tick 비교는 같은 기계에서 겹쳐 돈 값이다. 깨끗한 비교가 필요하면 한가한 기계에서 두 쪽을 연달아 다시 돌린다.
- 도크 앞 2차 대기(기다리는 로봇이 통로를 막는 시간)를 따로 재는 열은 만들지 않았다.
