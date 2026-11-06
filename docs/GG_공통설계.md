# GG 공통 설계

관제와 각 파트가 함께 따르는 설계 규칙입니다. 다른 문서에 있는 내용은 여기에 다시 적지 않습니다.

| 내용 | 문서 |
|---|---|
| 폴더, 장치 연결, 토픽·액션·서비스 이름과 타입, QoS | `README.md` |
| 핑키 FSM, 작업공간 서버, 출고 팔, 비전이 지킬 것 | `docs/구현_약속.md` |
| FMS 모듈 구조, 가짜 노드, 시나리오 형식, 값을 바꿀 곳 | `docs/관제_개발_참고.md` |
| 빌드, 실행, 이름 규칙 | `docs/개발_규칙.md` |

## 1. 시나리오

주문이 들어오면 핑키가 대기 구간에서 출발한다. 공유 작업공간의 로봇팔 2대가 핑키 뒤 트레이의 바구니에 주문 상품을 담는다. 핑키가 출고 구역으로 간다. 로봇팔 1대가 바구니째 꺼내고 빈 바구니를 트레이에 넣는다. 핑키는 대기 구간으로 돌아가거나, 주문이 밀려 있으면 바로 작업공간으로 간다.

- 작업공간 도크에는 핑키 1대만 들어간다. 나머지는 대기열에서 기다린다.
- 주문 1건 = 바구니 1개. 바구니당 최대 `basket_capacity`(4)개.
- 모든 핑키는 빈 바구니를 싣고 시작한다.
- 검수는 없다. 재고는 central이 관리하고 관리자가 손으로 고친다.
- 결품이면 담긴 것만 출고한다(`ship_partial`). 하나도 못 담으면 보류(HELD)하고 빈 바구니로 다음 일을 한다.

## 2. 용어

| 낱말 | 뜻 | 예 |
|---|---|---|
| FMS | 핑키에 일을 나눠 주고(배분), 길을 정리하고(교통), 스테이션에 작업을 요청하는 관제. 프로세스 1개, 노드 1개(`fleet_manager`) | |
| 모듈 | FMS 안의 파이썬 코드 묶음. 서로 파이썬 호출로 부르고 ROS를 쓰지 않는다 | `orders`, `traffic` |
| 플릿(fleet) | 핑키 3대 | `pinky1`~`pinky3` |
| 스테이션(station) | 작업공간과 출고. 도크와 대기 레인을 가진다 | `WORK`, `OUTBOUND` |
| 지점(waypoint) | 레인 그래프의 꼭짓점. ROS 노드와 헷갈리지 않게 "노드"라 부르지 않는다 | `W`, `P-4` |
| 레인(lane) | 지점에서 지점으로 가는 방향 있는 직선 | `Q`→`W` |
| 구역(zone) | 잠금 단위. 지점 하나와 그 지점으로 들어오는 레인들(3절) | `W`, `AVENUE` |
| 도크(dock), 대기 지점(parking) | 스테이션에서 일하려고 서는 지점, 일이 없을 때 서는 지점 | `W`, `P-3`~`P-6` |

"칸", "셀", "그리드"는 쓰지 않는다.

FMS 안에는 주문 진행을 맡는 `orders`, 교통을 맡는 `traffic`, 둘을 잇는 `FleetManager`가 있다. `traffic`은 주문과 바구니를 모른다. 팔의 동작은 스테이션(작업공간 서버, arm3)이 한다.

### 상태 값

| 값 | 어디 | 뜻 |
|---|---|---|
| `IDLE` | 임무 단계 | 지도에 올랐고 갈 곳이 없다. 호출이 있으면 가고 없으면 대기 지점으로 간다 |
| `PARKED` | 임무 단계 | 대기 지점에 있다. 더 안쪽 지점이 비면 전진한다 |
| `TO_PARK`, `TO_WORK`, `TO_OUTBOUND` | 임무 단계 | 대기 지점, 작업공간, 출고 구역으로 이동 중 |
| `DOCKING` | 임무 단계 | dock 목표 실행 중 |
| `AT_STATION` | 임무 단계 | 도크에 서서 다음 요청이나 놓아주기를 기다린다 |
| `NONE` | `fault` | 문제 없음 |
| `ERROR` | `fault` | 관리자가 봐야 한다. 이유는 `fault_reason` |
| `COMM_LOST` | `fault` | 보고가 `comm_timeout_s` 동안 없다. 판단은 멈추지 않고 그 로봇의 요청만 취소한다 |
| `RECOVERING` | `fault` | 보고가 돌아왔고 `recover_check_s` 뒤에 위치를 확인한다 |
| `OUT_OF_SERVICE` | `fault` | 관리자가 치웠다(`confirm_clear`). 되돌리는 동작은 없다 |
| `IDLE`, `MOVING`, `WAITING`, `DOCKING`, `DOCKED`, `ERROR` | `fsm_state` | 핑키가 보고한 값 그대로. 임무 단계와 이름이 같아도 다른 값이다 |
| `TO_WORK`, `FILL`, `TO_OUTBOUND`, `SWAP`, `WAIT` | `task_type` | `/fleet/assignments`와 `/fleet/robots`의 일 종류 |
| `PENDING`, `BOUND`, `FILLING`, `TO_OUTBOUND`, `SWAPPING`, `SHIPPED`, `PARTIAL`, `HELD` | 주문 상태 | 6절 |
| `EMPTY`, `FULL`, `NONE` | 바구니 | 빈 바구니, 담긴 바구니, 바구니 없음 |
| `RUN`, `PAUSE`, `STOP` | run_mode | 5절, 6절 |

### 이벤트 코드 (`/central/events`)

수준은 INFO, WARN, ERROR다. 심판은 시나리오가 예상하지 않은 ERROR를 실패로 센다.

| 코드 | 수준 | 내는 곳 | 뜻 |
|---|---|---|---|
| `DOCKED` | INFO | traffic | 도킹 끝. detail에 오차 |
| `GOAL_RETRY` | WARN | traffic | 이동이나 도킹 목표가 실패해 다시 보낸다 |
| `GOAL_FAILED` | ERROR | traffic | 재시도를 다 썼다 |
| `GOAL_NOT_SENT` | WARN | traffic | 액션 서버가 없어 목표를 보내지 못했다. 목표가 서버에 닿을 때까지 한 번만 낸다 |
| `COMM_LOST` | WARN | traffic | 보고가 `comm_timeout_s` 동안 없다 |
| `COMM_RESTORED` | INFO | traffic | 보고가 돌아왔다. 위치를 확인하고 이어 간다 |
| `ZONE_WAIT_LONG` | WARN | traffic | 구역 하나를 `zone_wait_warn_s` 넘게 기다린다 |
| `ZONES_CLEARED` | INFO | traffic | `confirm_clear`로 잠금을 풀었다 |
| `CLEAR_REFUSED` | WARN | traffic | `confirm_clear`를 거부했고 아무것도 바꾸지 않았다. 이유는 detail에 적는다 |
| `ZONES_STILL_HELD` | WARN | traffic | `confirm_clear` 뒤에도 그 로봇이 쥔 구역이 남았다. 다시 호출해야 풀린다 |
| `LANE_ERROR` | WARN | traffic | 이동 중 레인 이탈이 `lane_error_warn_m`을 넘었다(5절). 해제를 막지 않는다 |
| `DEADLOCK_DETECTED` | ERROR | traffic | 대기 고리를 찾았다 |
| `DEADLOCK_RETREAT` | ERROR | traffic | 로봇 1대가 아직 들어가지 않은 구역을 포기한다 |
| `DEADLOCK_UNRESOLVED` | ERROR | traffic | 풀지 못했다. 관리자가 봐야 한다 |
| `PLACEMENT_CONFLICT` | ERROR | traffic | 시작 때 다른 로봇이 쥔 구역이나 그 충돌쌍에 서 있다. 해결될 때까지 전원이 기다린다 |
| `RECOVERY_CONFLICT` | ERROR | traffic | 복구한 로봇이 서 있는 구역을 잡지 못했다(다른 로봇이 쥐었거나 먼저 기다리거나, 충돌 쌍 구역에 그런 로봇이 있다). 그 로봇만 ERROR가 되고 다른 로봇은 세우지 않는다(시작 때의 `PLACEMENT_CONFLICT`와 다르다) |
| `ROBOT_NOT_REPORTING` | ERROR | traffic | `startup_wait_s`가 지나도 지도에 오르지 않은 로봇이 있다. 전원이 기다린다 |
| `ORDER_BOUND` | INFO | orders | 주문을 로봇에 묶었다. detail은 주문 ID, source는 로봇 |
| `ORDER_REQUEUED` | INFO | orders | 치운 로봇의 주문을 대기 맨 앞으로 되돌렸다 |
| `FILL_RETRY`, `SWAP_RETRY` | WARN | orders | 팔 작업이 실패해 다시 보낸다 |
| `FILL_FAILED`, `SWAP_FAILED` | ERROR | orders | 재시도를 다 썼거나 담은 일부를 내보낼 수 없다 |
| `ARM_NO_RESULT` | ERROR | orders | 취소한 팔 작업의 결과가 끝내 안 왔다. 바구니 내용을 알 수 없다 |
| `ARM_LATE_RESULT` | WARN | orders | 포기했거나 치우면서 취소한 팔 작업의 결과가 늦게 왔다(6절) |
| `NO_ROBOT_AVAILABLE` | WARN | orders | 주문은 있는데 지도에 올라 고장이 아닌 로봇이 없다. 전원이 통신 두절이면 내지 않는다 |
| `CANCEL_UNCONFIRMED` | WARN | `fleet_manager` 노드 | 취소를 보냈는데 결과가 오지 않는다(6절). 목표당 한 번 |
| `ORDER_REPORT_REFUSED` | ERROR | `fleet_manager` 노드 | central이 주문 보고에 `accepted=false`로 답했다. 다시 보내지 않는다 |
| `ORDER_ACCEPTED`, `ORDER_SHIPPED` | INFO | central | 주문을 받았다, 출고했다 |
| `ORDER_REJECTED`, `ORDER_PARTIAL`, `ORDER_HELD` | WARN | central | 거절, 일부 출고, 보류 |
| `OUT_OF_STOCK`, `STOCK_MISMATCH` | WARN | central | 재고 기록도 0, 기록은 있는데 실제로 없음 |

주문 관련 코드의 detail은 주문 ID로 시작한다.

### 파라미터 (`pc/fleet_core/fleet_core/data/fleet_params.yaml`)

| 이름 | 값 | 뜻 |
|---|---|---|
| `basket_capacity` | 4 | 바구니 하나에 담는 최대 개수 |
| `ship_partial` | true | 일부만 담겨도 출고한다 |
| `robot_radius_m` | 0.08 | 몸체 반경 r. 트레이를 뺀 값 |
| `release_margin_extra_m` | 0.05 | 심판 전용. r + 이 값이면 코스 밖이다 |
| `release_distance_m` | 0.2 | 해제 거리 d(5절) |
| `release_confirm_samples` | 3 | 해제 전에 연속으로 확인하는 보고 수 |
| `lane_error_warn_m` | 0.15 | `LANE_ERROR`를 내는 이탈 e |
| `move_retry_limit` | 2 | 첫 실패 뒤 재시도 수. 이동, 도킹, 팔 작업이 같이 쓴다 |
| `move_retry_wait_s` | 2 | 이동이나 도킹 실패 뒤 다시 보내기까지 |
| `arrival_tolerance_m` | 0.15 | 이동 성공을 도착으로 치는 목표 지점까지의 거리. 가장 짧은 레인 길이에서 d를 뺀 값(기본 지도 0.2 m)보다 작아야 한다 |
| `arrival_settle_s` | 1 | 성공 결과 뒤 위치 보고를 기다리는 시간. 보고 지연 τ보다 길어야 한다 |
| `startup_wait_s` | 10 | `ROBOT_NOT_REPORTING`을 내기까지 |
| `comm_timeout_s` | 3 | `COMM_LOST`로 보기까지 |
| `heartbeat_period_s` | 0.5 | `/fleet/heartbeat` 주기. 벽시계 |
| `heartbeat_timeout_s` | 1.5 | 로봇이 멈추기까지. 벽시계 |
| `recover_check_s` | 1 | 통신 복구나 교착 포기 뒤 위치를 확인하기까지 |
| `arm_cancel_wait_s` | 5 | 취소한 팔 작업의 결과를 기다리는 시간 |
| `arm_retry_wait_s` | 2 | 팔 실패 뒤 다시 보내기까지 |
| `fill_timeout_s` | 90 | 담기 시간 제한 |
| `swap_timeout_s` | 60 | 교환 시간 제한 |
| `cancel_retry_s` | 1 | 결과가 없을 때 취소를 다시 보내는 간격 |
| `cancel_warn_after` | 5 | `CANCEL_UNCONFIRMED`까지의 재시도 주기 수. `cancel_retry_s`를 곱한 값이 `arm_cancel_wait_s` 이상이어야 한다 |
| `report_retry_s` | 2 | 답이 없는 주문 보고를 다시 보내기까지 |
| `zone_wait_warn_s` | 60 | `ZONE_WAIT_LONG`을 내기까지 |
| `deadlock_grace_s` | 5 | 대기 고리를 찾고 포기 대상을 정하기까지 |
| `score_same_call` | -4 | 이미 호출된 로봇의 점수 보정 |
| `score_distance_weight` | 0.91 | 레인을 따른 거리 1 m당 점수 |
| `priority_at_station`, `priority_has_cargo` | 200, 100 | 교착 때 포기 대상을 고르는 우선순위 |

## 3. 지도 (`pc/fleet_core/fleet_core/data/lane_map.yaml`)

실제 지도가 나오면 이 파일만 바꾼다. 로봇은 지점 사이를 어느 방향으로든 직선으로 간다. 대각선을 못 가는 점유 격자가 아니다.

지도 파일: 지점(`waypoints:`, id, x, y), 방향 있는 레인(from, to. 길이를 안 적으면 좌표로 계산), `two_way`(양방향 구간), 스테이션(dock, queue, commit, server, arms, action), parking(대기 지점, 주행 순서대로).

기본 지도: 지점 14개, 방향 있는 레인 17개, 구역 13개.
- `WORK`: 도크 W, 대기 레인 L→Q, Q→W, 서버 `workcell`, 팔 arm1·arm2, 동작 `fill`
- `OUTBOUND`: 도크 O, 대기 레인 R→QO, QO→O, 서버 `arm3`, 동작 `swap`
- 대기 지점 P-3~P-6. 양방향 구간 `AVENUE`(A↔B, 1.1 m). N→S가 대로를 지점 없이 가로지른다.
- `half_width_m` 0.05(레인에서 이만큼까지 레인 위로 친다), `lane_entry_m` 0.25, `bundle_two_way` true.

규칙:
- **구역:** 지점마다 구역 하나다. 레인은 끝 지점의 구역에 속한다. 한 지점으로 들어오는 레인들은 잠금 하나와 FIFO 하나를 함께 쓴다.
- **양방향 구간:** `two_way`로 적은 한 쌍(a→b, b→a)은 두 끝 지점의 구역을 하나로 합친다(`bundle_two_way`). 같은 쌍을 레인 두 줄로 적으면 지도를 거부한다. 대로 전체가 잠금 하나이므로 대로에서 멈춘 로봇 1대는 대로를 지나는 이동을 모두 막는다.
- **충돌쌍:** 두 구역의 레인이 2r보다 가까이 지나면 동시 점유 금지 쌍으로 자동 계산한다. 지점을 공유하는 레인은 그 지점을 떠나는 쪽의 처음 d를 빼고 잰다. 양방향 쌍 자신은 뺀다. 기본 지도의 충돌쌍은 `{AVENUE, S}` 하나다.
- 비교용 지도는 `sim/fake_robots/maps/`(`oneway_loop`, `oneway_shortcut`)에 있다.
- 지도를 바꾸면 전수 검사(`tools/model_check.py`)로 교착과 기아를 다시 확인한다.

## 4. 인터페이스 동작

계층: 화면 → central → FMS → 핑키 / 작업공간 서버(arm1·arm2) / arm3. 위가 아래를 부르고 아래는 상태와 결과만 올린다. central은 로봇을 직접 움직이지 않고, 로봇은 central을 모른다. 핑키 파트는 팔을 모르고 팔 파트는 핑키를 모른다.

### FMS 안의 호출

`orders`와 `traffic` 사이에는 ROS 액션, 서비스, 토픽을 두지 않는다. 파이썬 호출만 쓴다.

- `on_dispatch(goal_id, station_id, robot_id="")`: orders → traffic. 로봇을 스테이션으로 보내 달라는 요청. `robot_id`가 비면 traffic이 고른다.
- `on_dispatch_cancel(goal_id)`: orders → traffic. 끝나지 않은 요청을 취소한다.
- `on_release(robot)`: orders → traffic. 스테이션 일이 끝난 로봇을 놓아준다.
- `DispatchFeedback(goal_id, robot_id, phase)`: traffic → orders. `phase`는 `NO_ROBOT` 하나뿐이다.
- `DispatchResult(goal_id, success, robot_id, reason, error_x_cm, error_y_cm, error_yaw_deg)`: traffic → orders. success면 그 로봇은 도크에 서서(AT_STATION) 다음 `on_dispatch`(robot_id 지정)나 `on_release`를 기다린다. 실패 `reason`: `NOT_AVAILABLE`, `BUSY`, `CANCELED`, `ROBOT_REMOVED`, `REJECTED`(거절이 재시도 한도까지), `DOCK_FAILED`(도킹 실패가 재시도 한도까지). 이동 실패가 한도에 닿으면 이동 결과의 `reason`을, ERROR가 되면 그 이유를 그대로 넣는다.

`FleetManager`가 따로 전달하는 것:
- `on_run_mode(mode)`: STOP/PAUSE/RUN을 두 모듈에 각각 넘긴다.
- `on_robot_removed(robot)`: `confirm_clear`가 받아들여지면 orders에 알린다.
- `on_robot_fault(robot, reason)`: 팔 재시도가 소진되어 orders가 ERROR로 판단하면 traffic에 알린다.

`FleetManager`는 한 호출 동안 시각을 한 번만 읽는다. `fleet_manager_node --record PATH`(`pc.launch.py record:=PATH`)로 모든 입력과 결과를 JSON 한 줄씩 남기고 `tools/replay.py`로 재생한다. 노드가 직접 내는 `CANCEL_UNCONFIRMED`, `ORDER_REPORT_REFUSED`는 기록에 없다.

### 바깥 인터페이스

- `/central/submit_order`: 재고 기록에 없거나 용량을 넘으면 거절한다.
- `/central/orders`: run_mode와 최종 보고가 안 된 주문을 오래된 순서로 담는다. FMS는 run_mode를 여기서 받는다.
- `/central/report_order`: FMS는 `accepted=true`를 받을 때까지 `report_retry_s`마다 다시 보낸다. 그래서 central은 같은 보고가 여러 번 와도 결과가 같아야 한다. 이미 처리한 주문이면 재고를 다시 빼지 않고 `accepted=true`로 답한다. `accepted=false`는 다시 보내도 소용없을 때만 쓴다. FMS는 그 보고를 버리고 `ORDER_REPORT_REFUSED`를 낸다.
- `/central/events`: FMS와 central이 함께 발행하고 source로 구분한다.
- `/fleet/robots`: 1 Hz. 로봇마다 robot_id, fsm_state, task_type, order_id, fault, fault_reason, wait_zone, wait_robots, battery, x, y, yaw. 위치, 배터리, fsm_state는 마지막 보고 값이다. 화면은 핑키 상태를 위에서부터 먼저 맞는 것으로 나타낸다: `fault` ≠ NONE이면 오류 + `fault_reason`, `task_type`이 FILL이면 담기 대기, SWAP이면 출고 대기, `fsm_state`가 DOCKING이면 도킹, MOVING이면 이동, 나머지는 대기.
- `/{pinky}/move_to`: 레인 끝까지의 짧은 목표다(5절).
- 결과 reason: `MoveTo`, `FillBasket`, `SwapBasket` 결과에는 `REASON_CANCELED`("CANCELED")가 있다. 서버가 reason을 비워 두면 FMS가 목표 상태로 채운다(취소면 `CANCELED`, 중단이면 `ABORTED`). 서버가 쓴 reason은 그대로 둔다.
- `/{arm}/state`, `/{arm}/vision/items`는 FMS의 판단에 쓰지 않는다.
- `/{pinky}/navigate_to_pose`는 핑키 안에서만 쓰고 FMS는 부르지 않는다.

### heartbeat (`/fleet/heartbeat`)

FMS가 죽거나 통신이 끊기면 핑키가 스스로 서게 하는 장치다. 비상 정지 경로를 대신하지 않는다.

핑키 쪽:
- 이동이나 도킹 목표를 실행하는 동안, 마지막 heartbeat 또는 목표 수락에서 `heartbeat_timeout_s`(1.5 s, heartbeat 3번)가 지나면 그 목표를 취소하고 선다. 새 FMS의 첫 목표가 첫 heartbeat보다 먼저 올 수 있으므로 목표 수락도 FMS가 살아 있다는 증거로 친다.
- 할 일이 없는 로봇은 heartbeat가 없어도 아무것도 하지 않고 상태 보고를 계속한다. 멈춘 뒤 온 새 목표는 받는다.
- 시간은 벽시계로 잰다(시뮬 시계가 아님).
- 통신 두절도 같다. FMS가 살아 있어도 heartbeat를 못 받으면 선다.

FMS 쪽:
- 시작한 뒤 `heartbeat_timeout_s + heartbeat_period_s`(2 s) 동안 heartbeat를 내지 않고 상태 보고도 받지 않는다. 이전 FMS의 목표로 달리던 로봇은 그 사이에 멈춘다.
- 그 뒤 로봇의 `move_to`, `dock` 서버를 찾았고, 전원이 위치를 보고했고, `MOVING`이나 `DOCKING`이 아니어야 지도에 올리고 움직이기 시작한다.
- heartbeat는 전원이 지도에 오른 뒤에야 낸다. 늦게 수락된 이전 목표가 새 heartbeat를 자기 것으로 알고 계속 달리는 일을 막는다.

보장하지 않는 것: 마지막 heartbeat 뒤 도착한 목표는 수락 시각부터 다시 센다. 그래서 이전 FMS의 목표가 늦게 도착하면 그 로봇은 `heartbeat_timeout_s`까지 움직일 수 있다. heartbeat는 best effort라 3번 연달아 잃으면 FMS가 살아 있어도 선다. 감속 거리는 timeout에 들어 있지 않다.

## 5. 교통 규칙 (`traffic`, 핑키만)

시간·공간 예약, MAPF, Open-RMF는 쓰지 않는다. 구역 잠금으로만 막는다.

### 이동 명령

관제가 레인 끝까지의 짧은 `move_to`를 보낸다. 같은 방향으로 이어지는 레인은 잠금을 가진 데까지 한 목표로 묶고, 방향이 바뀌면 끊는다. Nav2는 점과 점 사이만 간다.

### 잠금과 해제

- 잠금은 현재 구역과 다음 구역 1개까지다. 로봇은 현재 구역 말고는 쥔 것이 없을 때만 다음 구역을 청한다. 그래서 기다리는 구역은 많아야 하나다(`wait_zone`).
- 구역마다 FIFO다. 충돌쌍은 쌍 전체의 요청 순서를 따른다.
- 해제: 로봇 위치 p를 그 구역을 떠나는 레인(시작 지점 p_i, 단위 벡터 u)에 투영해 s = (p − p_i)·u를 구한다. s > d(`release_distance_m`)가 `release_confirm_samples`번 연속이면 해제한다. 떠나는 레인이 경로에 없으면(앞쪽 구역, 손으로 옮긴 로봇) 구역 중심선에서 d 이상이어야 한다.
- d를 정하는 식: d ≥ r + ε + v·(τ + tick), d ≥ 2r + ε. r은 반경, ε은 위치 오차, v는 속도, τ는 보고 지연, tick은 관제 주기다.
- d ≤ 2r이거나, 구역을 떠나는 가장 짧은 레인이 d 이하이면 `TrafficController`가 시작을 거부한다. 기본 지도의 가장 짧은 레인은 0.4 m다.
- 이탈 e = |(p − p_i) − s·u|는 해제 조건이 아니다. 이동 목표 중에 e가 `lane_error_warn_m`을 넘는 보고가 `release_confirm_samples`번 이어지면 목표당 한 번 `LANE_ERROR`를 낸다. 경로의 첫 지점으로 가는 목표는 보지 않는다. 목표를 연장하거나 다시 보내면 횟수를 0부터 센다.

### 시작과 재시작

- 전원이 지도에 올라야 움직이기 시작한다. 로봇은 위치를 보고하고 `MOVING`이나 `DOCKING`이 아닐 때 지도에 오른다.
- `startup_wait_s`가 지나도 안 오른 로봇이 있으면 `ROBOT_NOT_REPORTING`을 낸다. `PLACEMENT_CONFLICT`로 거부된 로봇은 세지 않는다.
- 다른 로봇이 쥔 구역이나 그 충돌쌍에 서 있으면 지도에 올리지 않고 `PLACEMENT_CONFLICT`를 낸다. 그 로봇이 움직이거나 관리자가 치울 때까지 전원이 기다린다.
- 지점에서 d보다 멀리 서 있으면 먼저 그 레인의 끝으로 간다.
- 전원이 오르기 전에는 `/fleet/zones`를 내지 않는다. 잠금을 모르는 것과 빈 것은 다르다.
- FMS를 재시작하면 진행 중이던 주문, 할당, 바구니를 기억하지 않는다. 전원 보고부터 다시 시작한다.

### 교착

- 대기 고리가 있는 동안 새 호출과 새 이동 단계를 내지 않는다. 이미 보낸 이동 목표는 그대로 두고 포기 대상 로봇만 목표를 취소한다.
- 고리가 생기고 `deadlock_grace_s`가 지나면 우선순위가 가장 낮은 로봇 1대에서 아직 들어가지 않은 구역을 포기 대상으로 정하고 이동 목표를 취소한다(`DEADLOCK_RETREAT`). 들어간 구역에서 물러나는 동작은 없다.
- 실제 해제는 `recover_check_s` 뒤 위치 보고로 벗어난 것이 확인된 구역만 한다.
- 포기할 구역이 없으면 바로 `DEADLOCK_UNRESOLVED`를 낸다. 재확인 때 아무것도 해제하지 못했고 모든 대상이 해제 여유 안에 있으면 `DEADLOCK_UNRESOLVED`를 낸다. 대상 중 하나라도 해제 여유 밖에 있으면 재확인을 반복한다.
- 우선순위는 스테이션에 서 있는지(AT_STATION, `priority_at_station`)와 출고 구역으로 가는 요청을 받았는지(`priority_has_cargo`)만 본다.

### 호출

- 로봇 이름 없는 `on_dispatch(goal, WORK)`는 열린 호출로 쌓인다. 매 주기 열린 호출 수만큼 로봇을 고른다.
- 후보는 할당이 없는 사용 가능 로봇이다(AT_STATION이면 `on_release`를 받은 로봇). 점수가 가장 낮은 로봇을 고른다. 점수는 담기 도크까지 레인을 따른 거리 × `score_distance_weight`이고, 이미 호출된 로봇이면 `score_same_call`을 더한다. 호출은 주기마다 다시 계산하므로 다른 로봇이 이 보정을 넘을 만큼 가까워지면 호출이 옮겨 간다.
- 대기 지점에서는 추월할 수 없으므로 앞 로봇부터 호출한다. 고장 난 로봇 뒤의 대기 로봇은 호출하지 않는다.
- 고를 로봇이 없으면 호출을 연 채 기다린다. 지도에 올라 고장이 아닌 로봇이 하나도 없을 때만 `NO_ROBOT` 피드백을 보낸다(전원 통신 두절이면 보내지 않는다).
- 호출이 취소됐는데 그 로봇이 스테이션 `commit` 지점(기본 지도는 W)의 구역을 쥐기 전이면 대기 구간으로 돌린다. 쥐었으면 끝까지 가서 도킹 없이 대기 구간으로 간다.
- 도킹을 마치면 `DispatchResult`(success, robot_id, 도킹 오차)를 돌려주고 AT_STATION이 된다. 다음 요청이나 `on_release`가 올 때까지 도크를 비우지 않는다.
- `on_dispatch(goal, OUTBOUND, robot_id=X)`는 지정한 로봇을 보낸다. `on_release(X)`를 받은 로봇은 다음 주기의 호출 후보가 된다. 뽑히면 바로 작업공간으로 가고(작업 도크에 서 있었으면 그 자리에서 결과를 돌려준다), 안 뽑히면 대기 구간으로 간다.

### run_mode, confirm_clear, 재시도

- STOP: 이동 목표를 취소한다. 열린 호출은 그대로 두고 RUN이 되면 이어서 간다.
- PAUSE: 진행 중인 이동은 끝내고 새 이동을 내지 않는다.
- `confirm_clear`(관리자가 로봇을 치웠다): 구역 해제, OUT_OF_SERVICE, 이동 목표 취소. 그 로봇의 요청이 진행 중이면 실패(`ROBOT_REMOVED`)로 돌려준다. 주문을 되돌리는 것은 orders가 한다.
- `confirm_clear` 거부(`accepted=false`, `CLEAR_REFUSED`, 아무것도 바꾸지 않음): 없는 로봇, 목록에 없는 구역 이름이 하나라도 있을 때, 구역을 쥔 로봇인데 목록이 비었을 때. 구역을 쥐지 않은 로봇의 빈 목록은 정상이다. 쥔 구역의 일부만 보내면 받아들이고 `ZONES_STILL_HELD`를 낸다.
- 이동과 도킹 재시도: 목표가 실패하면(거절과 현재 목표의 `CANCELED`도 실패) 시도 횟수를 늘리고 `move_retry_wait_s` 동안 그 로봇에 새 목표를 보내지 않는다. 달리는 목표를 연장하는 것은 막지 않는다. 기다리는 동안 구역은 쥐고 있다. 세 번째 시도까지 실패하면 ERROR(`GOAL_FAILED`)다.
- 액션 서버가 없어 보내지 못한 이동과 도킹 목표는 실패한 시도로 세지 않는다. 같은 시간을 기다렸다가 다시 보낸다. 팔 목표는 `REJECTED`로 시도 횟수에 들어간다.

### 도착 판정

- 이동의 성공 결과가 와도 마지막 보고 위치가 목표 지점에서 `arrival_tolerance_m`보다 멀면 바로 도착으로 치지 않는다. `arrival_settle_s` 동안 새 목표를 내지 않고 위치 보고를 기다린다.
- 그 안에 허용치 안의 위치가 보고되면 도착이다. 기한이 지나도 멀면 실패한 시도(`NOT_AT_TARGET`)이고 처리는 실패한 이동과 같다.
- 성공 뒤 위치 보고가 하나도 오지 않았으면 판정하지 않고 기다린다. 보고가 끊긴 로봇은 `COMM_LOST`가 먼저 나고 복구가 위치를 판단한다.
- `COMM_LOST`나 `RECOVERING`인 로봇, 도킹 결과는 위치를 보지 않는다.

## 6. 주문 진행 규칙 (`orders`)

### 묶기

- 주문은 핑키가 WORK 도크에 도킹을 마친 순간 묶는다. WORK 요청의 결과가 success로 오면 그 로봇에 대기 주문 맨 앞을 묶고 `ORDER_BOUND`를 낸다. 그 전에는 호출만 한다. 그래서 주문이 등록 순서대로 처리되고, 도중에 고장 난 로봇이 주문을 쥐고 있지 않는다.
- 열어 두는 WORK 요청 수 = 아직 묶이지 않은 주문 수. 줄면 남는 요청을 `on_dispatch_cancel`로 취소한다.

### 담기 (FILLING)

1. 남은 품목 = 주문 품목 − 담은 것 − 최종 결품. 남은 것이 있으면 작업공간 서버에 `fill_basket`을 보낸다.
2. success=true: placed, missing을 누적하고 끝낸다. success=false: placed만 누적하고 재시도한다. `move_retry_limit`을 넘으면 ERROR이고 주문은 묶인 채 둔다.
3. 담은 것이 1개 이상이면 바구니 FULL, `on_dispatch(OUTBOUND, robot_id=X)`로 출고 구역에 보낸다. 0개면 HELD 보고, 주문을 풀고 바구니 EMPTY로 `on_release(X)`.
4. 주문이 `basket_capacity`를 넘으면 도킹 뒤 팔을 부르지 않고 HELD(detail에 사유).
5. `ship_partial`이 false면 일부만 담겨도 보내지 않고 HELD한다. 담긴 품목은 detail에 적고 ERROR로 처리한다.

### 교환 (SWAPPING, OUTBOUND 도킹 뒤)

1. `swap_basket`(take_full = 바구니가 FULL, put_empty = true)을 arm3에 보낸다.
2. took_full이면 바구니 NONE, **바로** SHIPPED(결품이 있었으면 PARTIAL) 보고 후 주문을 푼다. put_empty_done이면 바구니 EMPTY.
3. 남은 단계만 재시도한다. 한도를 넘으면 ERROR(`SWAP_FAILED`). 바구니가 EMPTY가 되면 `on_release(X)`. 다음 행선지는 traffic이 정한다.

### 팔 실패, 취소, 시간 초과

- 이 FMS가 STOP으로 취소한 작업의 `CANCELED`는 시도 횟수에 넣지 않는다. 시간 초과로 취소했거나 서버가 스스로 취소한 것은 실패로 센다.
- 실패나 거절 뒤에는 `arm_retry_wait_s`를 기다렸다가 다시 보낸다. 취소한 작업이 있으면 그 결과가 오거나 `arm_cancel_wait_s`가 지날 때까지 같은 서버에 다음 작업을 보내지 않는다.
- `fill_timeout_s`/`swap_timeout_s`를 넘으면 취소를 보내고 `arm_cancel_wait_s` 동안 결과를 기다린다. STOP으로 취소할 때도 같다. 결과가 오면 그대로 처리하고, 안 오면 ERROR(`ARM_NO_RESULT`)다.
- `ARM_NO_RESULT` 뒤에 온 결과는 `ARM_LATE_RESULT`로 보고 내용을 알린다. 주문이 아직 그 로봇에 묶여 있으면 반영한다(placed는 주문에 더하고, took_full이면 SHIPPED/PARTIAL로 보고하고 주문을 푼다). 로봇은 ERROR 그대로이고 놓아주지 않는다. 이미 로봇을 치웠거나 주문을 보고한 뒤면 이벤트만 낸다.

### 취소와 보고 확인 (`fleet_manager` 노드)

- 취소는 그 목표의 결과가 와야 끝난 것으로 친다(이동, 도킹, 팔 목표 공통). 결과가 올 때까지 `cancel_retry_s`마다 취소를 다시 보낸다. 처음 보낸 것을 포함해 `cancel_warn_after`번이 답 없이 지나면 `CANCEL_UNCONFIRMED`를 한 번 내고 계속 보낸다.
- 서버가 수락하기 전에 요청한 취소는 수락되는 순간 보내고, 수락을 기다리는 시간도 횟수에 넣는다.
- 주문 보고는 `accepted=true`를 받을 때까지 노드가 가지고 있다가 `report_retry_s`마다 다시 보낸다. 서비스가 안 보이는 동안은 보내지 않는다. `accepted=false`면 버리고 `ORDER_REPORT_REFUSED`를 낸다. 그 주문은 아무도 맡지 않으므로 관리자가 처리한다.

### run_mode와 로봇 제거

- STOP: 팔 작업을 취소한다. RUN으로 돌아오면 남은 일부터 다시 한다.
- PAUSE: 진행 중인 것은 끝내고 새 명령을 내지 않는다.
- 로봇 제거(`on_robot_removed`): 묶인 주문이 출고 전이면 대기 주문 맨 앞으로 되돌리고 `ORDER_REQUEUED`를 낸다. detail에 담겨 있던 품목을 적는다. 실제 바구니와 재고는 관리자가 확인한다. 진행 중이던 팔 작업은 취소하고 위의 취소 대기를 따른다. 그 작업의 늦은 결과는 반영하지 않고, 무엇을 했다고 하면 `ARM_LATE_RESULT`로 알린다. 같은 제거가 두 번 와도 주문은 한 번만 되돌린다.

## 7. 하드웨어와 실행 위치

| 장치 | 대수 | 구성 | 실행하는 소프트웨어 | 네트워크 |
|---|---|---|---|---|
| 관제 PC | 1대 이상 | | 화면, central, FMS(`fleet_manager`) | 유선 |
| 비전 서버 | 1대 | | arm_vision, pinky_vision | |
| 공유기 | 전용 1대 | | | |
| Pinky Pro (pinky1~3) | 3대 | Raspberry Pi 4B(4 GB), RPLidar C1, Dynamixel 2개, 카메라 OV5647, BNO055, 배터리 ADC | pinky_bringup, pinky_fsm, pinky_dock, Nav2 + AMCL | Wi-Fi |
| JetCobot 작업공간 (arm1, arm2) | 2대 | MyCobot 280(6축 + 그리퍼), 카메라 640×480, 팔마다 팔 PC 1대 | 팔마다 arm_control. 작업공간 서버는 두 팔 PC 중 1곳 | 유선 |
| JetCobot 출고 (arm3) | 1대 | 같은 구성 | arm_control(`swap_basket` 제공) | 유선 |

Nav2는 핑키 Pi에서, 작업공간 서버는 팔 PC에서, arm_vision과 pinky_vision은 비전 서버에서 돈다. 핑키와 팔의 카메라 영상은 HTTP로 비전 서버에 간다.
