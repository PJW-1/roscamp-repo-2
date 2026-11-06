# pinky_fsm (비어 있음)

핑키 한 대의 이동·도킹 명령을 받아 Nav2로 넘기고 상태를 보고합니다. 핑키 Pi에서 namespace `/{pinky}` 아래로 돕니다. 담당(제안): 핑키 제어·FSM. 지금 시험은 `fake_robots`의 `fake_pinky_node`가 대신하고, 이 패키지는 그와 같은 계약을 지켜야 합니다.

| 종류 | 이름 | 타입 | 상태 |
|---|---|---|---|
| 액션 서버 | `/{pinky}/move_to` | `MoveTo` | 계약에 있음. goal의 `path`는 지시서 7번 예정 |
| 액션 서버 | `/{pinky}/dock` | `Dock` | 계약에 있음 |
| 발행 | `/{pinky}/state` (10 Hz 권장) | `RobotState` | 계약에 있음. `BLOCKED`는 지시서 4번 예정 |
| 구독 | `/fleet/heartbeat` | `std_msgs/Header` | 제안 |
| 안에서 부름 | `/{pinky}/follow_path`(path 모드), `/{pinky}/navigate_to_pose`(point 모드) | Nav2 | 핑키 내부 |

지킬 약속은 루트 `README.md` "실제 FSM이 지켜야 하는 약속"과 Drive `핑키_제어파트_전달사항_2026-11-06` 3절(R1~R19)에 있습니다. 레인 그래프는 몰라도 됩니다. 선은 `MoveTo.path`로 옵니다.

노드 이름 제안: `pinky_fsm`, 실행 파일 `pinky_fsm_node`. 이 패키지는 `pc/*`를 import하지 않습니다.
