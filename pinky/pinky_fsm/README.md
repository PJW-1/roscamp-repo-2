# pinky_fsm

핑키 한 대의 이동·도킹 명령을 받아 Nav2로 넘기고 상태를 보고한다. 핑키 Pi에서 namespace `/{pinky}`로 돈다.

- 액션 서버: `move_to` (`MoveTo`), `dock` (`Dock`)
- 발행: `state` (`RobotState`, 10 Hz)
- 구독: `/fleet/heartbeat`. 끊기면 멈춘다.

레인 그래프는 모른다. `pc/*`를 import하지 않는다. 지킬 약속은 루트 `README.md`의 "실제 FSM이 지켜야 하는 약속".
