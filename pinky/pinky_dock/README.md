# pinky_dock (비어 있음)

스테이션(`WORK`, `OUTBOUND`) 도크에 정밀하게 붙는 동작입니다. 핑키 Pi에서 돕니다. 담당(제안): 핑키 제어·FSM. `pinky_fsm`의 `/{pinky}/dock` 서버가 이 패키지를 부릅니다.

- 목표 정밀도 ±1~2 cm(팔 도달 거리 약 20~25 cm, 전달사항 R11).
- 결과의 `error_x_cm`, `error_y_cm`, `error_yaw_deg`의 기준 프레임과 부호는 아직 정하지 않았습니다(전달사항 7절 질문 9).

노드 이름 제안: `pinky_dock`, 실행 파일 `pinky_dock_node`.
