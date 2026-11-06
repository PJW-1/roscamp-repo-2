# arm_control (비어 있음)

팔 PC에서 JetCobot을 움직입니다. 담당(제안): 로봇팔 제어. 지금 시험은 `fake_workcell_node`(arm1, arm2)와 `fake_arm_node`(arm3)가 대신합니다.

| 종류 | 이름 | 타입 | 상태 |
|---|---|---|---|
| 액션 서버 | `/workcell/fill_basket` (작업공간 서버, 팔 2대가 하나를 씀) | `FillBasket` | 계약에 있음 |
| 액션 서버 | `/arm3/swap_basket` | `SwapBasket` | 계약에 있음 |
| 발행 | `/arm1/state`, `/arm2/state`, `/arm3/state` | `ArmState` | 계약에 있음 |

작업공간 서버(workcell)를 이 패키지에 둘지 새 패키지로 뺄지는 팔 쪽에서 정합니다. arm1과 arm2가 동시에 PLACING이면 안 됩니다(루트 `README.md` "실제 작업공간 서버와 출고 팔이 지켜야 하는 약속").
