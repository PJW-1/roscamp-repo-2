# central (비어 있음)

주문, 재고, 화면 백엔드를 맡습니다. 관제 PC에서 돕니다. 담당(제안): 중앙 시스템. 지금 시험은 `fake_robots`의 `fake_central_node`가 대신합니다.

| 종류 | 이름 | 타입 | 상태 |
|---|---|---|---|
| 발행 | `/central/orders` | `OrderBoard` | 계약에 있음 |
| 발행 | `/central/events` (FMS도 발행) | `FleetEvent` | 계약에 있음 |
| 서비스 서버 | `/central/submit_order`, `/central/set_run_mode`, `/central/report_order` | `SubmitOrder`, `SetRunMode`, `ReportOrder` | 계약에 있음 |
| 서비스 호출 | `/fleet/confirm_clear` | `ConfirmClear` | 계약에 있음 |
| 구독 | `/fleet/robots`, `/fleet/assignments`, `/fleet/zones`, `/{arm}/state` | | 계획 |

노드 이름 제안: `central`, 실행 파일 `central_node`. 이름 규칙은 `docs/저장소_구조와_이름.md`.
