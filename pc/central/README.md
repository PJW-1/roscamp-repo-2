# central

주문·재고·화면 백엔드. 관제 PC에서 돈다.

- 발행: `/central/orders` (`OrderBoard`), `/central/events` (`FleetEvent`, FMS도 발행)
- 서비스: `/central/submit_order` (`SubmitOrder`), `/central/set_run_mode` (`SetRunMode`), `/central/report_order` (`ReportOrder`)
- 호출: `/fleet/confirm_clear` (`ConfirmClear`)
