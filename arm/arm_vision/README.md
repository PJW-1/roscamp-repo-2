# arm_vision (비어 있음)

작업공간 팔(arm1, arm2)의 카메라로 상품을 인식합니다. 담당(제안): 로봇팔 비전. 지금 시험은 `fake_vision_node`가 대신합니다.

| 종류 | 이름 | 타입 | 상태 |
|---|---|---|---|
| 발행 | `/arm1/vision/items`, `/arm2/vision/items` | `ItemArray` | 계약에 있음. 받는 쪽은 팔 쪽에서 정함 |

영상은 ROS로 보내지 않습니다. 집기용 사진은 HTTP 스냅샷 한 장, 화면용은 MJPEG over HTTP입니다(공통설계 13절). arm3 인식(SR-27)은 아직 정하지 않았습니다.
