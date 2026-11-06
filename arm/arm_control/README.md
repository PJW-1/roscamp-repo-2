# arm_control

팔 PC에서 JetCobot을 움직인다.

- 액션 서버: `/workcell/fill_basket` (`FillBasket`, arm1·arm2가 함께 씀), `/arm3/swap_basket` (`SwapBasket`)
- 발행: `/arm1/state`, `/arm2/state`, `/arm3/state` (`ArmState`)

arm1과 arm2가 동시에 PLACING이면 안 된다. 나머지 약속(`placed`, 취소, 상태 시각)은 루트 `README.md`의 "실제 작업공간 서버와 출고 팔이 지켜야 하는 약속".
