# arm_control

팔 PC에서 JetCobot을 움직인다.

- 액션 서버: `/workcell/fill_basket` (`FillBasket`, arm1·arm2가 함께 씀), `/arm3/swap_basket` (`SwapBasket`)
- 발행: `/arm1/state`, `/arm2/state`, `/arm3/state` (`ArmState`)

arm1과 arm2가 동시에 PLACING이면 안 된다.
