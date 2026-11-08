"""버전 2 준비 구조(V2-0). 동작은 그대로이고, 뒤 단계가 쓸 길만 연다.

- 레인 밖 짧은 동작의 goal 종류(RETREAT, DETOUR)는 레인 이동 감시가 다루지 않는다.
- "현재 + 다음 구역만 쥔다" 규칙은 controller._needed_zones 한곳에 있고, 빌린 구역
  (_Mission.borrowed)은 그 규칙의 예외다. 빌린 구역은 경로 위에 없어도 풀리지 않는다.
"""
from fleet_core.traffic.mission import DETOUR, DOCK, MOVE, RETREAT, _Goal
from test_lane_blocked import Pilot


def on_the_way(pilot):
    pilot.run(0.3)
    pilot.put("pinky1", "P-6", "N", 0.4)
    pilot.run(1.0)
    assert pilot.held() == {"N", "S"}


def test_goal_kinds_are_distinct():
    assert len({MOVE, DOCK, RETREAT, DETOUR}) == 4


def test_a_borrowed_zone_off_the_path_is_kept_until_it_is_given_back():
    pilot = Pilot()
    on_the_way(pilot)
    zones = pilot.core.zones
    assert zones.request("pinky1", "O")
    pilot.mission().borrowed = {"O"}
    pilot.run(2.0)
    assert "O" in pilot.held(), "빌린 구역은 경로 위에 없어도 쥔다"
    pilot.mission().borrowed = set()
    pilot.run(2.0)
    assert "O" not in pilot.held(), "돌려준 뒤에는 지금의 해제 규칙으로 놓는다"


def test_without_borrowing_a_zone_off_the_path_is_released_as_before():
    pilot = Pilot()
    on_the_way(pilot)
    assert pilot.core.zones.request("pinky1", "O")
    pilot.run(2.0)
    assert "O" not in pilot.held()


def test_a_borrowed_zone_does_not_stop_the_next_request():
    pilot = Pilot()
    on_the_way(pilot)
    m = pilot.mission()
    assert pilot.core.zones.request("pinky1", "O")
    m.borrowed = {"O"}
    traffic = pilot.core.traffic
    assert traffic._needed_zones(m) == {"N", "S", "O"}
    # 현재 구역 말고 쥔 것이 빌린 구역뿐이면 다음 구역을 청할 수 있다
    assert not traffic._holds_ahead(m, "N", ["N", "O"])
    assert traffic._holds_ahead(m, "N", ["N", "S", "O"])


def test_a_retreat_goal_is_not_watched_as_a_lane_move():
    pilot = Pilot()
    on_the_way(pilot)
    m = pilot.mission()
    m.goal = _Goal(RETREAT, 999)
    traffic = pilot.core.traffic
    assert traffic._running_lane(m) is None
    assert traffic._error_lane(m) is None
    before = m.idx
    pilot.run(1.0)  # 레인 진행 추적, 이탈 감시, 막힘 감시가 이 goal에 손대지 않는다
    assert m.idx == before
    assert m.goal is not None and m.goal.kind == RETREAT
