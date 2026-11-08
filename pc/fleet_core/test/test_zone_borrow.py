"""옆 구역 빌리기의 잠금과 닿는 구역(버전 2 1-C 1부). 아직 제어기에 잇지 않는다.

- borrow_all: 모두 비었고 그 구역(충돌 쌍 포함)을 기다리는 로봇도 없을 때만 한꺼번에 준다.
  빌리기는 양보형이라 줄에 서지 않는다(평소 교통이 먼저). 안 되면 아무것도 잡지 않는다.
  기다리는 동안 빌리려는 구역은 기록해 두고(borrowing), withdraw로 거둔다.
- zones_touched: 경로를 sample_m 간격으로 나눠, 바닥에서 release_m - half_width보다 가까운
  구역(_off_the_floor와 같은 기준이라 "닿는다"와 "해제된다"가 맞는다).
"""
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.zones import ZoneLocks


def locks():
    return ZoneLocks(["A", "B", "C"], lambda: 0.0, conflicts=[frozenset({"B", "C"})])


def test_free_zones_are_borrowed_together():
    z = locks()
    assert z.borrow_all("r1", ["A", "B"])
    assert z.held_by("r1") == ["A", "B"]
    assert z.borrowing("r1") == ()


def test_nothing_is_borrowed_while_one_zone_is_held_or_waited_for():
    z = locks()
    assert z.request("r2", "B")
    assert not z.borrow_all("r1", ["A", "B"])
    assert z.held_by("r1") == [] and z.queue("A") == () and z.queue("B") == ()
    assert z.borrowing("r1") == ("A", "B")
    z2 = locks()
    assert z2.request("r2", "A")
    assert not z2.request("r3", "A")  # r3 waits for A
    z2.release("r2", "A")
    assert not z2.borrow_all("r1", ["A"]), "기다리는 로봇이 있으면 빌리지 않는다"


def test_a_conflicting_zone_held_or_waited_for_also_stops_a_borrow():
    z = locks()
    assert z.request("r2", "C")
    assert not z.borrow_all("r1", ["B"])
    z.release("r2", "C")
    assert z.borrow_all("r1", ["B"])


def test_a_borrow_never_stands_in_front_of_normal_traffic():
    z = locks()
    assert z.request("r2", "A")
    assert not z.borrow_all("r1", ["A"])
    assert not z.request("r3", "A")
    z.release("r2", "A")
    assert z.request("r3", "A"), "빌리려는 로봇은 줄에 없다"


def test_withdraw_forgets_the_borrow():
    z = locks()
    assert z.request("r2", "A")
    assert not z.borrow_all("r1", ["A"])
    z.withdraw("r1")
    assert z.borrowing("r1") == ()
    z.withdraw("r1")  # 두 번 거둬도 된다


def test_zones_already_held_by_the_borrower_count_as_free():
    z = locks()
    assert z.request("r1", "A")
    assert z.borrow_all("r1", ["A", "B"])
    assert z.held_by("r1") == ["A", "B"]


def test_a_path_touches_the_zones_whose_floor_it_comes_near():
    site = load_lane_map(None)
    lane = next(iter(site.lanes.values()))
    zone = site.zone_of[lane.a]
    points = [(lane.ax, lane.ay), (lane.bx, lane.by)]
    touched = site.zones_touched(points, 0.05, 0.2)
    assert zone in touched and site.zone_of[lane.b] in touched
    far = [(lane.ax + 5.0, lane.ay + 5.0), (lane.ax + 5.1, lane.ay + 5.0)]
    assert site.zones_touched(far, 0.05, 0.2) == set()


def test_touching_uses_the_release_rule():
    """바닥에서 release - half_width 안이면 닿는다. 그 밖이면 _off_the_floor가 놓는 거리다."""
    site = load_lane_map(None)
    lane = next(lane for lane in site.lanes.values()
                if site.zone_of[lane.a] == site.zone_of[lane.b] and lane.span > 0.3)
    zone = site.zone_of[lane.a]
    mx, my = (lane.ax + lane.bx) / 2, (lane.ay + lane.by) / 2
    nx, ny = -lane.uy, lane.ux  # 레인에 수직
    for offset, expected in ((0.19, True), (0.21, False)):
        point = (mx + nx * offset, my + ny * offset)
        near = site.distance_to_zone(*point, zone) + site.half_width < 0.2
        assert near is expected
        assert (zone in site.zones_touched([point], 0.05, 0.2)) is expected
