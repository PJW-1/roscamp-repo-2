"""실행 중에 등록하고 지우는 임시 구역(버전 2 V2-0 3번). 1-C 비켜 가기가 쓴다.

잠금(zones.py)은 구역과 충돌 쌍을 실행 중에 더하고 뺄 수 있다. 쥐었거나 기다리는 로봇이 있는
구역은 지우지 않는다. 지도(lane_map.py)는 경로 선으로 된 임시 구역의 바닥과, 그 바닥이
2r보다 가깝게 지나는 다른 구역(충돌 쌍)을 안다.
"""
import pytest
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.zones import ZoneLocks


def locks():
    return ZoneLocks(["A", "B"], lambda: 0.0)


def test_an_added_zone_is_locked_like_any_other():
    z = locks()
    z.add_zone("T1", conflicts=("B",))
    assert z.exists("T1")
    assert z.request("r1", "T1")
    assert not z.request("r2", "T1")
    assert not z.request("r2", "B"), "충돌 쌍도 함께 잠근다"
    assert z.blockers("r2", "B") == {"r1"}


def test_adding_a_zone_twice_or_with_an_unknown_conflict_is_refused():
    z = locks()
    z.add_zone("T1")
    with pytest.raises(ValueError):
        z.add_zone("T1")
    with pytest.raises(ValueError):
        z.add_zone("T2", conflicts=("NOPE",))


def test_a_free_zone_is_removed_with_its_conflicts():
    z = locks()
    z.add_zone("T1", conflicts=("B",))
    assert z.remove_zone("T1")
    assert not z.exists("T1")
    assert z.request("r2", "B")
    assert "T1" not in z.snapshot()


def test_a_held_or_awaited_zone_is_not_removed():
    z = locks()
    z.add_zone("T1")
    z.request("r1", "T1")
    assert not z.remove_zone("T1")
    z.request("r2", "T1")
    z.release("r1", "T1")
    assert z.holder("T1") is None
    assert not z.remove_zone("T1"), "기다리는 로봇이 있으면 지우지 않는다"
    z.cancel_request("r2", "T1")
    assert z.remove_zone("T1")


def test_shipped_zones_cannot_be_removed():
    z = locks()
    with pytest.raises(ValueError):
        z.remove_zone("A")


def test_a_temporary_floor_has_distance_and_conflicts():
    site = load_lane_map()
    before = dict(site.zones)
    # AVENUE 바로 위 0.1 m 옆으로 지나는 짧은 선
    ax, ay = site.waypoint_pose("A")
    bx, by = site.waypoint_pose("B")
    points = [(ax + 0.3, ay + 0.1), (bx - 0.3, by + 0.1)]
    site.add_temp_zone("TMP", points)
    assert "TMP" in site.zones and site.zones["TMP"] == frozenset()
    assert site.distance_to_zone(ax + 0.5, ay + 0.1, "TMP") == 0.0
    assert site.distance_to_zone(ax + 0.5, ay + 0.5, "TMP") > 0.3
    assert "AVENUE" in site.temp_zone_conflicts("TMP", clearance_m=0.16)
    assert "P-3" not in site.temp_zone_conflicts("TMP", clearance_m=0.16)
    site.remove_temp_zone("TMP")
    assert site.zones == before
    with pytest.raises(KeyError):
        site.distance_to_zone(0.0, 0.0, "TMP")


def test_a_temporary_zone_needs_two_points_and_a_new_name():
    site = load_lane_map()
    with pytest.raises(ValueError):
        site.add_temp_zone("TMP", [(1.0, 1.0)])
    with pytest.raises(ValueError):
        site.add_temp_zone("AVENUE", [(1.0, 1.0), (1.2, 1.0)])
    with pytest.raises(ValueError):
        site.remove_temp_zone("AVENUE")


def test_conflicts_given_as_a_one_shot_iterable_are_all_kept():
    z = locks()
    z.add_zone("T1", conflicts=iter(["B"]))
    z.request("r1", "T1")
    assert not z.request("r2", "B")
