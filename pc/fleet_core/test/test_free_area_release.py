"""자유 구역을 출구로 떠날 때의 해제(버전 2 V2-0 검수 지적).

release_clear_of_floor가 꺼져 있어도(그 값이 없던 기록) 자유 구역은 다각형 바닥에서
release 거리만큼 벗어나야 놓는다.
"""
import copy
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.lane_map import LaneMap, load_lane_map
from fleet_core.traffic.mission import _Mission
from fleet_core.traffic.release_floor import TrafficReleaseFloor
from test_free_areas import BASE, site


class Floor(TrafficReleaseFloor):
    def __init__(self, s, params):
        self.site, self.params = s, params


def test_the_area_floor_is_checked_even_without_the_floor_rule():
    s = site()
    params = replace(load_params(), release_clear_of_floor=False)
    core = Floor(s, params)
    m = _Mission("r1")
    m.x, m.y = 2.05, 1.45  # G2->E0 위가 아니라 광장 모서리 바로 옆(바닥에서 0.05 m)
    assert not core._clear_of_floor(m, "PLAZA", ("G2", "E0"))
    m.x, m.y = 2.45, 1.0  # 바닥에서 0.45 m
    assert core._clear_of_floor(m, "PLAZA", ("G2", "E0"))


def test_ordinary_zones_keep_the_old_rule_without_the_flag():
    s = load_lane_map()
    params = replace(load_params(), release_clear_of_floor=False)
    core = Floor(s, params)
    m = _Mission("r1")
    m.x, m.y = s.waypoint_pose("A")
    assert core._clear_of_floor(m, "AVENUE", ("A", "L"))


def test_an_exit_ending_next_to_the_area_is_refused_even_without_the_floor_rule():
    data = copy.deepcopy(BASE)
    data["waypoints"].append({"id": "X", "x": 2.05, "y": 1.45})  # 광장 모서리 옆에서 끝남
    data["lanes"] += [{"from": "G2", "to": "X"}, {"from": "X", "to": "N"}]
    params = replace(load_params(), release_clear_of_floor=False)
    with pytest.raises(ValueError, match="G2 to X"):
        Floor(LaneMap(data), params)._check_exit_floor()
    Floor(site(), params)._check_exit_floor()  # 광장 지도 그대로는 통과
