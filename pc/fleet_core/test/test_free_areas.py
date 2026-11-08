"""자유 구역 지도 형식(버전 2 V2-0 7번, F절).

레인 지도 yaml의 `free_areas:`에 다각형 바닥, 출입구 지점(gates), 칸 크기(cell_m, F-2용)를
적는다. 출입구는 레인 그래프의 지점이고 모두 그 자유 구역의 잠금 단위에 든다. 경로 계획에는
출입구 사이를 잇는 가상 레인(양방향, 비용 = 직선 거리)이 들어간다. 자유 구역이 없는 지도는
지금과 똑같다.
"""
import copy
from pathlib import Path

import pytest
from fleet_core.traffic.lane_map import LaneMap, LaneMapError, load_lane_map

MAPS = Path(__file__).resolve().parents[3] / "sim" / "fake_robots" / "maps"

# 서쪽 레인 W0->G1, 광장(1.0~2.0 x 0.5~1.5), 동쪽 레인 G2->E0, 아래위로 도는 길 E0->S->W0,
# E0->N->W0. 도는 길은 광장 모서리에서 0.23 m 떨어진다
BASE = {
    "half_width_m": 0.05,
    "lane_entry_m": 0.25,
    "origin_m": [0.0, 0.0],
    "waypoints": [
        {"id": "W0", "x": 0.3, "y": 1.0},
        {"id": "G1", "x": 1.0, "y": 1.0},
        {"id": "G2", "x": 2.0, "y": 1.0},
        {"id": "E0", "x": 2.7, "y": 1.0},
        {"id": "S", "x": 1.5, "y": -0.5},
        {"id": "N", "x": 1.5, "y": 2.5},
    ],
    "lanes": [
        {"from": "W0", "to": "G1"},
        {"from": "G2", "to": "E0"},
        {"from": "E0", "to": "S"},
        {"from": "S", "to": "W0"},
        {"from": "E0", "to": "N"},
        {"from": "N", "to": "W0"},
    ],
    "free_areas": {
        "PLAZA": {
            "polygon": [[1.0, 0.5], [2.0, 0.5], [2.0, 1.5], [1.0, 1.5]],
            "gates": ["G1", "G2"],
            "cell_m": 0.35,
        },
    },
}


def site(data=None):
    return LaneMap(copy.deepcopy(data or BASE))


def test_gates_are_in_the_area_zone_and_joined_by_virtual_lanes():
    s = site()
    assert s.zones["PLAZA"] == frozenset({"G1", "G2"})
    assert s.zone_of["G1"] == s.zone_of["G2"] == "PLAZA"
    for key in (("G1", "G2"), ("G2", "G1")):
        lane = s.lane(*key)
        assert lane.length == pytest.approx(1.0)
        assert key in s.free_lanes
    assert ("W0", "G1") not in s.free_lanes
    area = s.free_areas["PLAZA"]
    assert area.gates == ("G1", "G2") and area.cell_m == 0.35
    assert s.free_area_of(("G1", "G2")) == "PLAZA"
    assert s.free_area_of(("W0", "G1")) is None


def test_the_planner_crosses_the_area():
    assert site().plan("W0", "E0") == ["W0", "G1", "G2", "E0"]


def test_the_area_floor_is_the_polygon():
    s = site()
    assert s.distance_to_zone(1.5, 1.4, "PLAZA") == 0.0  # 가상 레인에서 0.4 m지만 바닥 안
    assert s.distance_to_zone(1.5, 1.7, "PLAZA") == pytest.approx(0.2)
    assert s.inside_free_floor("PLAZA", 1.2, 0.6)
    assert not s.inside_free_floor("PLAZA", 1.2, 0.3)


def test_a_lane_near_the_polygon_conflicts_with_the_area():
    data = copy.deepcopy(BASE)
    data["waypoints"] += [{"id": "X0", "x": 1.0, "y": 1.6}, {"id": "X1", "x": 2.0, "y": 1.6}]
    data["lanes"] += [{"from": "X0", "to": "X1"}, {"from": "X1", "to": "N"},
                      {"from": "N", "to": "X0"}]
    pairs = site(data).conflict_pairs(0.16, 0.2)
    assert frozenset({"PLAZA", "X1"}) in pairs  # 광장 0.1 m 위를 지나는 X0->X1
    assert frozenset({"PLAZA", "S"}) not in pairs  # E0->S는 광장 모서리에서 0.23 m


def test_a_lane_out_of_a_gate_skips_the_release_distance_next_to_the_area():
    s = site()
    area = s.free_areas["PLAZA"]
    lane = s.lane("G2", "E0")  # 출입구에서 시작해 바닥 가장자리에 붙어 있다
    assert s._near_area(lane, area, 0.16, 0.0)
    assert not s._near_area(lane, area, 0.16, 0.2), "앞 release만큼은 공유 지점처럼 뺀다"
    assert frozenset({"PLAZA", "E0"}) not in s.conflict_pairs(0.16, 0.2)
    assert s.zone_of["W0"] != "PLAZA" and s.zone_of["G1"] == "PLAZA"  # 들어오는 레인은 광장 단위


@pytest.mark.parametrize("change, message", [
    (lambda d: d["free_areas"]["PLAZA"].update(gates=["G1", "NOPE"]), "nowhere"),
    (lambda d: d["free_areas"]["PLAZA"].update(polygon=[[1, 1], [2, 1]]), "three"),
    (lambda d: d["free_areas"]["PLAZA"].update(gates=["G1"]), "two gates"),
    (lambda d: d["free_areas"]["PLAZA"].update(gates=["G1", "N"]), "edge"),
    (lambda d: d.update(units={"U": ["G1"]}), "more than one unit"),
    (lambda d: d["free_areas"]["PLAZA"].update(cell_m=0), "cell_m"),
])
def test_bad_free_areas_are_refused(change, message):
    data = copy.deepcopy(BASE)
    change(data)
    with pytest.raises(LaneMapError, match=message):
        LaneMap(data)


def test_maps_without_free_areas_have_none():
    s = load_lane_map()
    assert s.free_areas == {} and s.free_lanes == frozenset()


def test_a_chord_inside_the_area_does_not_conflict_with_a_lane_leaving_a_gate():
    """가상 레인은 바닥 안에 있으니 바깥 레인과의 거리는 다각형이 본다(출입구 앞 release 제외).

    특징 광장 지도: T1-T2 가상 레인은 G4에서 0.1 m 위를 지나지만 G4->P-1은 G4에서 나가는 레인이다.
    """
    s = load_lane_map(MAPS / "feature_map_free.yaml")
    assert ("T1", "T2") in s.free_lanes
    assert frozenset({"PLAZA", "P-1"}) not in s.conflict_pairs(0.16, 0.2)


def test_a_gate_cannot_be_in_a_two_way_bundle():
    data = copy.deepcopy(BASE)
    data["lanes"] = [lane for lane in data["lanes"] if lane != {"from": "W0", "to": "G1"}]
    data["two_way"] = {"WEST": ["W0", "G1"]}
    with pytest.raises(LaneMapError, match="two_way"):
        LaneMap(data)


def test_overlapping_areas_conflict():
    """두 다각형이 겹치면 출입구와 가상 레인이 서로 멀어도 충돌 쌍이다."""
    data = copy.deepcopy(BASE)
    data["waypoints"] += [{"id": "H1", "x": 3.0, "y": 2.0}, {"id": "H2", "x": 2.4, "y": 2.2}]
    data["lanes"] += [{"from": "N", "to": "H2"}, {"from": "H1", "to": "E0"}]
    data["free_areas"]["YARD"] = {"polygon": [[1.8, 1.4], [3.0, 1.4], [3.0, 2.2], [1.8, 2.2]],
                                  "gates": ["H1", "H2"]}
    pairs = site(data).conflict_pairs(0.16, 0.2)
    assert frozenset({"PLAZA", "YARD"}) in pairs


def test_a_temporary_zone_over_the_area_floor_conflicts_with_the_area():
    s = site()
    s.add_temp_zone("TMP", [(1.4, 0.6), (1.6, 0.6)])  # 바닥 안, 가상 레인 G1-G2에서 0.4 m
    assert "PLAZA" in s.temp_zone_conflicts("TMP", clearance_m=0.16)
