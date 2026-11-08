"""자유 구역 칸 잠금 단위(버전 2 F-2 2부): 지도 쪽.

칸 잠금에서는 자유 구역 하나 대신 두 가지 단위를 잠근다.
  - 칸 "구역/ix,iy": 다각형 바닥을 cell_m으로 자른 정사각형(1부의 free_cells).
  - 진입 레인 "구역/>G": 바깥 지점에서 출입구 G로 들어오는 레인들. 구역 모드에서는 자유 구역
    바닥의 일부였다(레인은 끝 지점의 단위에 속한다). 칸이 덮지 않으므로 따로 잠근다.
충돌 쌍은 기존 규칙과 같다: 바닥이 2r보다 가까우면 함께 쥘 수 없고, 출입구에서 나가는 레인은
앞 release만큼 뺀다.
"""
import pytest
from fleet_core.traffic.lane_map import load_lane_map
from test_free_areas import MAPS, site

CLEAR, RELEASE = 0.16, 0.2


def test_the_units_are_the_cells_and_one_approach_per_gate_with_lanes_in():
    s = site()
    units = s.cell_units()
    cells = {f"PLAZA/{ix},{iy}" for ix in range(3) for iy in range(3)}
    assert set(units) == cells | {"PLAZA/>G1"}, "G2로 들어오는 바깥 레인은 없다"
    assert all(area == "PLAZA" for area in units.values())
    assert s.unit_area("PLAZA/1,1") == "PLAZA"
    assert s.unit_area("PLAZA") is None and s.unit_area("G1") is None
    assert s.approach_unit("PLAZA", "G1") == "PLAZA/>G1"
    assert s.approach_unit("PLAZA", "G2") is None


def test_distance_to_a_cell_is_the_distance_to_its_square():
    s = site()
    assert s.distance_to_zone(1.1, 0.6, "PLAZA/0,0") == 0.0
    assert s.distance_to_zone(1.6, 0.6, "PLAZA/0,0") == pytest.approx(0.25)


def test_distance_to_an_approach_is_the_lane_floor_like_a_zone():
    s = site()
    # W0(0.3, 1.0) -> G1(1.0, 1.0). 바닥은 lane_entry(0.25) 뒤부터 중심선 양쪽 half_width
    assert s.distance_to_zone(0.8, 1.0, "PLAZA/>G1") == 0.0
    assert s.distance_to_zone(0.8, 1.2, "PLAZA/>G1") == pytest.approx(0.15)
    assert s.distance_to_zone(0.3, 1.0, "PLAZA/>G1") == pytest.approx(0.2)


def test_cells_conflict_with_outside_lanes_closer_than_2r_but_not_the_trimmed_exit():
    s = site()
    pairs = s.cell_conflict_pairs(CLEAR, RELEASE)
    # 나가는 레인 G2->E0의 앞 0.2 m는 뺀다: (2.2, 1.0)부터. 칸 2,1은 x 2.05까지라 0.15 < 0.16
    assert frozenset({"PLAZA/2,1", s.zone_of["E0"]}) in pairs
    assert frozenset({"PLAZA/1,1", s.zone_of["E0"]}) not in pairs
    # 아래로 도는 길(E0->S->W0)은 광장 모서리에서 0.23 m 떨어진다
    assert not any(s.zone_of["S"] in pair for pair in pairs)


def test_the_approach_conflicts_with_the_cells_it_runs_into():
    s = site()
    pairs = s.cell_conflict_pairs(CLEAR, RELEASE)
    assert frozenset({"PLAZA/>G1", "PLAZA/0,1"}) in pairs, "출입구 칸과는 늘 충돌 쌍이다"
    assert frozenset({"PLAZA/>G1", "PLAZA/2,1"}) not in pairs


def test_the_plaza_map_cells_and_approaches():
    s = load_lane_map(MAPS / "feature_map_free.yaml")
    units = s.cell_units()
    assert {u for u in units if ">" in u} == {"PLAZA/>T1", "PLAZA/>T2", "PLAZA/>G3"}
    assert len([u for u in units if ">" not in u]) == 3
    pairs = s.cell_conflict_pairs(CLEAR, RELEASE)
    assert not any("PLAZA" in pair for pair in pairs), \
        "칸 모드에서는 자유 구역 단위 자체를 쓰지 않으므로 충돌 쌍에 넣지 않는다"
    assert frozenset({"PLAZA/>G3", s.zone_of["F2"]}) not in pairs


def test_a_map_without_free_areas_has_no_units():
    s = load_lane_map()
    assert s.cell_units() == {}
    assert s.cell_conflict_pairs(CLEAR, RELEASE) == set()
