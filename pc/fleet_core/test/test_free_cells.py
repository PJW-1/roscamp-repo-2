"""자유 구역 칸(버전 2 F-2 1부): 다각형 바닥을 cell_m 칸으로 나누고 경로가 닿는 칸을 찾는다.

칸은 다각형을 감싸는 사각형의 왼쪽 아래 모서리에서 cell_m 간격으로 자른 정사각형이고, 바닥과
겹치는(닿는 것 포함) 칸만 남는다. 경로가 닿는 칸은 꺾은선에서 clearance 안에 드는 칸이다.
아직 잠금에는 쓰지 않는다(동작 변화 없음).
"""
import copy

import pytest
from fleet_core.traffic.lane_map import LaneMap
from test_free_areas import BASE

TRIANGLE = [[1.0, 0.5], [2.0, 0.5], [1.0, 1.5]]


def site(polygon=None, cell_m=0.35):
    data = copy.deepcopy(BASE)
    if polygon is not None:
        data["free_areas"]["PLAZA"]["polygon"] = polygon
        # 출입구는 바닥 가장자리여야 한다: G2를 삼각형 밑변으로 옮긴다
        g2 = next(w for w in data["waypoints"] if w["id"] == "G2")
        g2["x"], g2["y"] = 1.5, 0.5
    data["free_areas"]["PLAZA"]["cell_m"] = cell_m
    return LaneMap(data)


def names(*cells):
    return frozenset(f"PLAZA/{ix},{iy}" for ix, iy in cells)


def test_a_square_floor_is_cut_into_a_grid_from_its_lower_left_corner():
    cells = site().free_cells("PLAZA")
    assert set(cells) == names(*[(ix, iy) for ix in range(3) for iy in range(3)])
    assert cells["PLAZA/0,0"] == pytest.approx((1.0, 0.5, 1.35, 0.85))
    # 마지막 칸은 바닥 밖으로 나가도 정사각형 그대로다(더 잡는 쪽이라 안전하다)
    assert cells["PLAZA/2,2"] == pytest.approx((1.7, 1.2, 2.05, 1.55))


def test_cells_that_do_not_touch_a_triangular_floor_are_dropped():
    # 왼쪽 아래 직각삼각형: 빗변 x + y = 2.5 위쪽 칸은 바닥에 닿지 않는다
    cells = site(TRIANGLE).free_cells("PLAZA")
    assert set(cells) == names((0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2))


def test_a_straight_path_takes_the_row_it_runs_in():
    s = site()
    assert s.cells_near("PLAZA", [(1.0, 1.0), (2.0, 1.0)], 0.05) == names((0, 1), (1, 1), (2, 1))


def test_clearance_reaches_into_the_next_row():
    # 가운데 줄 아래 경계 y=0.85까지 0.15, 위 경계 y=1.2까지 0.2
    s = site()
    assert s.cells_near("PLAZA", [(1.0, 1.0), (2.0, 1.0)], 0.16) == names(
        *[(ix, iy) for ix in range(3) for iy in (0, 1)])


def test_a_diagonal_path_takes_the_cells_whose_corners_it_grazes():
    # y = x - 0.5 는 칸 모서리 (1.35, 0.85), (1.7, 1.2)를 지난다
    s = site()
    got = s.cells_near("PLAZA", [(1.05, 0.55), (1.95, 1.45)], 0.01)
    assert got == names((0, 0), (1, 1), (2, 2), (0, 1), (1, 0), (1, 2), (2, 1))


def test_a_path_inside_one_cell_takes_only_that_cell():
    s = site()
    assert s.cells_near("PLAZA", [(1.1, 0.6), (1.2, 0.7)], 0.05) == names((0, 0))


def test_cells_near_a_path_are_cells_of_the_floor():
    s = site(TRIANGLE)
    got = s.cells_near("PLAZA", [(1.0, 1.0), (1.4, 0.6), (2.0, 0.5)], 0.3)
    assert got and got <= set(s.free_cells("PLAZA"))
