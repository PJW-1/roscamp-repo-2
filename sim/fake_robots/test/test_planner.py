"""가짜 경로 계산(버전 2 F-1, fake_robots/planner.py)."""
import math

from fake_robots.planner import plan_path

FLOOR = [(1.0, 0.5), (2.0, 0.5), (2.0, 0.85), (1.0, 0.85)]  # feature_map_free 광장


def gap_to(points, x, y):
    return min(math.dist(p, (x, y)) for p in points)


def test_no_box_gives_a_dense_straight_line():
    points = plan_path((1.0, 0.6), (2.0, 0.6))
    assert points[0] == (1.0, 0.6) and points[-1] == (2.0, 0.6)
    assert all(abs(y - 0.6) < 1e-9 for _, y in points)
    assert max(math.dist(a, b) for a, b in zip(points, points[1:])) <= 0.05 + 1e-9


def test_a_box_on_the_line_is_passed_with_clearance_inside_the_floor():
    box = (1.5, 0.6, 0.1)
    points = plan_path((1.0, 0.6), (2.0, 0.6), [box], clearance_m=0.12, polygon=FLOOR)
    assert points is not None
    assert gap_to(points, 1.5, 0.6) >= 0.05 + 0.12 - 1e-9
    assert all(1.0 - 1e-9 <= x <= 2.0 + 1e-9 and 0.5 - 1e-9 <= y <= 0.85 + 1e-9
               for x, y in points)


def test_the_planner_turns_to_the_side_that_stays_on_the_floor():
    box = (1.5, 0.78, 0.1)  # 위쪽에 붙은 상자: 왼쪽(위)으로 비키면 바닥 밖
    points = plan_path((1.0, 0.75), (2.0, 0.75), [box], clearance_m=0.08, polygon=FLOOR)
    assert points is not None
    assert min(y for _, y in points) < 0.75, "아래쪽으로 비켜 간다"


def test_no_way_around_gives_none():
    box = (1.5, 0.675, 0.3)  # 바닥 폭 0.35를 거의 다 막는다
    assert plan_path((1.0, 0.675), (2.0, 0.675), [box], clearance_m=0.12, polygon=FLOOR) is None


def test_a_box_in_the_middle_of_a_short_crossing_is_passed_with_two_turns():
    box = (1.35, 0.675, 0.06)  # G3에서 G4로 0.35 m 건너는 길 한가운데(free_area_box)
    points = plan_path((1.35, 0.85), (1.35, 0.5), [box], clearance_m=0.12, polygon=FLOOR)
    assert points is not None
    assert gap_to(points, 1.35, 0.675) >= 0.03 + 0.12 - 1e-9
    assert all(1.0 - 1e-9 <= x <= 2.0 + 1e-9 and 0.5 - 1e-9 <= y <= 0.85 + 1e-9
               for x, y in points)


def test_a_box_next_to_the_end_of_the_line_is_not_passed_straight():
    box = (2.0, 0.66, 0.06)  # 목표 바로 옆. 투영이 선분 끝(t=1)에 걸린다
    points = plan_path((1.0, 0.6), (2.0, 0.6), [box], clearance_m=0.12, polygon=FLOOR)
    assert points is None or gap_to(points, 2.0, 0.66) >= 0.03 + 0.12 - 1e-9


def test_a_zero_length_request_inside_a_box_gets_no_path():
    assert plan_path((1.5, 0.6), (1.5, 0.6), [(1.52, 0.6, 0.06)]) is None
    assert plan_path((1.5, 0.6), (1.5, 0.6)) == [(1.5, 0.6), (1.5, 0.6)]
