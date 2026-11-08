"""버전 2 4절 모서리 둥글리기(기본 선택 13번, 제안). 레인 경로의 꺾인 지점을 작은 원호로 바꾼다.

원호가 레인에서 벗어나는 폭은 half_width(0.05 m) 안이고, 반지름은 corner_radius_m(0.12 m)
이하다. 이어진 원호가 겹치지 않게 접점은 각 레인의 절반 안에 둔다. 자유 구역 경로 점과
로봇 위치, 목표 지점은 그대로 둔다.
"""
import math
import random

import pytest
from fleet_core.traffic.lane_map import _point_segment
from fleet_core.traffic.lane_path import round_corners


def gap_to(points, x, y):
    return min(_point_segment(x, y, *a, *b) for a, b in zip(points, points[1:], strict=False))


def progress(points, x, y):
    """원래 선 위로 투영한 진행 거리."""
    best, at, walked = None, 0.0, 0.0
    for a, b in zip(points, points[1:], strict=False):
        length = math.dist(a, b)
        t = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / (length * length)
        t = min(1.0, max(0.0, t))
        d = math.dist((x, y), (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        if best is None or d < best - 1e-12:
            best, at = d, walked + t * length
        walked += length
    return at


def test_a_right_angle_becomes_an_arc_inside_the_lane_floor():
    line = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    out = round_corners(line, [False, True, False], 0.12, 0.05)
    assert out[0] == line[0] and out[-1] == line[-1]
    assert (1.0, 0.0) not in out
    assert len(out) > 4
    assert max(gap_to(line, *p) for p in out) <= 0.05 + 1e-9
    mid = min(out[1:-1], key=lambda p: math.dist(p, (1.0, 0.0)))
    assert math.dist(mid, (1.0, 0.0)) == pytest.approx(0.12 * (math.sqrt(2) - 1), abs=0.005)
    assert out[1] == pytest.approx((1.0 - 0.12, 0.0)), "원호는 반지름만큼 앞에서 시작한다"


def test_nothing_changes_with_a_zero_radius_or_a_point_that_must_stay():
    line = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert round_corners(line, [False, True, False], 0.0, 0.05) == line
    assert round_corners(line, [False, False, False], 0.12, 0.05) == line


def test_a_straight_point_is_left_alone():
    line = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
    assert round_corners(line, [False, True, False], 0.12, 0.05) == line


def test_a_sharp_turn_gets_a_smaller_radius_to_stay_on_the_floor():
    line = [(0.0, 0.0), (1.0, 0.0), (0.3, 0.5)]  # 약 144도 꺾임
    out = round_corners(line, [False, True, False], 0.12, 0.05)
    assert max(gap_to(line, *p) for p in out) <= 0.05 + 1e-9


def test_short_lanes_keep_the_arcs_apart():
    line = [(0.0, 0.0), (0.1, 0.0), (0.1, 0.1), (0.2, 0.1)]
    out = round_corners(line, [False, True, True, False], 0.12, 0.05)
    s = [progress(line, *p) for p in out]
    assert all(b >= a - 1e-9 for a, b in zip(s, s[1:], strict=False))


def test_random_lines_stay_on_the_floor_and_only_move_forward():
    rng = random.Random(7)
    for _ in range(300):
        pts, heading = [(0.0, 0.0)], 0.0
        for _ in range(rng.randint(2, 6)):
            heading += math.radians(rng.uniform(-150, 150))
            length = rng.uniform(0.05, 1.5)
            x, y = pts[-1]
            pts.append((x + math.cos(heading) * length, y + math.sin(heading) * length))
        keep = [False] + [rng.random() < 0.8 for _ in pts[2:]] + [False]
        out = round_corners(pts, keep, 0.12, 0.05)
        assert out[0] == pts[0] and out[-1] == pts[-1]
        assert max(gap_to(pts, *p) for p in out) <= 0.05 + 1e-9
        for p, k in zip(pts, keep, strict=True):
            if not k:
                assert p in out, "둥글리지 않을 점은 그대로 남는다"
        s = [progress(pts, *p) for p in out]
        assert all(b >= a - 1e-6 for a, b in zip(s, s[1:], strict=False))


def test_no_tiny_arc_when_the_robot_stands_on_the_corner():
    line = [(1.0, -0.005), (1.0, 0.0), (0.0, 0.0)]
    assert round_corners(line, [False, True, False], 0.12, 0.05) == line


def test_an_accept_check_can_keep_a_corner():
    line = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert round_corners(line, [False, True, False], 0.12, 0.05, lambda i, arc: False) == line


# ---- 관제가 보내는 경로 -----------------------------------------------------------------------


def test_a_bundled_goal_rounds_a_corner_between_two_lane_waypoints():
    from dataclasses import replace

    from test_lane_path import make, moves, send, xy
    core, _ = make({"r1": "W"})
    core.params = replace(core.params, corner_radius_m=0.12)
    send(core, "r1", "OUTBOUND")
    for _ in range(3):  # W를 지나 W-A 레인 위(아직 W가 선에 남는 거리)
        core.on_robot_state("r1", *xy(core, "W"), 0.0, "MOVING")
    first = moves(core.tick())[0]
    points = [p[:2] for p in first.path]
    assert xy(core, "A") in points, "로봇 자리에서 오는 첫 꺾임은 둔다"


def test_the_yaml_turns_corner_rounding_on_and_records_without_it_keep_sharp_corners():
    from fleet_core.common.params import FleetParams, load_params
    assert load_params().corner_radius_m == pytest.approx(0.12)
    assert FleetParams.__dataclass_fields__["corner_radius_m"].default == 0.0


def test_a_full_run_sends_rounded_corners_that_stay_on_the_lane_floor():
    from fleet_core.common.commands import MoveCmd
    from sim_world import SimWorld
    world = SimWorld(seed=7)
    assert world.run_until(world.done)
    lanes = list(world.site.lanes.values())
    arcs = 0
    for _, c in world.commands:
        if not isinstance(c, MoveCmd):
            continue
        for x, y, _ in c.path[1:]:
            off = min(_point_segment(x, y, ln.ax, ln.ay, ln.bx, ln.by) for ln in lanes)
            assert off <= world.site.half_width + 1e-9
            arcs += off > 1e-6
    assert arcs > 0, "둥글린 모서리가 실제로 나간다"


# ---- horizon 2 Codex 검수 -------------------------------------------------------------------


def corner_site(plaza=True):
    """A(0,0) -> B(1,0) -> C(1,1). 모서리 안쪽에 다른 구역(자유 구역 PLAZA) 바닥의 모서리
    (0.97, 0.03)이 있다. 꺾인 그대로의 선은 그 바닥에 들지 않지만 원호 가운데는 든다."""
    from fleet_core.traffic.lane_map import LaneMap
    data = {
        "half_width_m": 0.05, "lane_entry_m": 0.25, "origin_m": [0.0, 0.0],
        "waypoints": [{"id": "A", "x": 0.0, "y": 0.0}, {"id": "B", "x": 1.0, "y": 0.0},
                      {"id": "C", "x": 1.0, "y": 1.0}],
        "lanes": [{"from": "A", "to": "B"}, {"from": "B", "to": "C"}],
    }
    if plaza:
        data["waypoints"] += [{"id": "G1", "x": 0.6, "y": 0.2}, {"id": "G2", "x": 0.8, "y": 0.4}]
        data["free_areas"] = {"PLAZA": {"polygon": [[0.6, 0.03], [0.97, 0.03], [0.97, 0.4],
                                                    [0.6, 0.4]], "gates": ["G1", "G2"]}}
    return LaneMap(data)


def arc_check(site):
    from types import SimpleNamespace

    from fleet_core.traffic.lane_path import TrafficPath
    core = SimpleNamespace(site=site, _zone=lambda w: site.zone_of[w])
    return lambda points: TrafficPath._arc_stays_in(core, points)


def test_an_arc_onto_another_zones_floor_is_not_used():
    site = corner_site()
    line = [site.waypoint_pose(w) for w in ("A", "B", "C")]
    assert all(site.distance_to_zone(x, y, "PLAZA") > 0 for x, y in line)
    out = round_corners(line, [False, True, False], 0.12, 0.05)
    arc = [p for p in out if p not in line]
    assert any(site.distance_to_zone(*p, "PLAZA") == 0.0 for p in arc), "시험 전제"
    accept = arc_check(site)(line)
    assert not accept(1, arc)
    assert round_corners(line, [False, True, False], 0.12, 0.05, accept) == line


def test_without_the_other_floor_the_same_corner_is_rounded():
    site = corner_site(plaza=False)
    line = [site.waypoint_pose(w) for w in ("A", "B", "C")]
    assert round_corners(line, [False, True, False], 0.12, 0.05, arc_check(site)(line)) != line


def test_gates_of_a_straight_free_crossing_keep_their_sharp_points():
    """계획이 없어 출입구 사이 직선만 있어도(안쪽 점 없음) 출입구는 둥글리지 않는다."""
    from dataclasses import replace

    from test_real_retreat import SOUTH, at_the_gate
    d, m = at_the_gate()
    d.core.params = replace(d.core.params, corner_radius_m=0.12)
    m.path, m.idx = ["F1", "G3", "T2", "QO"], 0
    fx, fy = d.site.waypoint_pose("F1")
    m.x, m.y, m.yaw = fx, fy - 0.05, SOUTH  # F1을 막 지나 F1이 선에 남는다
    m.free_paths[("G3", "T2")] = (d.site.waypoint_pose("G3"), d.site.waypoint_pose("T2"))
    points = [p[:2] for p in d.core._lane_polyline(m, 2, 0.0)]
    assert d.site.waypoint_pose("F1") in points, "시험 전제: G3은 앞뒤가 지점인 꺾임이다"
    assert d.site.waypoint_pose("G3") in points


def test_simworld_allows_off_lane_path_points_only_beside_a_rounded_corner():
    """horizon 2 Codex 검수: 레인 검사 완화가 모서리와 무관한 잘못된 선까지 받아들이면 안 된다."""
    from dataclasses import replace

    from sim_world import SimWorld
    world = SimWorld(seed=1)
    lane = max(world.site.lanes.values(), key=lambda ln: ln.span)
    mx, my = (lane.ax + lane.bx) / 2, (lane.ay + lane.by) / 2
    side = (-lane.uy * 0.04, lane.ux * 0.04)
    assert not world._path_point_ok(mx + side[0], my + side[1]), "레인 가운데 곁은 안 된다"
    near = (lane.bx - lane.ux * 0.05 + side[0], lane.by - lane.uy * 0.05 + side[1])
    assert world._path_point_ok(*near), "꺾인 지점 곁은 된다"
    world.params = replace(world.params, corner_radius_m=0.0)
    assert not world._path_point_ok(*near), "둥글리기를 끄면 안 된다"
