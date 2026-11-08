"""창고 사이트: 레인 지도와 상자 배치가 서로 맞는지, 장애물 일정이 그 지도에서 읽히는지."""
import itertools
import math
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena, obstacles  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
MAPS = PKG.parent / "fake_robots" / "maps"
SITE = yaml.safe_load((PKG / "sites" / "warehouse.yaml").read_text(encoding="utf-8"))
LANES = lane_map_module.load_lane_map(MAPS / f"{SITE['lane_map']}.yaml")
BOXES = [arena.Box(b["name"], b["x0"], b["y0"], b["x1"], b["y1"]) for b in SITE["boxes"]]
CLEAR_M = 0.25  # 레인에서 벽·상자까지
PARALLEL_M = 0.35  # 나란한 레인 사이
EPS = 1e-9


def test_site_names_an_existing_lane_map():
    assert (MAPS / f"{SITE['lane_map']}.yaml").is_file()


def test_waypoints_and_lanes_keep_clear_of_the_walls():
    x0, y0, x1, y1 = SITE["bounds_m"]
    for (ax, ay), (bx, by), name in arena.lane_segments(LANES):
        for x, y in ((ax, ay), (bx, by)):
            gap = min(x - x0, x1 - x, y - y0, y1 - y)
            assert gap >= CLEAR_M - EPS, f"{name} is {gap:.3f} m from a wall"


def test_every_lane_keeps_clear_of_every_box():
    for a, b, name in arena.lane_segments(LANES):
        for box in BOXES:
            gap = arena.clearance(a, b, box)
            assert gap >= CLEAR_M - EPS, f"{name} is {gap:.3f} m from {box.name}"


def test_boxes_are_valid_inside_and_apart():
    x0, y0, x1, y1 = SITE["bounds_m"]
    names = [b["name"] for b in SITE["boxes"]]
    assert len(names) == len(set(names))
    for raw, box in zip(SITE["boxes"], BOXES, strict=True):
        assert raw["kind"] in ("rack", "table", "pillar", "arm"), box.name
        assert raw["height_m"] > 0, box.name
        assert box.x0 < box.x1 and box.y0 < box.y1, box.name
        assert x0 - EPS <= box.x0 and box.x1 <= x1 + EPS, box.name
        assert y0 - EPS <= box.y0 and box.y1 <= y1 + EPS, box.name
    kinds = {raw["name"]: raw["kind"] for raw in SITE["boxes"]}
    for one, other in itertools.combinations(BOXES, 2):
        overlap = (min(one.x1, other.x1) - max(one.x0, other.x0) > EPS
                   and min(one.y1, other.y1) - max(one.y0, other.y0) > EPS)
        on_table = {kinds[one.name], kinds[other.name]} == {"arm", "table"}
        assert not overlap or on_table, f"{one.name} overlaps {other.name}"


def _inside(inner, outer) -> bool:
    return (outer.x0 - EPS <= inner.x0 and inner.x1 <= outer.x1 + EPS
            and outer.y0 - EPS <= inner.y0 and inner.y1 <= outer.y1 + EPS)


def test_each_arm_stands_on_its_table_by_its_dock():
    """arm1, arm2 at WORK and arm3 at OUTBOUND, each on top of its table (base taller than it)."""
    raw = {b["name"]: b for b in SITE["boxes"]}
    box = {b.name: b for b in BOXES}
    arms = sorted(n for n, b in raw.items() if b["kind"] == "arm")
    assert arms == ["arm1", "arm2", "arm3"]
    for arm in arms:
        table = f"table_{arm}"
        assert _inside(box[arm], box[table]), f"{arm} is not on {table}"
        assert raw[arm]["height_m"] > raw[table]["height_m"], arm
        station = "OUTBOUND" if arm == "arm3" else "WORK"
        assert arm in LANES.stations[station].arms
        x, y = LANES.waypoint_pose(LANES.stations[station].dock)
        a = box[arm]
        assert math.hypot(max(a.x0 - x, 0, x - a.x1), max(a.y0 - y, 0, y - a.y1)) < 0.5, arm
    for other in ("rack", "table"):  # 바닥이 창고처럼 보이게: 출고 쪽 포장대, 작업 쪽 대기 랙
        assert any(n.startswith(f"{'packing' if other == 'table' else 'staging'}_")
                   and b["kind"] == other for n, b in raw.items()), other


def test_tables_stand_by_both_docks():
    tables = [b for raw, b in zip(SITE["boxes"], BOXES, strict=True) if raw["kind"] == "table"]
    for station in ("WORK", "OUTBOUND"):
        x, y = LANES.waypoint_pose(LANES.stations[station].dock)
        near = min(math.hypot(max(t.x0 - x, 0, x - t.x1), max(t.y0 - y, 0, y - t.y1))
                   for t in tables)
        assert near < 0.4, f"no table next to the {station} dock"


def test_parallel_lanes_are_far_enough_apart():
    segments = arena.lane_segments(LANES)
    for (a1, a2, one), (b1, b2, other) in itertools.combinations(segments, 2):
        if {a1, a2} & {b1, b2}:
            continue  # 지점에서 만나는 레인
        ux, uy = a2[0] - a1[0], a2[1] - a1[1]
        vx, vy = b2[0] - b1[0], b2[1] - b1[1]
        sine = abs(ux * vy - uy * vx) / (math.hypot(ux, uy) * math.hypot(vx, vy))
        if sine > math.sin(math.radians(15)):
            continue
        gap = lane_map_module._segment_distance(a1, a2, b1, b2)
        assert gap >= PARALLEL_M - EPS, f"{one} and {other} are {gap:.3f} m apart"


def test_waypoints_are_spaced_out():
    for lane in LANES.lanes.values():
        assert lane.span >= 0.35 - EPS, f"{lane.a}->{lane.b} is {lane.span:.3f} m"


def test_stations_and_parking_match_the_contract():
    work, out = LANES.stations["WORK"], LANES.stations["OUTBOUND"]
    assert (work.server, work.arms, work.action) == ("workcell", ("arm1", "arm2"), "fill")
    assert (out.server, out.arms, out.action) == ("arm3", ("arm3",), "swap")
    assert set(LANES.stations) == {"WORK", "OUTBOUND"}
    assert 3 <= len(LANES.parking) <= 4
    # 작업대와 출고대는 양 끝에 둔다
    assert abs(LANES.waypoint_pose(work.dock)[0] - LANES.waypoint_pose(out.dock)[0]) > 4.0


def test_obstacle_schedule_parses_on_the_warehouse_map():
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    assert 5 <= len(schedule.obstacles) <= 7
    assert schedule.repeat_every_s is not None
    for one in schedule.obstacles:
        assert 20 <= one.duration_s <= 40, one.label


def test_every_blocked_lane_leaves_a_way_around():
    """A robot at the tail of a blocked lane must have another exit, so the fleet can DETOUR
    instead of waiting for the box to go."""
    schedule = obstacles.load_schedule(PKG / "obstacles" / "warehouse.yaml", LANES)
    for one in schedule.obstacles:
        a, b = one.lane
        tails = [(a, b)] if (b, a) not in LANES.lanes else [(a, b), (b, a)]
        for tail, head in tails:
            exits = [lane for lane in LANES.lanes if lane[0] == tail and lane[1] != head]
            assert exits, f"{one.label}: {tail} has no other exit than {head}"
