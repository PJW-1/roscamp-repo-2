"""실제 경기장(2.05 x 1.05 m, 핑키 2대): 레인 지도, 사이트의 상자, 생성한 월드·지도, 시나리오."""
import itertools
import math
from pathlib import Path

import pytest
import yaml

lane_map_module = pytest.importorskip("fleet_core.traffic.lane_map")
from fleet_gz_sim import arena  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
MAPS = PKG.parent / "fake_robots" / "maps"
SITE_FILE = PKG / "sites" / "real_arena.yaml"
SITE = arena.load_site(SITE_FILE)
LANES = lane_map_module.load_lane_map(MAPS / "real_arena.yaml")
PARAMS_FILE = Path(lane_map_module.__file__).parents[1] / "data" / "fleet_params_real_arena.yaml"
RELEASE_M = 0.17  # 가장 짧은 레인이 0.174 m라 기본 0.2 m 대신
BODY_RADIUS_M = 0.08  # 핑키 몸체만, 뒤 트레이는 뺀 값
EPS = 1e-9
BOXES = {b.name: b for b in SITE.features}


def test_the_site_names_the_lane_map_and_a_clearance_the_map_keeps():
    assert SITE.lane_map == "real_arena" and SITE.inside == (0.0, 0.0, 2.05, 1.05)
    assert set(BOXES) == {"bin", "box_small", "block_long", "table_2"}
    value, lane, box = arena.min_clearance(LANES, SITE)
    assert value >= SITE.lane_clearance_m - EPS, f"{lane} is {value:.3f} m from {box}"


def test_the_tightest_spot_is_the_goal_lane_past_the_long_block():
    """브리프: G->PS가 block_long 모서리를 0.106 m로 지난다(몸체 반경 0.08 m면 가장자리 2.6 cm).
    상자 위치는 ±5 cm 추정이고 0.08 m에는 트레이가 빠져 있다."""
    value, lane, box = arena.min_clearance(LANES, SITE)
    assert (lane, box) == ("G-PS", "block_long")
    assert value == pytest.approx(0.106, abs=0.001)
    assert value - BODY_RADIUS_M == pytest.approx(0.026, abs=0.001)


def test_every_lane_keeps_the_body_off_every_wall_and_box():
    for a, b, name in arena.lane_segments(LANES):
        for box in arena.obstacles(SITE):
            gap = arena.clearance(a, b, box)
            assert gap >= BODY_RADIUS_M + 0.02, f"{name} is {gap:.3f} m from {box.name}"


def test_waypoints_are_inside_the_walls_by_the_site_clearance():
    x0, y0, x1, y1 = SITE.inside
    for (ax, ay), (bx, by), name in arena.lane_segments(LANES):
        for x, y in ((ax, ay), (bx, by)):
            gap = min(x - x0, x1 - x, y - y0, y1 - y)
            assert gap >= SITE.lane_clearance_m - EPS, f"{name} is {gap:.3f} m from a wall"


def test_every_lane_is_longer_than_the_release_distance():
    spans = {f"{a}->{b}": lane.span for (a, b), lane in LANES.lanes.items()}
    assert min(spans.values()) > RELEASE_M, spans
    assert min(spans.values()) == pytest.approx(0.174, abs=0.001)  # 브리프: 가장 짧은 레인


def test_the_loop_is_one_way_and_closed():
    """들어가는 길(WORK -> 북쪽 벽 -> OUTBOUND)과 나오는 길이 한 고리. 양방향 구간이 없다."""
    edges = list(LANES.lanes)
    assert all((b, a) not in LANES.lanes for a, b in edges), "a lane has a reverse twin"
    nodes = {n for e in edges for n in e}
    assert all(sum(a == n for a, _ in edges) == 1 for n in nodes), "a place has two ways out"
    assert all(sum(b == n for _, b in edges) == 1 for n in nodes), "a place has two ways in"
    here, seen = edges[0][0], []
    while here not in seen:
        seen.append(here)
        here = next(b for a, b in edges if a == here)
    assert here == edges[0][0] and set(seen) == nodes, "the lanes do not make one cycle"
    assert len(nodes) == 13 and len(edges) == 13


def test_every_dock_and_parking_spot_is_on_the_loop():
    nodes = {n for e in LANES.lanes for n in e}
    for station in LANES.stations.values():
        assert station.dock in nodes
    assert set(LANES.parking) <= nodes


def test_the_stations_follow_the_contract():
    work, out = LANES.stations["WORK"], LANES.stations["OUTBOUND"]
    assert set(LANES.stations) == {"WORK", "OUTBOUND"}
    assert (work.dock, work.server, work.arms, work.action) == (
        "W", "workcell", ("arm1", "arm2"), "fill")
    assert (out.dock, out.server, out.arms, out.action) == ("O", "arm3", ("arm3",), "swap")
    assert LANES.parking == ("P-1", "P-2")


def test_the_way_in_runs_north_of_box_small_and_the_way_out_south_of_table_2():
    order = ["W", "WN", "QO", "EN", "O", "S1", "S2", "S3", "G", "PS", "P-1", "P-2", "QW"]
    for a, b in itertools.pairwise(order):
        assert (a, b) in LANES.lanes, f"{a}->{b}"
    small, table = BOXES["box_small"], BOXES["table_2"]
    y = LANES.waypoint_pose("QO")[1]
    assert y > small.y1 and LANES.waypoint_pose("S1")[1] < table.y0


def test_the_shipped_world_and_maps_are_what_the_generator_makes(tmp_path):
    arena.generate(tmp_path, LANES, site=SITE)
    for name in SITE.outputs():
        assert (PKG / name).read_bytes() == (tmp_path / name).read_bytes(), name


def test_the_world_steps_physics_at_four_milliseconds():
    text = (PKG / "worlds" / "real_arena.sdf").read_text(encoding="utf-8")
    assert "<max_step_size>0.004</max_step_size>" in text
    assert f"{SITE.lane_clearance_m} m" in text


def test_the_shipped_params_differ_from_the_default_only_in_the_release_distance():
    default = yaml.safe_load((PARAMS_FILE.parent / "fleet_params.yaml").read_text("utf-8"))
    mine = yaml.safe_load(PARAMS_FILE.read_text("utf-8"))
    assert mine.pop("release_distance_m") == RELEASE_M
    assert default.pop("release_distance_m") == 0.2
    assert mine == default
    assert RELEASE_M >= BODY_RADIUS_M + 0.03 + 0.2 * (0.2 + 0.1) - EPS
    assert math.isclose(RELEASE_M, 0.17)


# ---- 시나리오 ----
SCENARIO = PKG / "scenarios" / "gz_real_arena.yaml"


def _scenario():
    pytest.importorskip("fake_robots")
    from fleet_gz_sim.fleet_setup import arena_for, load
    found = arena_for(SCENARIO, PKG, MAPS.parent)
    return found, *load(SCENARIO, found.lane_map)


def test_the_real_arena_scenario_runs_in_the_real_arena_world():
    found, _, _ = _scenario()
    assert found.name == "real_arena"
    assert Path(found.world).name == "real_arena.sdf"
    assert Path(found.nav_map).name == "real_arena.yaml"
    assert Path(found.nav_map_fine).name == "real_arena_fine.yaml"
    assert Path(found.lane_map) == MAPS / "real_arena.yaml"


def test_two_pinkys_start_on_the_parking_spots_of_the_map():
    from fleet_gz_sim.fleet_setup import start_poses
    _, site, scenario = _scenario()
    assert list(scenario.robots) == ["pinky1", "pinky2"]
    assert {cfg.start for cfg in scenario.robots.values()} == {"P-1", "P-2"}
    poses = start_poses(site, scenario)
    assert poses["pinky1"][:2] == pytest.approx((0.575, 0.15))  # P-2
    assert poses["pinky2"][:2] == pytest.approx((0.925, 0.20))  # P-1
    x0, y0, x1, y1 = SITE.inside
    for x, y, _ in poses.values():
        assert x0 < x < x1 and y0 < y < y1


def test_the_scenario_sends_eight_orders_twenty_seconds_apart_and_has_stock_for_them():
    _, _, scenario = _scenario()
    assert [o.at_s for o in scenario.orders] == [20 * i for i in range(8)]
    stock = {item: n for items in scenario.stock.values() for item, n in items.items()}
    for order in scenario.orders:
        assert set(order.items) <= set(stock), order.id
    for item in stock:
        assert sum(o.items.get(item, 0) for o in scenario.orders) <= stock[item], item
    assert not scenario.faults and scenario.expect.orders_shipped == 8
