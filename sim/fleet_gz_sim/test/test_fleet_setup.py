"""gz.launch.py가 시나리오에서 정하는 것: 핑키를 놓는 자리와 방향, 돌릴 수 없는 시나리오."""
import math
from itertools import pairwise
from pathlib import Path

import pytest

pytest.importorskip("fleet_core")
pytest.importorskip("fake_robots")
from fleet_gz_sim.fleet_setup import ScenarioUnfit, load, scenario_path, start_poses  # noqa: E402


def share():
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return get_package_share_directory("fake_robots")
    except PackageNotFoundError:
        pytest.skip("fake_robots is not installed")


def test_pinkys_start_on_their_scenario_waypoints_facing_along_the_lane_in():
    """basic_orders: pinky1 P-6, pinky2 P-5, pinky3 P-4. 대기 줄을 따라 들어온 방향을 본다."""
    site, scenario = load(scenario_path("basic_orders", share()))
    poses = start_poses(site, scenario)
    assert list(poses) == ["pinky1", "pinky2", "pinky3"]
    assert poses["pinky1"] == pytest.approx((1.20, 2.05, 0.0))  # P-5 → P-6, 동쪽
    assert poses["pinky2"] == pytest.approx((0.80, 2.05, math.atan2(0.10, 0.45)))  # P-4 → P-5
    assert poses["pinky3"] == pytest.approx((0.35, 1.95, math.pi / 2))  # P-3 → P-4, 북쪽


def test_a_scenario_with_its_own_map_places_pinkys_on_that_map():
    """free_area_crossing은 feature_map_crossing을 쓴다(버전 2 F-2). 시작 지점도 그 지도에서."""
    site, scenario = load(scenario_path("free_area_crossing", share()))
    assert "PLAZA" in site.free_areas
    assert start_poses(site, scenario)["pinky1"][:2] == pytest.approx((1.10, 0.48))  # P-3


def test_a_scenario_with_robot_faults_is_refused():
    """로봇 고장(stuck 등)은 가짜 핑키가 일으킨다. 진짜 pinky_fsm에는 일으킬 곳이 없다."""
    with pytest.raises(ScenarioUnfit, match="stuck"):
        load(scenario_path("stuck_robot", share()))


def test_a_scenario_is_a_name_in_fake_robots_or_a_file(tmp_path):
    assert scenario_path("basic_orders", share()).name == "basic_orders.yaml"
    mine = tmp_path / "mine.yaml"
    mine.write_text("{}")
    assert scenario_path(str(mine), share()) == mine
    with pytest.raises(ScenarioUnfit, match="nowhere"):
        scenario_path("no_such_scenario", share())


def test_a_scenario_name_is_looked_up_in_each_share_in_order(tmp_path):
    """gz.launch.py는 fleet_gz_sim의 scenarios를 먼저 보고, 없으면 fake_robots를 본다."""
    first, second = tmp_path / "first", tmp_path / "second"
    for folder in (first, second):
        (folder / "scenarios").mkdir(parents=True)
        (folder / "scenarios" / "both.yaml").write_text("{}")
    (second / "scenarios" / "only_second.yaml").write_text("{}")
    assert scenario_path("both", first, second) == first / "scenarios" / "both.yaml"
    assert scenario_path("only_second", first, second) == second / "scenarios" / "only_second.yaml"
    with pytest.raises(ScenarioUnfit, match="nowhere"):
        scenario_path("no_such_scenario", first, second)


ROAM = Path(__file__).resolve().parent.parent / "scenarios" / "gz_roam.yaml"


def test_the_roaming_scenario_keeps_three_pinkies_busy_for_twenty_minutes():
    """gz_roam: 주문이 끊기지 않고 들어와 3대가 20분 넘게 계속 다음 목적지를 받는다."""
    site, scenario = load(ROAM)
    assert list(scenario.robots) == ["pinky1", "pinky2", "pinky3"]
    times = [order.at_s for order in scenario.orders]
    assert times == sorted(times)
    assert times[-1] >= 1200
    assert max(b - a for a, b in pairwise(times)) <= 60  # 1분 넘게 주문이 끊기지 않는다
    assert scenario.timeout_s > times[-1] + 600  # 마지막 주문도 끝낼 시간


def test_the_roaming_scenario_has_stock_for_every_order():
    """재고가 모자라면 주문이 HELD로 멈춰 로봇이 쉰다."""
    _, scenario = load(ROAM)
    stock = {item: count for items in scenario.stock.values() for item, count in items.items()}
    for item in stock:
        wanted = sum(order.items.get(item, 0) for order in scenario.orders)
        assert wanted <= stock[item], item


PKG = Path(__file__).resolve().parents[1]
FAKE = PKG.parent / "fake_robots"  # 소스 폴더도 share와 같은 모양(scenarios/, maps/)


def test_a_scenario_without_a_map_runs_in_the_lane_arena():
    from fleet_gz_sim.fleet_setup import arena_for
    arena = arena_for(ROAM, PKG, FAKE)
    assert arena.name == "lane_arena" and arena.lane_map == ""
    assert arena.world == str(PKG / "worlds" / "lane_arena.sdf")
    assert arena.nav_map == str(PKG / "maps" / "lane_arena.yaml")
    assert arena.nav_map_fine == str(PKG / "maps" / "lane_arena_fine.yaml")


def test_a_warehouse_scenario_gets_the_generated_warehouse_world_and_maps():
    """map: warehouse -> the world and Nav2 maps arena.py builds from sites/warehouse.yaml,
    and the lane map that site names."""
    from fleet_gz_sim import arena as arena_module
    from fleet_gz_sim.fleet_setup import arena_for
    arena = arena_for(WAREHOUSE, PKG, FAKE)
    site = arena_module.load_site(PKG / "sites" / "warehouse.yaml")
    assert arena.name == site.name == "warehouse"
    world, nav, _, fine, _ = site.outputs()
    assert (arena.world, arena.nav_map, arena.nav_map_fine) == tuple(
        str(PKG / rel) for rel in (world, nav, fine))
    assert arena.lane_map == str(FAKE / "maps" / f"{site.lane_map}.yaml")
    for path in (arena.world, arena.nav_map, arena.nav_map_fine, arena.lane_map):
        assert Path(path).is_file(), path


def test_a_scenario_map_without_a_site_runs_in_the_lane_arena_with_that_lane_map():
    """버전 2 F-2: sites/에 없는 지도(feature_map_crossing)는 lane_arena 월드에서 레인 지도만
    바꾼다."""
    from fleet_gz_sim.fleet_setup import arena_for
    arena = arena_for(FAKE / "scenarios" / "free_area_crossing.yaml", PKG, FAKE)
    assert arena.name == "lane_arena"
    assert arena.world == str(PKG / "worlds" / "lane_arena.sdf")
    assert arena.nav_map == str(PKG / "maps" / "lane_arena.yaml")
    assert arena.nav_map_fine == str(PKG / "maps" / "lane_arena_fine.yaml")
    assert arena.lane_map == str(FAKE / "maps" / "feature_map_crossing.yaml")


def test_a_site_names_the_lane_map_its_world_was_built_from(tmp_path):
    """sites/<map>.yaml의 lane_map으로 월드의 레인 그림과 상자 여유를 만들었으므로 관제도 그 레인
    지도를 쓴다(지도 이름과 달라도)."""
    from fleet_gz_sim.fleet_setup import arena_for
    gz = tmp_path / "gz"
    for rel in ("sites/foo.yaml", "worlds/foo.sdf", "maps/foo.yaml", "maps/foo_fine.yaml"):
        (gz / rel).parent.mkdir(parents=True, exist_ok=True)
        (gz / rel).write_text("lane_map: two_lane_segments\n" if rel.startswith("sites") else "")
    mine = tmp_path / "mine.yaml"
    mine.write_text("map: foo\n")
    arena = arena_for(mine, gz, FAKE)
    assert arena.world == str(gz / "worlds" / "foo.sdf")
    assert arena.lane_map == str(FAKE / "maps" / "two_lane_segments.yaml")


def test_an_empty_lane_map_from_a_site_means_the_default_map_when_loading(tmp_path):
    """site에 lane_map이 없으면 기본 레인 지도다. load는 그 빈 값을 시나리오의 map으로 바꾸지
    않는다(인자를 주지 않았을 때만 시나리오의 map을 쓴다)."""
    mine = tmp_path / "scenarios" / "mine.yaml"
    mine.parent.mkdir()
    text = (FAKE / "scenarios" / "basic_orders.yaml").read_text(encoding="utf-8")
    mine.write_text("map: foo\n" + text, encoding="utf-8")
    site, scenario = load(mine, "")
    assert set(site.lanes) == set(load(FAKE / "scenarios" / "basic_orders.yaml")[0].lanes)
    assert list(scenario.robots) == ["pinky1", "pinky2", "pinky3"]


def test_a_scenario_file_finds_its_map_in_the_maps_folder_beside_its_own_first(tmp_path):
    """버전 2 scenario_map_file처럼, 파일로 준 시나리오는 자기 옆 maps 폴더의 지도를 먼저 쓴다."""
    from fleet_gz_sim.fleet_setup import arena_for
    (tmp_path / "scenarios").mkdir()
    (tmp_path / "maps").mkdir()
    mine = tmp_path / "scenarios" / "mine.yaml"
    mine.write_text("map: two_lane_segments\n")
    own = tmp_path / "maps" / "two_lane_segments.yaml"
    own.write_bytes((FAKE / "maps" / "two_lane_segments.yaml").read_bytes())
    arena = arena_for(mine, PKG, FAKE)
    assert arena.name == "lane_arena" and arena.lane_map == str(own)


def test_a_scenario_map_without_a_gazebo_world_is_refused(tmp_path):
    from fleet_gz_sim.fleet_setup import arena_for
    mine = tmp_path / "mine.yaml"
    mine.write_text("map: no_such_map\n")
    with pytest.raises(ScenarioUnfit, match="no_such_map"):
        arena_for(mine, PKG, FAKE)


def test_the_scenario_map_drives_loading_and_start_poses():
    """warehouse_rush starts on P-3..P-1 of the warehouse map, which the default map lacks."""
    site, scenario = load(FAKE / "scenarios" / "warehouse_rush.yaml")
    poses = start_poses(site, scenario)
    assert poses["pinky2"][:2] == pytest.approx((3.0, 3.15))  # P-2
    from fleet_gz_sim.fleet_setup import arena_for
    site, scenario = load(WAREHOUSE, arena_for(WAREHOUSE, PKG, FAKE).lane_map)
    parking = {next(iter(site.zones[name])) for name in ("P-1", "P-2", "P-3")}
    assert {cfg.start for cfg in scenario.robots.values()} == parking


WAREHOUSE = PKG / "scenarios" / "gz_warehouse.yaml"


def _warehouse():
    from fleet_gz_sim.fleet_setup import arena_for
    return load(WAREHOUSE, arena_for(WAREHOUSE, PKG, FAKE).lane_map)[1]


def test_the_warehouse_scenario_keeps_three_pinkies_busy_for_twenty_minutes():
    scenario = _warehouse()
    assert list(scenario.robots) == ["pinky1", "pinky2", "pinky3"]
    times = [order.at_s for order in scenario.orders]
    assert times == sorted(times) and times[0] <= 10
    assert times[-1] >= 1200
    assert max(b - a for a, b in pairwise(times)) <= 45
    assert scenario.timeout_s > times[-1] + 600
    assert scenario.arm.seconds_per_item <= 3.0 and scenario.dock.duration_s <= 3.0
    assert scenario.arm.fail_rate == 0.0 and not scenario.faults


def test_the_warehouse_scenario_has_stock_for_every_order():
    scenario = _warehouse()
    stock = {item: count for items in scenario.stock.values() for item, count in items.items()}
    for order in scenario.orders:
        assert set(order.items) <= set(stock), order.id
    for item in stock:
        assert sum(order.items.get(item, 0) for order in scenario.orders) <= stock[item], item


NONSTOP = Path(__file__).resolve().parent.parent / "scenarios" / "gz_nonstop.yaml"
WAREHOUSE_NONSTOP = NONSTOP.with_name("gz_warehouse_nonstop.yaml")


@pytest.mark.parametrize("path", [NONSTOP, WAREHOUSE_NONSTOP], ids=lambda p: p.stem)
def test_the_nonstop_scenario_keeps_a_backlog_so_each_pinky_gets_its_next_goal_on_arrival(path):
    """gz_nonstop: orders come faster than three pinkys finish them (6 orders took 253 s on
    Gazebo, about 40 s each), so a pinky that finishes an order already has the next one.
    gz_warehouse_nonstop is the same on the warehouse map, where the trips are longer."""
    from fleet_gz_sim.fleet_setup import arena_for
    _, scenario = load(path, arena_for(path, PKG, FAKE).lane_map)
    assert list(scenario.robots) == ["pinky1", "pinky2", "pinky3"]
    times = [order.at_s for order in scenario.orders]
    assert len(times) >= 15 and times == sorted(times)
    assert max(b - a for a, b in pairwise(times)) <= 15
    assert scenario.timeout_s >= 40 * len(times) + 300  # time for the backlog to drain
    stock = {item: count for items in scenario.stock.values() for item, count in items.items()}
    for item in stock:
        assert sum(order.items.get(item, 0) for order in scenario.orders) <= stock[item], item


def test_an_obstacle_schedule_is_off_a_share_name_or_a_file(tmp_path):
    """obstacles 인자: 비면 장애물 없음, 이름은 share의 obstacles 폴더, 아니면 파일."""
    from fleet_gz_sim.fleet_setup import obstacles_path
    from fleet_gz_sim.obstacles import ScheduleError
    (tmp_path / "obstacles").mkdir()
    (tmp_path / "obstacles" / "roam.yaml").write_text("{}")
    mine = tmp_path / "mine.yaml"
    mine.write_text("{}")
    assert obstacles_path("", tmp_path) == ""
    assert obstacles_path("roam", tmp_path) == str(tmp_path / "obstacles" / "roam.yaml")
    assert obstacles_path(str(mine), tmp_path) == str(mine)
    with pytest.raises(ScheduleError, match="nowhere"):
        obstacles_path("no_such_schedule", tmp_path)
