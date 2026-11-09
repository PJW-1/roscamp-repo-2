"""Check tools/baseline.py without ROS: metric extraction, CSV columns, summary, smoke runs."""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import baseline  # noqa: E402

DT = 0.1


class StubZones:
    def __init__(self):
        self.waiting = {}

    def waiting_for(self, robot):
        return self.waiting.get(robot)


class StubCore:
    def __init__(self, robots):
        self.zones = StubZones()
        self.views = {r: {"state": "MOVING", "path": ["A", "B"]} for r in robots}

    def robot_view(self, robot):
        return self.views[robot]


class StubSite:
    poses = {"A": (0.0, 0.0), "B": (1.0, 0.0), "P": (5.0, 5.0)}

    def waypoint_pose(self, waypoint):
        return self.poses[waypoint]


def robot(x=0.0, y=0.0, **over):
    fields = {"x": x, "y": y, "removed": False, "docked_at": "", "goal_kind": "move",
              "stopped_at": None, "blocked": False}
    return SimpleNamespace(**{**fields, **over})


def stub_world(*names):
    return SimpleNamespace(t=0.0, virtual={n: robot() for n in names}, core=StubCore(names),
                           site=StubSite(), events=[])


def advance(world, meter, **moves):
    """One 0.1 s step: robots named in moves go to the given x, then the meter looks."""
    world.t = round(world.t + DT, 6)
    for name, x in moves.items():
        world.virtual[name].x = x
    meter.observe(world)


# ---- metric extraction ----------------------------------------------------------------------


def test_a_stop_is_moving_then_still_before_the_end_of_the_path():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.1)  # moving
    advance(world, meter, r1=0.1)  # still, B is still ahead: a stop
    advance(world, meter, r1=0.1)  # still again: the same stop
    advance(world, meter, r1=0.2)
    advance(world, meter, r1=0.2)  # a second stop
    assert meter.stops == 2


def test_arriving_at_the_last_waypoint_of_the_path_is_not_a_stop():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.5)
    advance(world, meter, r1=1.0)
    advance(world, meter, r1=1.0)  # at B, the end of the path
    world.core.views["r1"]["path"] = []
    advance(world, meter, r1=1.0)
    assert meter.stops == 0


def test_a_robot_that_never_moved_has_no_stop():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    for _ in range(5):
        advance(world, meter)
    assert meter.stops == 0


def test_waiting_time_is_the_time_each_robot_holds_a_zone_ticket():
    world = stub_world("r1", "r2")
    meter = baseline.Meter(["r1", "r2"], DT)
    world.core.zones.waiting["r1"] = "Z1"
    for _ in range(3):
        advance(world, meter)
    world.core.zones.waiting["r2"] = "Z2"
    del world.core.zones.waiting["r1"]
    advance(world, meter)
    assert meter.wait == pytest.approx({"r1": 0.3, "r2": 0.1})


def test_object_time_counts_every_still_robot_until_the_blocked_robot_moves_again():
    world = stub_world("r1", "r2", "r3")
    meter = baseline.Meter(["r1", "r2", "r3"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.1, r2=0.1, r3=0.1)
    advance(world, meter)  # nobody met an object yet: no object time
    world.virtual["r1"].stopped_at = world.t  # r1 stops in front of an object
    world.virtual["r3"].docked_at = "WORK"  # r3 works at a station: not because of the object
    advance(world, meter)  # r1 and r2 still
    advance(world, meter, r2=0.2)  # r2 moves, r1 still
    world.virtual["r1"].stopped_at = None
    advance(world, meter)  # object gone, r1 still waits for its goal again: r1 and r2
    advance(world, meter, r1=0.2)  # r1 moves again: the window closes
    advance(world, meter)  # everyone still, but the window is closed
    assert meter.object_still == pytest.approx(0.6)
    assert meter.object_hits == 1


def test_parked_robots_do_not_count_as_standing_because_of_the_object():
    world = stub_world("r1", "r2")
    meter = baseline.Meter(["r1", "r2"], DT)
    meter.observe(world)
    world.core.views["r2"]["state"] = "PARKED"
    world.virtual["r1"].stopped_at = 0.0
    advance(world, meter)
    advance(world, meter)
    assert meter.object_still == pytest.approx(0.2)


def test_a_second_robot_stopped_by_an_object_keeps_its_own_window():
    world = stub_world("r1", "r2", "r3")
    meter = baseline.Meter(["r1", "r2", "r3"], DT)
    world.virtual["r3"].docked_at = "WORK"  # never counted
    meter.observe(world)
    world.virtual["r1"].stopped_at = 0.1
    advance(world, meter)  # r1, r2 still: 0.2
    world.virtual["r2"].stopped_at = 0.2  # r2 meets an object while r1's window is open
    advance(world, meter)  # 0.4
    world.virtual["r2"].stopped_at = None  # r2's object is gone, r2 has not moved yet
    advance(world, meter)  # 0.6
    world.virtual["r1"].stopped_at = None
    advance(world, meter, r1=0.1)  # r1 moves: its window closes, r2 still: 0.7
    advance(world, meter, r1=0.2)  # r2 still stands because of its object: 0.8
    advance(world, meter, r1=0.3, r2=0.1)  # r2 moves: its window closes
    advance(world, meter)  # no window open
    assert meter.object_hits == 2
    assert meter.object_still == pytest.approx(0.8)


def test_overlapping_windows_count_a_still_robot_once_per_step():
    world = stub_world("r1", "r2")
    meter = baseline.Meter(["r1", "r2"], DT)
    meter.observe(world)
    world.virtual["r1"].stopped_at = 0.1
    world.virtual["r2"].stopped_at = 0.1
    advance(world, meter)
    assert meter.object_hits == 2 and meter.object_still == pytest.approx(0.2)


def test_a_robot_entering_error_is_counted_once_per_entry():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    for state in ("MOVING", "ERROR", "ERROR", "OUT_OF_SERVICE", "PARKED", "ERROR"):
        world.core.views["r1"]["state"] = state
        advance(world, meter)
    assert meter.error_entries == 2


def test_removed_robots_are_not_measured():
    world = stub_world("r1")
    meter = baseline.Meter(["r1"], DT)
    meter.observe(world)
    advance(world, meter, r1=0.1)
    world.virtual["r1"].removed = True
    world.core.zones.waiting["r1"] = "Z1"
    advance(world, meter)
    assert meter.stops == 0 and meter.wait == {"r1": 0.0}


# ---- scenario to world ----------------------------------------------------------------------


def test_scenario_settings_reach_the_world():
    data = baseline.load_scenario("blocked_lane")
    world = baseline.build_world(data, "lane_map", seed=3, mode="point")
    assert world.seed == 3
    assert world.params.lane_goal_mode == "point"
    assert world.stock == {"A": 6, "B": 6, "C": 6, "D": 6}
    assert world.arm_s_per_item == 4.0 and world.swap_step_s == 4.0 and world.dock_s == 3.0
    assert [o["id"] for o in world.orders] == [f"O-00{i}" for i in range(1, 7)]
    assert world.faults == [{"at_s": 8.0, "robot": "pinky1", "type": "blocked",
                             "duration_s": 15.0}]
    assert world.robots["pinky3"]["localizing_s"] == 2.0


def test_the_default_plan_only_puts_scenarios_on_maps_that_have_their_start_points():
    for map_name, scenarios in baseline.DEFAULT_PLAN.items():
        site = baseline.load_map(map_name)
        for name in scenarios:
            data = baseline.load_scenario(name)
            for spec in data["robots"].values():
                assert spec["start"] in site.zones, (map_name, name)


def test_missing_arm_and_dock_times_take_the_scenario_parser_defaults():
    data = baseline.load_scenario("basic_orders")
    del data["arm"], data["dock"]
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    # fake_robots/scenario.py: 4 s per item, 8 s swap, 3 s dock
    assert (world.arm_s_per_item, world.swap_step_s, world.dock_s) == (4.0, 4.0, 3.0)


def test_a_scenario_whose_central_record_differs_from_the_shelves_is_refused():
    data = {**baseline.load_scenario("basic_orders"), "inventory": {"A": 6, "B": 6, "C": 6}}
    with pytest.raises(ValueError, match="inventory"):
        baseline.build_world(data, "lane_map", seed=1, mode="path")


def test_an_inventory_equal_to_the_summed_stock_is_accepted():
    data = {**baseline.load_scenario("basic_orders"),
            "inventory": {"A": 6, "B": 6, "C": 6, "D": 6}}
    assert baseline.build_world(data, "lane_map", seed=1, mode="path").stock == data["inventory"]


def test_zero_counts_do_not_make_inventory_and_stock_differ():
    data = {**baseline.load_scenario("basic_orders"),
            "inventory": {"A": 6, "B": 6, "C": 6, "D": 6, "E": 0}}
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    assert world.stock == {"A": 6, "B": 6, "C": 6, "D": 6}


def test_an_unknown_fault_type_is_refused():
    data = {**baseline.load_scenario("basic_orders"),
            "faults": [{"at_s": 0, "arm": "arm1", "type": "arm_fail", "count": 3}]}
    with pytest.raises(ValueError, match="arm_fail"):
        baseline.build_world(data, "lane_map", seed=1, mode="path")


# ---- CSV and summary ------------------------------------------------------------------------


def row(seed, finish, finished=True, **over):
    base = {"map": "lane_map", "scenario": "basic_orders", "mode": "path", "seed": seed,
            "finished": finished, "error": "", "orders_total": 6, "orders_shipped": 6,
            "finish_s": finish, "wait_total_s": 10.0, "object_still_s": "",
            "object_hits": "", "interventions": 0, "error_entries": 0, "stops": 4,
            "detours": 0, "wait_pinky1_s": 1.0}
    return {**base, **over}


def test_the_csv_has_the_fixed_columns_then_one_wait_column_per_robot(tmp_path):
    path = tmp_path / "out.csv"
    baseline.write_rows(path, [row(1, 100.0), row(2, 110.0, wait_pinky2_s=2.0)])
    with path.open(encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert reader.fieldnames == [*baseline.COLUMNS, "wait_pinky1_s", "wait_pinky2_s"]
    assert rows[0]["finish_s"] == "100.0" and rows[0]["wait_pinky2_s"] == ""


def test_the_summary_gives_mean_min_max_over_finished_seeds_and_skips_empty_cells():
    rows = [row(1, 100.0, stops=2), row(2, 110.0, stops=4),
            row(3, "", finished=False, stops=9)]
    summary = {(s["metric"]): s for s in baseline.summarize(rows)}
    finish = summary["finish_s"]
    assert (finish["runs"], finish["n"], finish["mean"], finish["min"], finish["max"]) == (
        3, 2, 105.0, 100.0, 110.0)
    # a run cut by the timeout keeps counting waits and stops, so it is left out
    assert summary["stops"]["n"] == 2 and summary["stops"]["mean"] == 3.0
    assert summary["object_still_s"]["n"] == 0 and summary["object_still_s"]["mean"] == ""
    assert summary["finished"]["n"] == 3
    assert summary["finished"]["mean"] == pytest.approx(2 / 3)


def test_the_summary_keeps_map_scenario_and_mode_apart():
    rows = [row(1, 100.0), row(1, 200.0, mode="point")]
    groups = {(s["mode"], s["metric"]): s["mean"] for s in baseline.summarize(rows)}
    assert groups[("path", "finish_s")] == 100.0 and groups[("point", "finish_s")] == 200.0


# ---- smoke runs on the real core ------------------------------------------------------------


def test_a_basic_run_on_the_default_map_finishes_all_six_orders():
    result = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path")
    assert result["finished"] and result["error"] == ""
    assert result["orders_shipped"] == result["orders_total"] == 6
    assert 0 < result["finish_s"] < 900
    assert result["object_still_s"] == "" and result["interventions"] == 0
    assert result["stops"] >= 0 and result["detours"] == 0
    assert set(result) >= {"wait_pinky1_s", "wait_pinky2_s", "wait_pinky3_s"}
    assert result["wait_total_s"] == pytest.approx(
        sum(result[f"wait_pinky{i}_s"] for i in (1, 2, 3)))


def test_the_same_seed_gives_the_same_row():
    def fake_clock_part(row):  # tick_ms_* are wall time
        return {k: v for k, v in row.items() if not k.startswith("tick_ms_")}

    assert (fake_clock_part(baseline.run_one("lane_map", "basic_orders", seed=2, mode="point"))
            == fake_clock_part(baseline.run_one("lane_map", "basic_orders", seed=2,
                                                mode="point")))


def test_a_blocked_lane_run_measures_time_lost_to_the_object():
    result = baseline.run_one("lane_map", "blocked_lane", seed=1, mode="path")
    assert result["finished"]
    assert result["object_hits"] == 1 and result["object_still_s"] >= 15.0


@pytest.mark.parametrize("seed", [1, 3, 5])
def test_a_blocked_lane_run_on_the_segment_map_finishes_with_the_shipped_params(seed):
    # 1-C borrowing switched on in the shipped file left these seeds isolated after a deadlock
    # (docs/worklog_2026-11-09_v2-1c.md), so the shipped file keeps it off for now
    result = baseline.run_one("two_lane_segments", "blocked_lane", seed=seed, mode="path")
    assert result["finished"] and result["error"] == ""


def test_the_box_on_avenue_row_measures_borrowing_although_the_shipped_file_keeps_it_off():
    result, world = baseline.run_with_world("two_lane_avenue_wide", "box_on_avenue", seed=1,
                                            mode="path")
    assert result["finished"] and result["error"] == ""
    assert "ZONE_BORROWED" in world.event_codes()


def test_detours_are_counted_from_the_core_events(tmp_path):
    # the settings of test_detour.feature_world: pinky1 meets an object at 42 s for 15 s
    data = baseline.load_scenario("feature_blocked_lane")
    data.update(orders=[{"at_s": 0, "id": f"O-00{i}", "items": {"AB"[i % 2]: 1, "CD"[i % 2]: 1}}
                        for i in range(1, 7)],
                stock={"arm1": {"A": 20, "B": 20}, "arm2": {"C": 20, "D": 20}},
                faults=[{"at_s": 42.0, "robot": "pinky1", "type": "blocked",
                         "duration_s": 15.0}])
    for spec in data["robots"].values():
        spec.pop("localizing_s", None)
    path = tmp_path / "detour.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    result = baseline.run_one("feature_map", str(path), seed=7, mode="path")
    assert result["finished"] and result["scenario"] == "detour"
    assert result["detours"] >= 1


def test_the_reinstate_scenario_counts_both_operator_actions():
    result = baseline.run_one("lane_map", "reinstate", seed=1, mode="path")
    assert result["finished"], result
    assert result["interventions"] == 2


def test_finish_time_is_the_tick_of_the_last_final_report():
    result, world = baseline.run_with_world("lane_map", "basic_orders", seed=1, mode="path")
    last = max(t for t, c in world.commands if type(c).__name__ == "ReportOrderCmd")
    assert result["finish_s"] == pytest.approx(last)


def test_a_run_cut_before_every_robot_parks_is_not_finished():
    full = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path")
    assert full["finished"] and full["end_s"] > full["finish_s"] + 1.0
    cut = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path",
                           timeout_s=full["finish_s"] + 0.5)
    assert not cut["finished"] and cut["error"] == ""
    assert cut["finish_s"] == full["finish_s"]  # the orders did finish
    assert cut["end_s"] == pytest.approx(full["finish_s"] + 0.5)


def no_order_scenario(tmp_path, faults=()):
    # the fakes' parser wants a shipped order whenever there are faults
    data = {**baseline.load_scenario("basic_orders"), "orders": [], "faults": list(faults),
            "expect": {"orders_shipped": 1} if faults else {}}
    path = tmp_path / "empty.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def test_a_scenario_without_orders_finishes_at_zero(tmp_path):
    result = baseline.run_one("lane_map", no_order_scenario(tmp_path), seed=1, mode="path")
    assert result["error"] == "" and result["finished"]
    assert result["finish_s"] == 0.0 and result["orders_total"] == 0


def test_a_run_ends_when_every_robot_has_been_taken_off_the_floor(tmp_path):
    faults = [{"at_s": 1.0, "robot": r, "type": "operator_remove"}
              for r in ("pinky1", "pinky2", "pinky3")]
    result = baseline.run_one("lane_map", no_order_scenario(tmp_path, faults), seed=1,
                              mode="path")
    assert result["finished"] and result["interventions"] == 3
    assert result["end_s"] < 10.0


def test_a_refused_confirm_clear_leaves_the_robot_on_the_floor():
    data = baseline.load_scenario("basic_orders")
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    world.step()
    world.core.confirm_clear = lambda *args: None
    operator = baseline.Operator(world, [{"at_s": 0.0, "robot": "pinky1",
                                          "type": "operator_remove"}])
    operator.act()
    assert not world.virtual["pinky1"].removed
    assert (operator.accepted, operator.refused) == (0, 1)
    assert operator.pending  # tried again later


def test_a_refused_reinstate_does_not_move_the_robot():
    data = baseline.load_scenario("basic_orders")
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    world.step()
    r = world.virtual["pinky1"]
    before = (r.x, r.y, r.removed)
    operator = baseline.Operator(world, [{"at_s": 0.0, "robot": "pinky1",
                                          "type": "operator_reinstate", "start": "P-3",
                                          "basket": "EMPTY"}])
    operator.act()  # pinky1 was never removed, so the fleet refuses
    assert (r.x, r.y, r.removed) == before
    assert (operator.accepted, operator.refused) == (0, 1)


def test_the_command_line_writes_the_runs_and_the_summary(tmp_path, capsys):
    out = tmp_path / "runs.csv"
    code = baseline.main(["--maps", "lane_map", "--scenarios", "basic_orders", "--modes",
                          "path", "--seeds", "1", "--out", str(out)])
    assert code == 0
    with out.open(encoding="utf-8") as f:
        assert len(list(csv.DictReader(f))) == 1
    summary = tmp_path / "runs_summary.csv"
    with summary.open(encoding="utf-8") as f:
        metrics = {r["metric"] for r in csv.DictReader(f)}
    assert {"finish_s", "wait_total_s", "stops", "detours", "interventions"} <= metrics
    assert "basic_orders" in capsys.readouterr().out


# ---- free areas (version 2 F-1) -------------------------------------------------------------


def test_the_free_area_scenarios_cross_the_plaza_on_the_planned_path():
    for scenario in ("free_area_basic", "free_area_box"):
        row, world = baseline.run_with_world("feature_map_free", scenario, 1, "path")
        assert row["finished"], (scenario, row["error"])
        assert "FREE_PATH" in world.event_codes(), scenario
        assert "FREE_PATH_REJECTED" not in world.event_codes(), scenario
    assert world.boxes and world.box_gap_min > 0.0, "the robot body keeps off the box corners"


def test_a_robot_on_the_straight_line_drives_into_the_box():
    data = baseline.load_scenario("free_area_box")
    world = baseline.build_world(data, "feature_map_free", 1, "path")
    world.planner = lambda *args, **kwargs: None  # no path: the fleet falls back to the chord
    world.boxes_stop = False  # nothing stops it at the box (1-C added the stop)
    with pytest.raises(AssertionError, match="box"):
        world.run(300)


def test_the_box_scenario_runs_in_path_mode_only(tmp_path):
    """point 모드의 가짜 로봇은 상자를 비켜 갈 경로를 스스로 만들지 않는다(Nav2와 다르다)."""
    out = tmp_path / "runs.csv"
    assert baseline.main(["--maps", "feature_map_free", "--scenarios", "free_area_box",
                          "--seeds", "1", "--out", str(out)]) == 0
    with out.open(encoding="utf-8") as f:
        assert [r["mode"] for r in csv.DictReader(f)] == ["path"]


def test_the_free_floor_of_the_world_is_the_polygon_itself():
    """출입구 옆 허용(half_width)은 SimWorld의 바닥 검사에 쓰지 않는다."""
    world = baseline.build_world(baseline.load_scenario("free_area_basic"), "feature_map_free", 1,
                                 "path")
    assert world._on_free_floor(1.5, 0.7)
    assert not world._on_free_floor(1.5, 0.47), "가장자리에서 0.03 m 밖, 출입구에서 멀다"


# ---- free areas, several robots (version 2 F-2) ---------------------------------------------


def test_two_robots_cross_the_plaza_at_once_on_the_crossing_map():
    """free_area_crossing: 출고 뒤 광장 동쪽과 대기 줄에서 WORK로 가는 서쪽이 칸 열이 달라
    두 대가 동시에 광장 칸을 쥔다. 몸 아래 칸은 늘 그 로봇 것이다(SimWorld 불변식)."""
    assert "free_area_crossing" in baseline.DEFAULT_PLAN["feature_map_crossing"]
    for seed in (1, 2):
        row, world = baseline.run_with_world("feature_map_crossing", "free_area_crossing", seed,
                                             "path")
        assert row["finished"], (seed, row["error"])
        assert world.cells_shared_max >= 2, seed
        assert world.cell_checks > 0
        assert "FREE_CELLS" in world.event_codes()


def test_the_crossing_map_leaves_room_to_stop_short_on_every_exit_lane():
    """Gazebo에서 로봇은 지점 앞 도착 허용치(arrival 0.15 m) 안에 선다. 구역을 떠나는 레인이
    release_distance_m + 0.15보다 짧으면 끝 지점에 선 로봇이 앞 구역을 못 놓고 다음 구역도
    청하지 않아 멈춘다(첫 Gazebo 실행에서 O->F0 0.25 m로 생김). 출입구로 들어가는 레인은
    뺀다: 이 지도를 쓰는 칸 모드에서는 광장 경로와 칸을 먼저 쥐어 목표가 출입구에서 끝나지
    않는다(F-1 모드는 광장 경로를 기다리는 동안 출입구에서 설 수 있다)."""
    from pathlib import Path

    from fleet_core.traffic.lane_map import load_lane_map
    maps = Path(__file__).resolve().parents[3] / "sim" / "fake_robots" / "maps"
    site = load_lane_map(str(maps / "feature_map_crossing.yaml"))
    gates = {g for area in site.free_areas.values() for g in area.gates}
    short = [(a, b, round(lane.span, 3)) for (a, b), lane in site.lanes.items()
             if site.zone_of[a] != site.zone_of[b] and b not in gates
             and lane.span < 0.2 + 0.15 + 0.005]  # 해제는 s > 0.2(엄격)라 5 mm 더
    assert short == []


def test_the_crossing_map_has_no_lane_conflict_pairs():
    """2차 Gazebo 실행: O로 들어오는 E2->O와 O에서 나가는 O->F0가 예각이라 (F0, O)가 충돌 쌍이었다.
    O에서 떠나려는 로봇의 F0 요청이 O를 먼저 기다린 E2의 로봇 뒤에 서서 둘이 교착했다
    (DEADLOCK_UNRESOLVED, 격리). 이 지도에는 레인끼리의 충돌 쌍을 두지 않는다(자유 구역 단위가
    든 쌍은 칸 모드에서 칸 쌍으로 바뀐다)."""
    from pathlib import Path

    from fleet_core.traffic.lane_map import load_lane_map
    maps = Path(__file__).resolve().parents[3] / "sim" / "fake_robots" / "maps"
    site = load_lane_map(str(maps / "feature_map_crossing.yaml"))
    lanes_only = [pair for pair in site.conflict_pairs(2 * 0.08, 0.2)
                  if not pair & set(site.free_areas)]
    assert lanes_only == []


# ---- borrowing the neighbour lane (version 2 1-C) -------------------------------------------


def test_box_on_avenue_borrows_round_the_box_in_path_mode_only():
    """box_on_avenue: 동쪽행 차선 상자를 서쪽행 차선을 빌려 비켜 간다. 막힘 표시와 오류 없이 끝나고,
    로봇은 늘 쥔 구역 바닥 곁에 있다(outside_held 0). point 모드는 돌리지 않는다."""
    assert "box_on_avenue" in baseline.DEFAULT_PLAN["two_lane_avenue_wide"]
    assert "box_on_avenue" in baseline.PATH_ONLY
    row, world = baseline.run_with_world("two_lane_avenue_wide", "box_on_avenue", 1, "path")
    assert row["finished"], row["error"]
    seen = world.event_codes()
    assert {"DETOUR_PLANNED", "ZONE_BORROWED", "DETOUR_DONE", "ZONE_RETURNED"} <= set(seen)
    assert "LANE_BLOCKED" not in seen
    assert world.outside_held == 0


# ---- columns shared with the free-floor comparison (compare_floor.py) -----------------------


def test_spacing_takes_the_smallest_true_gap_after_each_step_and_the_distance_driven():
    world = stub_world("r1", "r2", "r3")
    world.virtual["r2"].x = 1.0
    world.virtual["r3"].y = 2.0
    spacing = baseline.Spacing(world)
    assert spacing.min_gap == float("inf") and spacing.path_m == 0.0  # the start is not a step
    world.virtual["r1"].x = 0.3
    spacing.observe(world)
    assert spacing.min_gap == pytest.approx(0.7) and spacing.path_m == pytest.approx(0.3)
    world.virtual["r2"].removed = True  # off the floor: no gap, no distance
    world.virtual["r2"].x = 0.3
    world.virtual["r1"].x = 0.0
    spacing.observe(world)
    assert spacing.min_gap == pytest.approx(0.7) and spacing.path_m == pytest.approx(0.6)


def test_tick_times_are_taken_around_every_fleet_tick_and_the_answer_is_kept():
    world = SimpleNamespace(core=SimpleNamespace(tick=lambda: ["out"]))
    ticks = baseline.time_ticks(world)
    assert world.core.tick() == ["out"] and world.core.tick() == ["out"]
    assert len(ticks) == 2 and all(t >= 0.0 for t in ticks)


def test_tick_statistics_are_milliseconds_with_the_p99_index_of_compare_floor():
    ticks = [i / 1000 for i in range(1, 101)]  # 1 .. 100 ms
    stats = baseline.tick_stats(ticks)
    # compare_floor.py: sorted(ticks)[int(0.99 * (len - 1))] -> index 98 -> 99 ms
    assert stats == {"tick_ms_mean": 50.5, "tick_ms_max": 100.0, "tick_ms_p99": 99.0}
    assert baseline.tick_stats([]) == {"tick_ms_mean": "", "tick_ms_max": "",
                                       "tick_ms_p99": ""}


def test_a_run_fills_the_free_floor_columns():
    result, world = baseline.run_with_world("lane_map", "basic_orders", seed=1, mode="path")
    assert result["finished"]
    assert 0.0 < result["min_gap_m"] < 5.0 and result["path_m"] > 10.0
    assert 0.0 < result["tick_ms_mean"] <= result["tick_ms_p99"] <= result["tick_ms_max"]
    codes = world.event_codes()
    assert result["deadlocks"] == codes.count("DEADLOCK_DETECTED")
    assert result["unresolved"] == codes.count("DEADLOCK_UNRESOLVED")
    assert result["isolations"] == codes.count("ROBOT_ISOLATED")
    assert result["violations"] == 0
    assert {"min_gap_m", "path_m", "tick_ms_mean", "tick_ms_max", "tick_ms_p99", "deadlocks",
            "unresolved", "isolations", "violations"} <= set(baseline.COLUMNS)


def test_deadlock_and_isolation_columns_count_their_own_events():
    codes = ["DEADLOCK_DETECTED", "DETOUR", "DEADLOCK_DETECTED", "DEADLOCK_UNRESOLVED",
             "ROBOT_ISOLATED", "DEADLOCK_RETREAT", "DETOUR", "DETOUR", "DEADLOCK_UNRESOLVED",
             "DEADLOCK_DETECTED", "DEADLOCK_DETECTED"]
    events = [SimpleNamespace(code=c) for c in codes]
    # every count differs, so a swapped mapping fails
    assert baseline.event_counts(events) == {"detours": 3, "deadlocks": 4, "unresolved": 2,
                                             "isolations": 1}


def test_a_broken_invariant_counts_as_one_violation(monkeypatch):
    import sim_world

    def broken(self):
        if self.t >= 1.0:
            raise AssertionError("made up")

    monkeypatch.setattr(sim_world.SimWorld, "_check_invariants", broken)
    result = baseline.run_one("lane_map", "basic_orders", seed=1, mode="path")
    assert result["error"].startswith("invariant at 1.0 s") and not result["finished"]
    assert result["violations"] == 1


def test_speed_and_noise_options_replace_every_robot_setting():
    data = baseline.load_scenario("basic_orders")
    world = baseline.build_world(data, "lane_map", seed=1, mode="path", speed=0.12, noise=0.05)
    assert {(r.speed, r.noise) for r in world.virtual.values()} == {(0.12, 0.05)}
    assert data["robots"]["pinky2"]["speed_mps"] == 0.18  # the loaded scenario is not changed


def test_a_robot_without_speed_or_noise_takes_the_fake_robot_defaults():
    data = baseline.load_scenario("basic_orders")
    for spec in data["robots"].values():
        del spec["speed_mps"], spec["pose_noise_m"]
    world = baseline.build_world(data, "lane_map", seed=1, mode="path")
    # fake_robots/scenario.py RobotCfg: 0.2 m/s, no noise
    assert {(r.speed, r.noise) for r in world.virtual.values()} == {(0.2, 0.0)}


def test_a_params_file_replaces_the_shipped_one_but_not_the_goal_mode(tmp_path):
    text = (baseline.PARAMS_DIR / "fleet_params.yaml").read_text(encoding="utf-8")
    path = tmp_path / "small.yaml"
    path.write_bytes(text.replace("release_distance_m: 0.2", "release_distance_m: 0.17")
                     .replace("lane_goal_mode: path", "lane_goal_mode: point").encode())
    data = baseline.load_scenario("basic_orders")
    world = baseline.build_world(data, "lane_map", seed=1, mode="path", params=str(path))
    assert world.params.release_distance_m == 0.17 and world.params.lane_goal_mode == "path"
    shipped = baseline.build_world(data, "lane_map", seed=1, mode="path",
                                   params="fleet_params")  # a name in fleet_core/data
    assert shipped.params.release_distance_m == 0.2


def test_scenarios_are_also_found_in_the_gazebo_and_tool_folders():
    assert baseline.load_scenario("gz_warehouse")["map"] == "warehouse"
    assert len(baseline.load_scenario("long_orders_100")["orders"]) == 100
    with pytest.raises(FileNotFoundError):
        baseline.load_scenario("no_such_scenario")


def test_the_long_run_builds_on_the_default_map():
    world = baseline.build_world(baseline.load_scenario("long_orders_100"), "lane_map", seed=1,
                                 mode="path")
    assert len(world.orders) == 100 and world.params.lane_goal_mode == "path"


def test_the_real_arena_row_runs_slow_noisy_robots_with_the_arena_params():
    result, world = baseline.run_with_world("real_arena", "gz_real_arena", 1, "path", speed=0.12,
                                            noise=0.03, params="fleet_params_real_arena")
    assert result["finished"], result["error"]
    assert result["orders_shipped"] == 8 and world.params.release_distance_m == 0.17
    assert {(r.speed, r.noise) for r in world.virtual.values()} == {(0.12, 0.03)}


def test_the_table_keeps_three_decimals_for_millisecond_ticks_and_gaps():
    rows = [row(1, 100.0, tick_ms_mean=0.123, min_gap_m=0.3068),
            row(2, 100.0, tick_ms_mean=0.141, min_gap_m=0.35)]
    line = baseline.table(baseline.summarize(rows)).splitlines()[2]
    assert "0.132 (0.123-0.141)" in line and "0.328 (0.307-0.350)" in line


def test_the_command_line_passes_speed_noise_and_params(tmp_path, monkeypatch):
    seen = []

    def fake_run(map_, scenario, seed, mode, timeout_s=None, **options):
        seen.append(options)
        return baseline._blank_row(map_, scenario, mode, seed, ["pinky1"], 0)

    monkeypatch.setattr(baseline, "run_one", fake_run)
    baseline.main(["--maps", "lane_map", "--scenarios", "basic_orders", "--modes", "path",
                   "--seeds", "1", "--speed", "0.12", "--noise", "0.03", "--params",
                   "fleet_params", "--out", str(tmp_path / "o.csv")])
    assert seen == [{"speed": 0.12, "noise": 0.03, "params": "fleet_params"}]
