"""ETA 2부: 예상 대기(앞 로봇이 쥔 구역, 줄, 스테이션 작업 남은 시간)와 매니저의 eta()."""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import (
    FILLING,
    SWAPPING,
    OrderOrchestrator,
    _ArmJob,
    _BoundOrder,
)
from fleet_core.traffic.lane_map import LaneMapError, load_lane_map
from fleet_core.traffic.mission import (
    AT_STATION,
    DOCKING,
    ERROR,
    PARKED,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)

REPO = Path(__file__).resolve().parents[3]
SITE = load_lane_map(REPO / "sim/fake_robots/maps/feature_map.yaml")
PARAMS = replace(load_params(), eta_speed_mps=0.2, eta_turn_s_per_rad=0.0, eta_dock_s=3.0,
                 eta_fill_s_per_item=4.0, eta_swap_s=8.0, eta_fill_items_default=2)
EXIT_S = PARAMS.release_distance_m / PARAMS.eta_speed_mps  # 1 s to be clear of a zone


def make(starts, **over):
    now = [0.0]
    core = FleetManager(SITE, replace(PARAMS, **over), list(starts), lambda: now[0])
    for robot, waypoint in starts.items():
        x, y = SITE.waypoint_pose(waypoint)
        for _ in range(PARAMS.release_confirm_samples):
            core.on_robot_state(robot, x, y, 0.0, "IDLE")
    core.on_order_board("RUN", [])
    return core, now


def run(core, robot, phase, path, dest=None, hold=()):
    """robot이 path를 따라가는 중이고 hold 구역을 쥔 상태로 맞춘다."""
    m = core.traffic._missions[robot]
    m.phase, m.path, m.idx, m.dest, m.goal = phase, list(path), 0, dest, None
    m.x, m.y = SITE.waypoint_pose(path[0])
    for zone in hold:
        core.zones.force_acquire(robot, zone)
    return m


def dist(a, b):
    return math.dist(SITE.waypoint_pose(a), SITE.waypoint_pose(b))


def test_a_free_path_costs_only_travel():
    core, _ = make({"pinky1": "E0"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2"], "OUTBOUND", hold=["E0"])
    eta = core.eta("pinky1")
    assert eta.travel_s == pytest.approx(0.70 / 0.2)
    assert eta.wait_s == 0.0
    assert eta.total_s == pytest.approx(3.5)


def test_an_unknown_robot_has_no_eta():
    core, _ = make({"pinky1": "E0"})
    assert core.eta("nobody") is None


def test_a_waypoint_that_is_not_on_the_map_or_not_reachable_has_no_eta(monkeypatch):
    core, _ = make({"pinky1": "E0"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1"], "OUTBOUND", hold=["E0"])
    assert core.eta("pinky1", "nowhere") is None

    def no_way(start, goal):
        raise LaneMapError("no path")

    monkeypatch.setattr(core.site, "plan", no_way)
    assert core.eta("pinky1", "O") is None


def test_the_eta_to_a_waypoint_on_the_path_stops_there():
    core, _ = make({"pinky1": "E0"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2", "O"], "OUTBOUND", hold=["E0"])
    assert core.eta("pinky1", "E1").total_s == pytest.approx(0.35 / 0.2)


def test_a_waypoint_past_the_path_end_is_reached_along_the_lanes():
    core, _ = make({"pinky1": "E2"})
    run(core, "pinky1", TO_PARK, ["E2"], hold=["E2"])
    assert core.eta("pinky1", "O").total_s == pytest.approx(dist("E2", "O") / 0.2)


def test_a_zone_held_by_a_runner_ahead_costs_its_release_time():
    core, _ = make({"pinky1": "E0", "pinky2": "E1"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2"], "OUTBOUND", hold=["E0"])
    # pinky2 holds E1 and leaves it at once: it only has to get clear of the zone
    run(core, "pinky2", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    eta = core.eta("pinky1")
    assert eta.wait_s == pytest.approx(EXIT_S)
    assert eta.total_s == pytest.approx(3.5 + EXIT_S)


def test_a_holder_that_is_gone_before_we_arrive_costs_nothing():
    core, _ = make({"pinky1": "E0", "pinky2": "E2"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2"], "OUTBOUND", hold=["E0", "E1"])
    run(core, "pinky2", TO_OUTBOUND, ["E2", "O"], "OUTBOUND", hold=["E2"])
    # pinky1 reaches the E2 boundary after 1.75 s and pinky2 is clear of E2 after 1 s
    assert core.eta("pinky1").wait_s == 0.0


def test_a_robot_working_at_a_station_costs_its_work_time_left():
    core, now = make({"pinky1": "E1", "pinky2": "O"})
    run(core, "pinky1", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    m = run(core, "pinky2", AT_STATION, ["O"], "OUTBOUND", hold=["O"])
    m.hold = True
    r = core.orchestrator._robots["pinky2"]
    r.stage, r.arm = SWAPPING, _ArmJob("arm3", 1, "swap", sent_at=0.0)
    now[0] = 3.0  # five of the eight seconds are left
    eta = core.eta("pinky1")
    run_to_o = (0.35 / 0.2) + dist("E2", "O") / 0.2
    assert eta.total_s == pytest.approx(5.0 + EXIT_S + dist("E2", "O") / 0.2)
    assert eta.travel_s == pytest.approx(run_to_o)


def test_robots_queued_ahead_each_add_their_service_time():
    core, _ = make({"pinky1": "E0", "pinky2": "O", "pinky3": "E2"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2", "O"], "OUTBOUND", hold=["E0"])
    m2 = run(core, "pinky2", AT_STATION, ["O"], "OUTBOUND", hold=["O"])
    m2.hold = True
    r2 = core.orchestrator._robots["pinky2"]
    r2.stage, r2.arm = SWAPPING, _ArmJob("arm3", 1, "swap", sent_at=0.0)
    run(core, "pinky3", TO_OUTBOUND, ["E2", "O"], "OUTBOUND", hold=["E2"])
    assert core.zones.request("pinky3", "O") is False  # queued behind pinky2
    free_at = (8.0 + EXIT_S) + (dist("E2", "O") / 0.2 + 3.0 + 8.0 + EXIT_S)
    assert core.eta("pinky1").total_s == pytest.approx(free_at + dist("E2", "O") / 0.2)


def test_a_holder_that_cannot_move_makes_the_eta_infinite():
    core, _ = make({"pinky1": "E0", "pinky2": "E1"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2"], "OUTBOUND", hold=["E0"])
    m2 = run(core, "pinky2", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    m2.fault = ERROR
    assert core.eta("pinky1").total_s == math.inf


def test_a_parked_holder_is_an_unknown_wait_of_the_detour_estimate():
    core, _ = make({"pinky1": "E0", "pinky2": "E1"}, station_wait_estimate_s=30.0)
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2"], "OUTBOUND", hold=["E0"])
    run(core, "pinky2", PARKED, ["E1"], hold=["E1"])
    assert core.eta("pinky1").wait_s == pytest.approx(30.0)


def test_a_holder_docking_costs_the_dock_and_a_default_fill_and_the_exit():
    core, _ = make({"pinky1": "F1", "pinky2": "W"})
    run(core, "pinky1", TO_WORK, ["F1", "F2", "W"], "WORK", hold=["F1"])
    run(core, "pinky2", DOCKING, ["W"], "WORK", hold=["W"])
    # not yet bound to an order: eta_fill_items_default items at 4 s each. pinky1 stands at the
    # W boundary (F2) after 1.75 s, so that much of the wait is spent driving
    assert core.eta("pinky1").wait_s == pytest.approx(3.0 + 2 * 4.0 + EXIT_S - 0.35 / 0.2)


def test_a_holder_still_on_its_way_to_the_dock_costs_its_arrival_and_service():
    core, _ = make({"pinky1": "E1", "pinky2": "E2"})
    run(core, "pinky1", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    # pinky2 is granted the dock zone but is still at E2: it arrives, docks, swaps and leaves
    run(core, "pinky2", TO_OUTBOUND, ["E2", "O"], "OUTBOUND", hold=["E2", "O"])
    free_at = dist("E2", "O") / 0.2 + 3.0 + 8.0 + EXIT_S
    assert core.eta("pinky1").total_s == pytest.approx(free_at + dist("E2", "O") / 0.2)


def test_the_station_time_left_of_a_filling_robot():
    core, now = make({"pinky1": "W"})
    m = run(core, "pinky1", AT_STATION, ["W"], "WORK", hold=["W"])
    m.hold = True
    r = core.orchestrator._robots["pinky1"]
    r.stage, r.order = FILLING, _BoundOrder("O-1", {"A": 1, "C": 1})
    r.arm = _ArmJob("workcell", 1, "fill", sent_at=0.0)
    now[0] = 2.0
    assert core.eta_station_left("pinky1") == pytest.approx(8.0 - 2.0 + EXIT_S)
    now[0] = 50.0  # overdue: assume it ends soon
    assert core.eta_station_left("pinky1") == pytest.approx(1.0 + EXIT_S)


def test_a_robot_not_at_a_station_has_no_station_time_left():
    core, _ = make({"pinky1": "E0"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1"], "OUTBOUND", hold=["E0"])
    assert core.eta_station_left("pinky1") is None
    assert core.eta_station_left("nobody") is None


def test_the_orchestrator_tells_the_job_it_is_running():
    core, _ = make({"pinky1": "W"})
    orch: OrderOrchestrator = core.orchestrator
    assert orch.station_job("pinky1") is None
    r = orch._robots["pinky1"]
    r.stage, r.order = FILLING, _BoundOrder("O-1", {"A": 2, "C": 1}, placed={"A": 1})
    assert orch.station_job("pinky1") == ("fill", 2, None)
    r.stage = SWAPPING
    assert orch.station_job("pinky1") == ("swap", 1, None)
    assert orch.station_job("nobody") is None


def test_a_robot_heading_for_the_same_dock_ahead_of_us_is_served_first():
    core, _ = make({"pinky1": "E0", "pinky2": "E2"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2", "O"], "OUTBOUND", hold=["E0"])
    # pinky2 reaches the dock first (1.73 s against 5.2 s); it holds nothing of the dock yet
    run(core, "pinky2", TO_OUTBOUND, ["E2", "O"], "OUTBOUND", hold=["E2"])
    free_at = dist("E2", "O") / 0.2 + 3.0 + 8.0 + EXIT_S
    assert core.eta("pinky1").total_s == pytest.approx(free_at + dist("E2", "O") / 0.2)


def test_a_holder_at_the_end_of_its_path_before_docking_costs_the_service():
    core, _ = make({"pinky1": "E1", "pinky2": "O"})
    run(core, "pinky1", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    # the robot reached the dock waypoint; traffic has not yet moved it on to DOCKING
    run(core, "pinky2", TO_OUTBOUND, ["O"], "OUTBOUND", hold=["O"])
    free_at = 3.0 + 8.0 + EXIT_S
    assert core.eta("pinky1").total_s == pytest.approx(free_at + dist("E2", "O") / 0.2)


def test_a_fill_has_a_set_up_time_that_defaults_to_nothing():
    assert PARAMS.eta_fill_base_s == 0.0
    core, now = make({"pinky1": "W"}, eta_fill_base_s=4.0)
    m = run(core, "pinky1", AT_STATION, ["W"], "WORK", hold=["W"])
    m.hold = True
    r = core.orchestrator._robots["pinky1"]
    r.stage, r.order = FILLING, _BoundOrder("O-1", {"A": 1, "C": 1})
    r.arm = _ArmJob("workcell", 1, "fill", sent_at=0.0)
    assert core.eta_station_left("pinky1") == pytest.approx(4.0 + 8.0 + EXIT_S)


def test_a_robot_part_way_along_its_last_lane_still_has_that_distance_to_go():
    # traffic advances idx once the robot is 0.2 m along a lane, so path[idx:] can be the end only
    core, _ = make({"pinky1": "E1"})
    m = run(core, "pinky1", TO_OUTBOUND, ["E1", "E2"], "OUTBOUND", hold=["E1", "E2"])
    m.idx = 1
    m.x, m.y = SITE.waypoint_pose("E1")[0] + 0.25, SITE.waypoint_pose("E1")[1]
    assert core.eta("pinky1").travel_s == pytest.approx(0.10 / 0.2)


def test_a_robot_beside_its_last_waypoint_still_has_the_sideways_distance_to_go():
    core, _ = make({"pinky1": "E1"})
    m = run(core, "pinky1", TO_OUTBOUND, ["E1", "E2"], "OUTBOUND", hold=["E1", "E2"])
    m.idx = 1
    m.x, m.y = SITE.waypoint_pose("E2")[0], SITE.waypoint_pose("E2")[1] - 0.10
    assert core.eta("pinky1").travel_s == pytest.approx(0.10 / 0.2)


def test_a_robot_queued_behind_another_is_not_stuck():
    core, _ = make({"pinky1": "E0", "pinky2": "E1"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2"], "OUTBOUND", hold=["E0"])
    m2 = run(core, "pinky2", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    m2.robot_state, m2.queue_since = "BLOCKED", 0.0  # a normal line behind a robot in front
    assert math.isfinite(core.eta("pinky1").total_s)
    m2.queue_since = None  # BLOCKED by an object: nothing says when it goes on
    assert core.eta("pinky1").total_s == math.inf


def test_robots_queued_for_a_conflicting_zone_are_served_first():
    core, _ = make({"pinky1": "F1", "pinky2": "F2", "pinky3": "T1"})
    run(core, "pinky1", TO_PARK, ["F1", "P-1"], hold=["F1"])
    run(core, "pinky3", TO_PARK, ["T1", "T2", "QO"], hold=["TUN"])
    run(core, "pinky2", TO_PARK, ["F2", "T1", "T2"], hold=["F2"])
    assert core.zones.request("pinky2", "TUN") is False  # TUN (conflicts with P-1) is held
    release = dist("T1", "T2") / 0.2 + EXIT_S
    hold = (dist("F2", "T1") + dist("T1", "T2")) / 0.2 + EXIT_S
    assert core.eta("pinky1").total_s == pytest.approx(release + hold + dist("F1", "P-1") / 0.2)


def test_a_delay_on_the_way_lets_a_later_robot_reach_the_dock_first():
    core, now = make({"pinky1": "E1", "pinky2": "T2", "pinky4": "E2"})
    run(core, "pinky1", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    run(core, "pinky2", TO_OUTBOUND, ["T2", "QO", "O"], "OUTBOUND", hold=["TUN"])
    m4 = run(core, "pinky4", AT_STATION, ["E2"], "WORK", hold=["E2"])
    m4.hold = True
    r4 = core.orchestrator._robots["pinky4"]
    r4.stage, r4.arm = SWAPPING, _ArmJob("arm3", 1, "swap", sent_at=0.0)
    now[0] = 3.0  # pinky4 frees E2 after 5 + 1 s: pinky1 reaches O at 7.75 s, pinky2 at 6.1 s
    reach2 = (dist("T2", "QO") + dist("QO", "O")) / 0.2
    free_at = reach2 + 3.0 + 8.0 + EXIT_S
    assert core.eta("pinky1").total_s == pytest.approx(free_at + dist("E2", "O") / 0.2)


def test_a_robot_at_its_station_has_arrived_and_leaves_after_its_work():
    core, now = make({"pinky2": "O"})
    m = run(core, "pinky2", AT_STATION, ["O"], "OUTBOUND", hold=["O"])
    m.hold = True
    r = core.orchestrator._robots["pinky2"]
    r.stage, r.arm = SWAPPING, _ArmJob("arm3", 1, "swap", sent_at=0.0)
    now[0] = 3.0
    assert core.eta("pinky2").total_s == 0.0
    # it starts driving when the work is done; clearing the dock zone is part of that drive
    assert core.eta("pinky2", "F0").total_s == pytest.approx(5.0 + dist("O", "F0") / 0.2)


def test_a_robot_heading_for_the_same_dock_behind_us_is_not_waited_for():
    core, _ = make({"pinky1": "E2", "pinky2": "E0"})
    run(core, "pinky1", TO_OUTBOUND, ["E2", "O"], "OUTBOUND", hold=["E2"])
    run(core, "pinky2", TO_OUTBOUND, ["E0", "E1", "E2", "O"], "OUTBOUND", hold=["E0"])
    assert core.eta("pinky1").wait_s == 0.0


def test_two_robots_ahead_are_served_one_after_the_other():
    core, _ = make({"pinky1": "E0", "pinky2": "E2", "pinky3": "E1"})
    run(core, "pinky1", TO_OUTBOUND, ["E0", "E1", "E2", "O"], "OUTBOUND", hold=["E0"])
    run(core, "pinky2", TO_OUTBOUND, ["E2", "O"], "OUTBOUND", hold=["E2"])
    run(core, "pinky3", TO_OUTBOUND, ["E1", "E2", "O"], "OUTBOUND", hold=["E1"])
    service = 3.0 + 8.0 + EXIT_S
    first = dist("E2", "O") / 0.2 + service  # pinky2 is done
    # pinky3 gets to the dock at 1.75 + 1.73 s, before pinky2 is done, so it starts at `first`
    second = max(first, 0.35 / 0.2 + dist("E2", "O") / 0.2) + service
    assert core.eta("pinky1").total_s == pytest.approx(second + dist("E2", "O") / 0.2)
