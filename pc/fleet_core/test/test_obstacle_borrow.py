"""1-C 옆 구역 빌리기(버전 2 지역 대응 5절, 계획서 S2~S4).

막힌 로봇(BLOCKED, 이동 goal이 열린 채)이 obstacle_wait_s 지나도 그대로면 관제가 그 로봇의
planner에 막힌 레인 끝 지점까지 경로를 청한다(PlanCmd, area ""). 답이 쥔 구역과 다른 구역의 바닥
안에만 있으면(lane_map.zones_touched) 닿는 구역 중 쥐지 않은 것을 한꺼번에 빌린다
(ZoneLocks.borrow_all: 모두 비고 기다리는 로봇도 없을 때만). 빌리면 그 경로를 DETOUR goal로 보낸다.
안 되면 borrow_wait_s 동안 매 tick 다시 보고, 빌리기가 대기 순환을 만들면 바로 거둔다. 시간이
지나면 거두고 지금의 막힘 처리(LANE_BLOCKED, 격리)로 간다.
"""
import sys
from dataclasses import replace
from pathlib import Path

from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import DETOUR
from fleet_core.traffic.zones import ZoneLocks
from sim_world import SimWorld

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "sim" / "fake_robots"))
from fake_robots.planner import plan_path  # noqa: E402

MAPS = REPO / "sim" / "fake_robots" / "maps"
AVENUE = MAPS / "two_lane_avenue_wide.yaml"
CROSSING = MAPS / "feature_map_crossing.yaml"
BLOCKED_AT = (1.70, 1.30)  # 동쪽행 AE->BE 위, 상자(1.95, 1.30) 앞
# 상자를 서쪽행 차선(y = 1.00) 쪽으로 비켜 BE로 돌아오는 길
AROUND = [BLOCKED_AT, (1.79, 1.11), (2.11, 1.11), (2.55, 1.30)]
NORTH = [BLOCKED_AT, (1.75, 1.57), (2.20, 1.57), (2.55, 1.30)]  # 레인 바닥 밖(북쪽)


def kinds(out, name):
    return [c for c in out if type(c).__name__ == name]


def codes(out):
    return [c.code for c in kinds(out, "EventOut")]


class Avenue:
    """r1이 AE에서 출고 도크로 가다가 AE->BE의 BLOCKED_AT에서 막힌다."""

    def __init__(self, **changes):
        self.now = [0.0]
        self.site = load_lane_map(AVENUE)
        self.params = replace(load_params(), lane_goal_mode="path", **changes)
        self.core = TrafficController(self.site, self.params, ["r1"], lambda: self.now[0])
        self.pose = self.site.waypoint_pose("AE")
        self.state = "IDLE"
        for _ in range(self.params.release_confirm_samples):
            self.report()
        self.core.on_run_mode("RUN")

    def report(self):
        self.core.on_robot_state("r1", *self.pose, 0.0, self.state)

    def tick(self, n=1):
        out = []
        for _ in range(n):
            self.report()
            out += self.core.tick()
            self.now[0] = round(self.now[0] + 0.1, 6)
        return out

    def m(self):
        return self.core._missions["r1"]

    def blocked(self):
        """출고 도크로 보내고 BLOCKED_AT까지 간 뒤 막힌다. 그동안의 출력을 돌려준다."""
        self.core.on_dispatch(1, "OUTBOUND", "r1")
        out = self.tick(2)
        self.pose, self.state = BLOCKED_AT, "MOVING"
        out += self.tick(5)
        assert self.m().path[self.m().idx] == "BE", self.m().path
        assert "AE" not in self.core.zones.held_by("r1")
        self.state = "BLOCKED"
        out += self.tick(round(self.params.obstacle_wait_s / 0.1) + 2)
        return out

    def plan(self, out):
        plans = kinds(out, "PlanCmd")
        assert len(plans) == 1, plans
        return plans[0]


# ---- 잠금과 지도 ------------------------------------------------------------------------------


def test_borrowing_ignores_robots_waiting_for_a_zone_the_borrower_already_holds():
    """빌릴 구역의 충돌 쌍을 이미 쥐고 있으면, 그 구역을 기다리는 로봇은 어차피 빌리는 로봇을
    기다린다. 그 대기자 때문에 빌리기를 막지 않는다."""
    z = ZoneLocks(["A", "B", "C"], lambda: 0.0, conflicts=[("B", "C")])
    assert z.request("r1", "C")
    assert not z.request("r2", "C")
    assert z.borrow_all("r1", ["B"])
    assert z.holder("B") == "r1"


def test_borrowing_still_yields_to_robots_waiting_for_the_borrowed_zone():
    z = ZoneLocks(["A", "B", "C"], lambda: 0.0, conflicts=[("B", "C")])
    assert z.request("r1", "C")
    assert z.request("r3", "A")
    assert not z.request("r2", "B")  # r2가 B를 기다린다(충돌 쌍 C를 r1이 쥠)
    assert not z.borrow_all("r1", ["B"])


def test_zones_touched_leaves_out_cells_when_cells_are_off():
    site = load_lane_map(CROSSING)
    pose = site.waypoint_pose
    line = [pose("GSW"), pose("GNW")]
    with_cells = site.zones_touched(line, 0.05, 0.2)
    assert any(site.unit_area(z) and z not in site.free_areas for z in with_cells)
    without = site.zones_touched(line, 0.05, 0.2, cells=False)
    assert not any(site.unit_area(z) and z not in site.free_areas for z in without)
    assert "PLAZA" in without


# ---- 제어기 ----------------------------------------------------------------------------------


def test_a_blocked_robot_asks_its_planner_for_a_way_round_after_the_wait():
    d = Avenue()
    out = d.blocked()
    plan = d.plan(out)
    assert plan.area == ""
    assert (plan.start_x, plan.start_y) == BLOCKED_AT
    assert (plan.goal_x, plan.goal_y) == d.site.waypoint_pose("BE")
    assert "LANE_BLOCKED" not in codes(out)


def test_a_free_neighbour_lane_is_borrowed_and_the_detour_sent():
    d = Avenue()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    out = d.tick(1)
    assert "DETOUR_PLANNED" in codes(out)
    assert "ZONE_BORROWED" in codes(out)
    m = d.m()
    assert m.goal.kind == DETOUR
    assert {"AW", "S"} <= m.borrowed
    assert {"AW", "S"} <= set(d.core.zones.held_by("r1"))
    move = kinds(out, "MoveCmd")[-1]
    assert move.target_id == "BE"
    points = [(round(x, 6), round(y, 6)) for x, y, _ in move.path]
    assert (1.79, 1.11) in points and (2.11, 1.11) in points
    assert m.block is None
    assert "LANE_BLOCKED" not in codes(d.tick(80)), "빌려서 가는 동안은 막힘 표시가 없다"


def held_aw():
    """ghost가 서쪽행 AW를 쥐고 있어 빌릴 수 없다. ghost는 아무도 안 기다린다."""
    d = Avenue()
    assert d.core.zones.force_acquire("ghost", "AW")
    return d


def test_the_borrow_waits_while_the_neighbour_lane_is_held_then_goes():
    d = held_aw()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    out = d.tick(20)
    assert "ZONE_BORROWED" not in codes(out)
    assert "AW" in d.core.zones.borrowing("r1")
    assert d.m().goal.kind != DETOUR
    assert "LANE_BLOCKED" not in codes(out), "빌리기를 기다리는 동안은 막힘 표시를 미룬다"
    d.core.zones.release("ghost", "AW")
    out = d.tick(1)
    assert "ZONE_BORROWED" in codes(out)
    assert d.m().goal.kind == DETOUR
    assert d.core.zones.borrowing("r1") == ()


def test_the_borrow_is_withdrawn_after_borrow_wait_s_and_the_lane_marked():
    d = held_aw()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    out = d.tick(round(d.params.borrow_wait_s / 0.1) + 5)
    assert "DETOUR_REJECTED" in codes(out)
    assert d.core.zones.borrowing("r1") == ()
    assert "LANE_BLOCKED" in codes(out), "그다음은 지금의 막힘 처리"
    assert "AW" not in d.core.zones.held_by("r1")


def test_a_borrow_that_would_close_a_wait_cycle_is_withdrawn_at_once():
    d = Avenue()
    plan = d.plan(d.blocked())
    # ghost는 AW를 쥐고 r1이 쥔 BE를 기다린다. r1이 AW를 빌리려고 ghost를 기다리면 순환이다
    assert d.core.zones.force_acquire("ghost", "AW")
    assert not d.core.zones.request("ghost", "BE")
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    out = d.tick(1)
    assert "DETOUR_REJECTED" in codes(out)
    assert d.core.zones.borrowing("r1") == ()


def test_a_path_off_the_lane_floors_is_rejected():
    d = Avenue()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, NORTH)
    out = d.tick(1)
    assert "ZONE_BORROWED" not in codes(out)
    second = d.plan(out)  # 다음 후보(R)
    assert (second.goal_x, second.goal_y) == d.site.waypoint_pose("R")
    d.core.on_plan_result("r1", second.plan_id, None)
    out = d.tick(1)
    rejected = [e for e in kinds(out, "EventOut") if e.code == "DETOUR_REJECTED"]
    assert rejected and "leaves the lane floors" in rejected[0].detail
    assert d.m().goal.kind != DETOUR


def test_a_straight_answer_along_the_lane_is_not_a_detour():
    """planner가 레인 그대로의 길을 내면 물체를 못 보는 것이다. 같은 길을 다시 보내지 않는다."""
    d = Avenue()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, [BLOCKED_AT, d.site.waypoint_pose("BE")])
    second = d.plan(d.tick(1))
    d.core.on_plan_result("r1", second.plan_id, None)
    out = d.tick(1)
    rejected = [e for e in kinds(out, "EventOut") if e.code == "DETOUR_REJECTED"]
    assert rejected and "does not leave the lanes" in rejected[0].detail
    assert d.m().goal.kind != DETOUR


def test_a_finished_detour_rejoins_the_path_and_returns_the_borrowed_zones():
    d = Avenue()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    d.tick(1)
    m = d.m()
    goal = m.goal
    d.pose, d.state = d.site.waypoint_pose("BE"), "IDLE"
    d.tick(1)
    d.core.on_move_result("r1", goal.id, True, "")
    out = d.tick(8)
    assert "DETOUR_DONE" in codes(out)
    assert m.borrowed == set()
    assert m.path[m.idx] == "BE"
    assert "ZONE_RETURNED" in codes(out)
    assert not {"AW", "S"} & set(d.core.zones.held_by("r1"))


def test_a_failed_detour_isolates_the_robot_with_its_zones():
    d = Avenue()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    d.tick(1)
    goal = d.m().goal
    d.pose, d.state = (1.79, 1.11), "IDLE"
    d.report()  # 길 밖까지 간 뒤 실패했다
    d.core.on_move_result("r1", goal.id, False, "NAV_ABORTED")
    out = d.tick(3)
    assert d.m().fault == "ERROR"
    assert "AW" in d.core.zones.held_by("r1"), "길 밖 로봇은 빌린 구역을 쥔 채 격리한다"
    assert "DETOUR_DONE" not in codes(out)


def test_a_detour_that_never_left_the_lane_goes_back_to_the_block_rules():
    """우회 goal이 출발 자리에서 그대로 멈췄으면 로봇은 아직 제 레인 위다. 격리하지 않고 이동
    실패 한 번으로 센 뒤, 지금·다음 구역이 아닌 빌린 구역은 해제 규칙에 넘긴다."""
    d = Avenue()
    plan = d.plan(d.blocked())
    d.core.on_plan_result("r1", plan.plan_id, AROUND)
    d.tick(1)
    assert d.m().goal.kind == DETOUR
    out = d.tick(round((d.params.stall_s + 2 * d.params.stall_smooth_s) / 0.1))
    failed = [e for e in kinds(out, "EventOut") if e.code == "DETOUR_FAILED"]
    assert failed and failed[0].level == "WARN" and "never left" in failed[0].detail
    assert not d.m().fault
    assert "ROBOT_ISOLATED" not in codes(out)
    assert d.m().borrowed == set()


def test_without_borrowing_the_robot_is_marked_blocked_as_before():
    d = Avenue(borrow_enabled=False)
    out = d.blocked()
    assert not kinds(out, "PlanCmd")
    out += d.tick(round(d.params.blocked_mark_s / 0.1))
    assert "LANE_BLOCKED" in codes(out)


# ---- SimWorld: 상자 하나가 동쪽행 차선에 ------------------------------------------------------


def avenue_world(**changes):
    params = replace(load_params(), lane_goal_mode="path", **changes)
    return SimWorld(seed=1, site=load_lane_map(AVENUE), params=params,
                    boxes=((1.75, 1.30, 0.08),), planner=plan_path)


def test_every_robot_borrows_its_way_round_the_box_and_ships():
    w = avenue_world()
    assert w.run_until(w.done, timeout_s=900)
    seen = w.event_codes()
    for code in ("DETOUR_PLANNED", "ZONE_BORROWED", "DETOUR_DONE", "ZONE_RETURNED"):
        assert code in seen, code
    assert "LANE_BLOCKED" not in seen
    assert not w.event_codes("ERROR")
    assert w.box_gap_min > 0.0
    assert w.outside_held == 0
