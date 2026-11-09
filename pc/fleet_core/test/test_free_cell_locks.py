"""자유 구역 F-2: 여러 대, 칸 단위 잠금(버전 2 F절 F-2 1~3번).

free_cells를 켜면 자유 구역 하나 대신 칸과 진입 레인(test_free_cell_units.py)을 잠근다.
  - 들어갈 로봇은 먼저 광장 안 경로를 받는다(path 모드: 로봇 planner, 답이 없으면 직선).
    point 모드는 로봇이 알아서 지나므로 광장의 칸을 모두 잡는다.
  - 그 경로(앞뒤 레인 포함)에서 release_distance_m 안에 드는 칸, 들어오는 출입구의 진입 레인,
    출구 다음 구역을 모두 한꺼번에 청한다(request_all). 하나라도 안 되면 아무것도 잡지 않는다.
  - 지나간 칸은 지금의 해제 규칙(바닥에서 release_distance_m 넘게 떨어진 보고 N번)으로 놓는다.
    앞으로 갈 경로에서 release_distance_m 안에 드는 칸은 놓지 않는다.
자유 구역 단위 자체는 칸 모드에서 아무도 잡지 않는다.
"""
from dataclasses import replace

from fleet_core.common.commands import PlanCmd
from test_free_area_entry import Drive

G3_G4 = {"PLAZA/0,0", "PLAZA/1,0"}  # x = 1.35 선은 두 칸의 경계다


def cells_drive(starts, **changes):
    return Drive(starts, free_cells=True, **changes)


def plans(out):
    return [c for c in out if isinstance(c, PlanCmd)]


def answer_straight(d, out, robot="r1"):
    for plan in plans(out):
        if plan.robot == robot:
            d.core.on_plan_result(robot, plan.plan_id,
                                  [(plan.start_x, plan.start_y), (plan.goal_x, plan.goal_y)])


def test_the_path_comes_first_and_nothing_is_taken_while_waiting_for_it():
    d = cells_drive({"r1": "F1"})
    out = d.tick(3)
    assert len(plans(out)) == 1, "들어가기 전에 광장 안 경로를 청한다"
    assert d.core.zones.held_by("r1") == ["F1"]
    assert d.core.zones.requests_of("r1") == ()
    assert d.m().goal is None, "칸을 잡기 전에는 출입구로도 가지 않는다"


def test_the_cells_along_the_path_the_approach_and_the_exit_zone_are_taken_together():
    d = cells_drive({"r1": "F1"})
    out = d.tick(3)
    answer_straight(d, out)
    out = d.tick(1)
    held = set(d.core.zones.held_by("r1"))
    assert held == {"F1", "PLAZA/>G3", "P-1"} | G3_G4
    assert "PLAZA" not in held and d.core.zones.holder("PLAZA") is None
    moves = [c for c in out if type(c).__name__ == "MoveCmd"]
    assert moves and moves[-1].target_id == "P-1", "모두 쥐면 광장을 지나 출구 다음 지점까지 잇는다"


def test_nothing_is_taken_while_one_cell_of_the_path_is_held():
    d = cells_drive({"r1": "F1"})
    assert d.core.zones.request("ghost", "PLAZA/1,0")
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(3)
    assert d.core.zones.held_by("r1") == ["F1"], "전부 아니면 전무"
    assert set(d.core.zones.requests_of("r1")) == {"PLAZA/>G3", "P-1"} | G3_G4
    d.core.zones.release("ghost", "PLAZA/1,0")
    d.tick(2)
    assert set(d.core.zones.held_by("r1")) == {"F1", "PLAZA/>G3", "P-1"} | G3_G4


def test_two_robots_are_in_the_plaza_at_once_on_cells_apart():
    """G3->G4로 건너는 r1(칸 0, 1)과 T2에서 QO로 나가는 r2(칸 2).

    F0->T2 진입 레인은 칸 1 모서리에서 0.144 m(< 2r)라 칸 1과 충돌 쌍이다. 그래서 이 작은
    광장에서는 T2로 들어오는 로봇과 G3->G4 로봇이 함께 있을 수 없다(칸 단위라 보수적이다).
    """
    d = cells_drive({"r1": "F1", "r2": "T2"})
    assert frozenset({"PLAZA/>T2", "PLAZA/1,0"}) in d.core.site.cell_conflict_pairs(0.16, 0.2)
    m2 = d.m("r2")
    d.core._start_leg(m2, "TO_PARK", None, "QO")  # 대기 줄로 가면 광장을 가로지른다(T2->G4)
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(2)
    assert d.m("r2").path[:2] == ["T2", "QO"]
    assert G3_G4 <= set(d.core.zones.held_by("r1"))
    assert {"PLAZA/2,0", d.core._zone("QO")} <= set(d.core.zones.held_by("r2"))


def test_point_mode_takes_every_cell_of_the_area():
    d = cells_drive({"r1": "F1"}, lane_goal_mode="point")
    out = d.tick(3)
    assert not plans(out), "point 모드는 경로를 청하지 않는다"
    assert set(d.core.zones.held_by("r1")) == {
        "F1", "PLAZA/>G3", "P-1", "PLAZA/0,0", "PLAZA/1,0", "PLAZA/2,0"}


def test_passed_cells_are_let_go_after_the_robot_is_clear_of_them():
    d = cells_drive({"r1": "F1"})
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(1)
    d.pose["r1"] = (1.35, 0.6)  # 진입 레인 바닥(G3 위 0.05 m까지)에서 0.2 m: 중심선 기준 0.25 m
    d.tick(5)
    held = set(d.core.zones.held_by("r1"))
    assert "PLAZA/>G3" not in held, "지나간 진입 레인은 놓는다"
    assert G3_G4 <= held, "앞으로 갈 경로에 닿는 칸은 쥔다"
    d.pose["r1"] = d.site.waypoint_pose("P-1")
    d.tick(5)
    held = set(d.core.zones.held_by("r1"))
    assert not held & G3_G4, "광장을 지나 바닥에서 0.2 m 넘게 떨어지면 칸을 모두 놓는다"


def test_a_cell_is_kept_while_the_robot_is_still_near_it():
    d = cells_drive({"r1": "F1"})
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(1)
    d.pose["r1"] = (1.35, 0.42)  # G4를 지나 0.08 m: 칸 바닥에서 0.08 m
    d.tick(5)
    assert G3_G4 <= set(d.core.zones.held_by("r1"))


def test_the_area_does_not_count_as_needed_and_is_never_requested():
    d = cells_drive({"r1": "F1"})
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(1)
    assert "PLAZA" not in d.core._needed_zones(d.m())
    assert "PLAZA" not in d.core.zones.requests_of("r1")


def test_a_new_route_drops_the_cell_requests():
    d = cells_drive({"r1": "F1"})
    assert d.core.zones.request("ghost", "PLAZA/1,0")
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(2)
    m = d.m()
    assert d.core.zones.requests_of("r1")
    d.core._start_leg(m, m.phase, m.dest, "F2")
    assert d.core.zones.requests_of("r1") == ()


def test_a_robot_placed_on_a_gate_takes_the_cells_around_it():
    d = cells_drive({"r1": "T2"})
    held = set(d.core.zones.held_by("r1"))
    assert held == {"PLAZA/2,0"}, held
    assert d.m().placed


def test_the_one_at_a_time_rule_stays_without_free_cells():
    d = Drive({"r1": "F1"}, free_cells=False)
    d.tick(3)
    assert set(d.core.zones.held_by("r1")) == {"F1", "PLAZA", "P-1"}


def test_cells_are_off_in_the_dataclass_and_on_in_the_yaml():
    from fleet_core.common.params import FleetParams, load_params
    params = load_params()
    assert params.free_cells is True
    assert replace(params, free_cells=False).free_cells is False
    assert FleetParams.__dataclass_fields__["free_cells"].default is False


def test_a_cell_counts_as_left_only_at_the_release_distance_from_its_box():
    """칸에는 중심선이 없다. 레인처럼 half_width를 더하면 0.15 m에서 놓아 출구 레인의
    충돌 쌍(앞 release_distance_m을 뺐다)과 어긋난다(Codex F-2 지적 1)."""
    d = cells_drive({"r1": "F1"})
    m = d.m()
    m.x, m.y = 1.35, 0.34  # 칸 바닥(y = 0.5)에서 0.16 m
    assert not d.core._off_the_floor(m, "PLAZA/1,0")
    m.x, m.y = 1.35, 0.29  # 0.21 m
    assert d.core._off_the_floor(m, "PLAZA/1,0")


def test_the_claim_covers_the_body_of_a_robot_drifting_up_to_the_lane_error_limit():
    """몸(중심이 경로에서 lane_error_warn_m 안, 반지름 r)이 쥔 칸 안에 있어야 한다(지적 2)."""
    d = cells_drive({"r1": "F1"}, lane_error_warn_m=0.3)
    out = d.tick(3)
    answer_straight(d, out)
    d.tick(1)
    claim = d.m().claims["PLAZA"]
    reach = d.core.params.lane_error_warn_m + d.core.params.robot_radius_m
    assert set(d.core.site.cells_near("PLAZA", claim.line, reach)) <= claim.units


def test_the_rest_of_a_path_that_comes_back_near_its_start_is_not_cut_short():
    """가장 가까운 선분만 보면 되돌아오는 경로의 끝을 골라 갈 길을 버린다(지적 4).
    오차 한도 안의 가장 앞 선분을 고른다(남기는 쪽이 안전하다)."""
    from fleet_core.traffic.free_area import _remaining
    line = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.05, 0.02)]
    rest = _remaining(line, 0.05, 0.03, near=0.15)
    assert (2.0, 0.0) in rest and (2.0, 1.0) in rest
    assert _remaining(line, 2.0, 0.5, near=0.15)[1:] == [(2.0, 0.5), (2.0, 1.0), (0.05, 0.02)]


def test_a_detour_is_still_considered_before_asking_for_the_cells(monkeypatch):
    """칸 모드에서도 다음 구역이 자유 구역이면 우회를 먼저 본다(지적 5)."""
    d = cells_drive({"r1": "F1"})
    asked = []
    monkeypatch.setattr(d.core, "_consider_detour",
                        lambda m, zone: asked.append(zone) or False)
    d.tick(3)
    assert "PLAZA" in asked
