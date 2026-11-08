"""자유 구역 F-1: 광장 안 경로는 로봇 planner가 낸다(버전 2 F절 3~5번).

광장과 출구 다음 구역을 쥐면 관제가 그 로봇에 입구에서 출구까지의 경로 계산을 청한다
(PlanCmd, 로봇의 compute_path_to_pose). 답이 오기 전에는 입구까지만 보낸다. 답은 바닥 안
(distance_to_zone == 0), 입구와 출구에 닿음, 직선의 free_path_max_ratio배 이하일 때만 쓰고,
아니면 입구와 출구를 잇는 직선으로 대신한다. 답이 free_plan_wait_s 안에 없어도 직선이다.
광장 안에서는 받은 경로를 기준으로 레인 이탈을 잰다.
"""
from fleet_core.traffic.mission import MOVE
from test_free_area_entry import Drive

CURVE = [(1.35, 0.85), (1.55, 0.75), (1.55, 0.60), (1.35, 0.50)]  # G3에서 G4로 동쪽으로 휜 길


def kinds(out, name):
    return [c for c in out if type(c).__name__ == name]


def codes(out):
    return [c.code for c in kinds(out, "EventOut")]


def entered():
    """r1이 F1에서 광장과 P-1을 쥐고 G3까지 가는 goal과 경로 요청을 받은 상태."""
    d = Drive({"r1": "F1"})
    out = d.tick(3)
    return d, out


def test_the_robot_is_asked_for_a_path_and_sent_only_to_the_gate_meanwhile():
    d, out = entered()
    plans = kinds(out, "PlanCmd")
    assert len(plans) == 1
    plan = plans[0]
    assert (plan.robot, plan.area) == ("r1", "PLAZA")
    assert (plan.start_x, plan.start_y) == d.site.waypoint_pose("G3")
    assert (plan.goal_x, plan.goal_y) == d.site.waypoint_pose("G4")
    moves = kinds(out, "MoveCmd")
    assert moves and moves[-1].target_id == "G3", "답 전에는 입구까지만"
    assert len(kinds(d.tick(5), "PlanCmd")) == 0, "기다리는 동안 다시 청하지 않는다"


def test_an_answer_inside_the_floor_is_followed():
    d, out = entered()
    plan = kinds(out, "PlanCmd")[0]
    d.core.on_plan_result("r1", plan.plan_id, CURVE)
    out = d.tick(1)
    assert "FREE_PATH" in codes(out)
    move = kinds(out, "MoveCmd")[-1]
    assert move.target_id == "P-1"
    points = [(round(x, 6), round(y, 6)) for x, y, _ in move.path]
    for p in CURVE[1:3]:
        assert p in points, "광장 안에서는 받은 경로의 점을 따른다"


def test_an_answer_leaving_the_floor_is_replaced_by_the_straight_line():
    d, out = entered()
    plan = kinds(out, "PlanCmd")[0]
    bad = [(1.35, 0.85), (2.30, 0.70), (1.35, 0.50)]  # 바닥(x <= 2.0) 밖으로 나간다
    d.core.on_plan_result("r1", plan.plan_id, bad)
    out = d.tick(1)
    assert "FREE_PATH_REJECTED" in codes(out)
    move = kinds(out, "MoveCmd")[-1]
    assert move.target_id == "P-1"
    xs = {round(x, 6) for x, _, _ in move.path[1:]}
    assert xs == {1.35}, "입구와 출구를 잇는 직선(x = 1.35)"


# 실패한 답(None), 직선 0.35 m의 2배를 넘는 길(0.55 + 0.33 + 0.55 m), 출구에 닿지 않는 길
LONG = [(1.35, 0.85), (1.90, 0.84), (1.90, 0.51), (1.35, 0.50)]
SHORT = [(1.35, 0.85), (1.40, 0.70)]


def test_a_failed_too_long_or_unfinished_answer_is_replaced_too():
    for answer in (None, LONG, SHORT):
        d, out = entered()
        plan = kinds(out, "PlanCmd")[0]
        d.core.on_plan_result("r1", plan.plan_id, answer)
        out = d.tick(1)
        assert "FREE_PATH_REJECTED" in codes(out), answer
        assert kinds(out, "MoveCmd")[-1].target_id == "P-1"


def test_no_answer_in_time_falls_back_to_the_straight_line():
    d, out = entered()
    wait = d.params.free_plan_wait_s
    out = d.tick(round(wait / 0.1) + 2)
    assert "FREE_PATH_REJECTED" in codes(out)
    assert kinds(out, "MoveCmd")[-1].target_id == "P-1"


def test_a_stale_answer_is_ignored():
    d, out = entered()
    plan = kinds(out, "PlanCmd")[0]
    d.core.on_plan_result("r1", plan.plan_id + 1000, CURVE)
    out = d.tick(1)
    assert "FREE_PATH" not in codes(out)
    assert not kinds(out, "MoveCmd") or kinds(out, "MoveCmd")[-1].target_id == "G3"


def test_lane_error_inside_the_area_is_measured_from_the_path():
    d, out = entered()
    plan = kinds(out, "PlanCmd")[0]
    d.core.on_plan_result("r1", plan.plan_id, CURVE)
    d.tick(1)
    m = d.m()
    assert m.goal.kind == MOVE
    d.pose["r1"] = (1.55, 0.68)  # 직선에서 0.2 m지만 받은 경로 위
    out = d.tick(5)
    assert "LANE_ERROR" not in codes(out)
    d.pose["r1"] = (1.76, 0.68)  # 받은 경로에서 0.21 m
    out = d.tick(5)
    assert "LANE_ERROR" in codes(out)
