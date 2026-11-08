"""버전 2 1-A: 진짜로 물러나기(제안, 팀 합의 전, 계약 0.7-draft Retreat).

교착 순환에서 양보할 로봇이 쥔 구역 안에 모두 들어가 있으면 내놓을 구역이 없다(전에는
DEADLOCK_UNRESOLVED 뒤 격리). 이제 그 로봇이 들어온 레인의 시작 지점 구역을 바로 잠글 수 있고,
로봇이 그 레인 방향을 보고 있으며, 물러난 자리가 지금 구역 바닥에서 해제 거리만큼 떨어지면
그 구역을 잠그고 Retreat(곧게 후진)를 보낸다. 끝나면 경로 인덱스를 하나 되돌리고, 확인 시각에
벗어난 구역을 놓는다. 실패하면(뒤에 물체, 시간 초과, 거절) 전처럼 DEADLOCK_UNRESOLVED다.
"""
import math

from fleet_core.common.commands import CancelCmd, RetreatCmd
from fleet_core.traffic.mission import RETREAT
from test_free_area_entry import Drive

SOUTH = -math.pi / 2


def codes(out):
    return [c.code for c in out if type(c).__name__ == "EventOut"]


def at_the_gate():
    """r1이 F1에서 내려와 광장 입구 G3에 서 있다(진행 인덱스 1, 쥔 구역은 PLAZA뿐).

    r2와 교착 순환을 손으로 만든다(r1은 r2의 구역을, r2는 PLAZA를 기다린다).
    """
    d = Drive({"r1": "F1", "r2": "QO"})
    m = d.m("r1")
    m.path, m.idx, m.goal, m.settle_idx = ["F1", "G3", "G4", "P-1"], 1, None, None
    d.pose["r1"] = d.site.waypoint_pose("G3")
    for zone in list(d.core.zones.held_by("r1")):
        d.core.zones.release("r1", zone)
    assert d.core.zones.request("r1", "PLAZA")
    m.x, m.y, m.yaw = (*d.site.waypoint_pose("G3"), SOUTH)
    other = d.core._zone("QO")
    d.core.zones.request("r1", other)
    d.core.zones.request("r2", "PLAZA")
    return d, m


def retreat(d, m):
    d.core._retreat("r1", ["r1", "r2"])
    return d.core._out


def report(d, robot, x, y, yaw, n=3):
    for _ in range(n):
        d.core.on_robot_state(robot, x, y, yaw, "IDLE")


def test_a_robot_inside_every_zone_backs_up_its_lane_after_locking_the_spot_behind():
    d, m = at_the_gate()
    out = retreat(d, m)
    (cmd,) = [c for c in out if isinstance(c, RetreatCmd)]
    assert cmd.robot == "r1"
    assert cmd.distance_m == math.dist(d.site.waypoint_pose("F1"), d.site.waypoint_pose("G3"))
    assert cmd.speed_mps == d.params.retreat_speed_mps
    assert m.goal.kind == RETREAT and m.goal.id == cmd.goal_id
    assert "F1" in d.core.zones.held_by("r1"), "물러날 자리를 먼저 잠근다"
    assert "DEADLOCK_RETREAT" in codes(out)


def test_the_spot_behind_stays_locked_while_the_robot_backs_into_it():
    """SimWorld에서 드러남: 잠근 뒤 로봇이 아직 그 바닥 밖이라 '지난 구역 놓기'가 곧바로 놓았다."""
    d, m = at_the_gate()
    d.pose["r1"] = (m.x, m.y)
    retreat(d, m)
    d.core._out.clear()
    for _ in range(10):
        report(d, "r1", m.x, m.y, SOUTH, n=1)
        d.core.tick()
        d.now[0] = round(d.now[0] + 0.1, 6)
    assert "F1" in d.core.zones.held_by("r1")


def test_after_the_retreat_the_robot_gives_up_the_zone_it_backed_out_of():
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core.on_retreat_result("r1", cmd.goal_id, True, "")
    assert m.goal is None and m.idx == 0, "들어온 레인의 시작 지점으로 돌아갔다"
    report(d, "r1", *d.site.waypoint_pose("F1"), SOUTH)  # 결과 뒤의 보고로 벗어남을 센다
    d.now[0] += d.params.recover_check_s + 0.1
    d.core._finish_retreat(["r1", "r2"])
    assert "PLAZA" not in d.core.zones.held_by("r1")
    assert d.core.zones.held_by("r1") == ["F1"]


def test_one_noisy_report_after_a_good_retreat_does_not_end_it_unresolved():
    """SimWorld(gaussian_noise)에서 드러남: 물러난 뒤 확인 시각의 보고 하나가 잡음으로 바닥 안이면
    곧바로 DEADLOCK_UNRESOLVED였다. 성공한 물러나기는 확인을 몇 번 더 기다린다."""
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core.on_retreat_result("r1", cmd.goal_id, True, "")
    report(d, "r1", *d.site.waypoint_pose("G3"), SOUTH, n=1)  # 잡음: 아직 광장 바닥 안으로 보임
    d.now[0] += d.params.recover_check_s + 0.1
    d.core._out.clear()
    d.core._finish_retreat(["r1", "r2"])
    assert "DEADLOCK_UNRESOLVED" not in codes(d.core._out)
    report(d, "r1", *d.site.waypoint_pose("F1"), SOUTH)
    d.now[0] += d.params.recover_check_s + 0.1
    d.core._finish_retreat(["r1", "r2"])
    assert d.core.zones.held_by("r1") == ["F1"]
    assert d.core._retreating is None


def test_a_failed_retreat_keeps_the_path_and_ends_unresolved():
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core._out.clear()
    d.core.on_retreat_result("r1", cmd.goal_id, False, "OBSTACLE_BEHIND")
    assert "RETREAT_FAILED" in codes(d.core._out)
    assert m.goal is None and m.idx == 1
    d.now[0] += d.params.recover_check_s + 0.1
    report(d, "r1", *d.site.waypoint_pose("G3"), SOUTH)
    d.core._out.clear()
    d.core._finish_retreat(["r1", "r2"])
    assert "DEADLOCK_UNRESOLVED" in codes(d.core._out)
    assert "PLAZA" in d.core.zones.held_by("r1")


def test_a_rejected_retreat_is_a_failed_retreat():
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core._out.clear()
    d.core.on_goal_rejected("r1", cmd.goal_id)
    assert "RETREAT_FAILED" in codes(d.core._out)
    assert m.goal is None and m.idx == 1


def test_no_retreat_when_the_spot_behind_is_taken():
    d, m = at_the_gate()
    d.core.zones.request("r2", "F1")
    out = retreat(d, m)
    assert not [c for c in out if isinstance(c, RetreatCmd)]
    assert "DEADLOCK_UNRESOLVED" in codes(out)
    assert "r1" not in d.core.zones.queue("F1"), "잠글 수 없으면 줄에도 서지 않는다"


def test_no_retreat_when_the_robot_does_not_face_along_its_lane():
    d, m = at_the_gate()
    m.yaw = 0.0  # 동쪽을 본다. 곧게 후진하면 레인을 벗어난다
    out = retreat(d, m)
    assert not [c for c in out if isinstance(c, RetreatCmd)]
    assert "DEADLOCK_UNRESOLVED" in codes(out)
    assert "F1" not in d.core.zones.held_by("r1")


def test_no_retreat_at_the_start_of_the_path():
    d, m = at_the_gate()
    m.path, m.idx = ["G3", "G4", "P-1"], 0
    out = retreat(d, m)
    assert not [c for c in out if isinstance(c, RetreatCmd)]
    assert "DEADLOCK_UNRESOLVED" in codes(out)


def test_the_retreat_cancels_a_running_move_first():
    d, m = at_the_gate()
    from fleet_core.traffic.mission import MOVE, _Goal
    m.goal = _Goal(MOVE, 77, target_idx=1)
    out = retreat(d, m)
    kinds = [type(c).__name__ for c in out if isinstance(c, (CancelCmd, RetreatCmd))]
    assert kinds == ["CancelCmd", "RetreatCmd"]


# ---- 1-A Codex 검수 ---------------------------------------------------------------------


def test_a_retreat_with_no_result_by_its_deadline_is_canceled_and_waited_for():
    """결과가 기한까지 안 오면 취소를 한 번 보내고, 끝났다는 결과(또는 잃음)가 올 때까지 goal을
    열어 둔다. 그동안 뒤 구역도 앞 구역도 놓지 않는다(1-A 재검수 3)."""
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core._out.clear()
    for _ in range(3):
        d.now[0] = d.core._retreating[2] + 0.1
        report(d, "r1", *d.site.waypoint_pose("G3"), SOUTH)
        d.core._finish_retreat(["r1", "r2"])
    assert [c for c in d.core._out if isinstance(c, CancelCmd)] == [CancelCmd("r1", cmd.goal_id)]
    assert m.goal is not None and m.goal.kind == RETREAT
    assert set(d.core.zones.held_by("r1")) == {"PLAZA", "F1"}
    d.core.on_retreat_result("r1", cmd.goal_id, False, "CANCELED")
    assert "RETREAT_FAILED" in codes(d.core._out)
    assert m.goal is None and m.idx == 1


def test_a_canceled_retreat_whose_result_is_lost_ends_failed():
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.now[0] = d.core._retreating[2] + 0.1
    d.core._finish_retreat(["r1", "r2"])
    d.core._out.clear()
    d.core.on_goal_lost("r1", cmd.goal_id)
    assert "RETREAT_FAILED" in codes(d.core._out)
    assert m.goal is None


def test_the_deadline_still_holds_when_the_cycle_goes_away():
    """1-A 재검수 4: 물러나는 중에 순환이 사라져도(다른 로봇이 요청을 거둠) 기한에 취소한다."""
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core.zones.cancel_all_requests("r2")
    d.core._out.clear()
    d.now[0] = d.core._retreating[2] + 0.1
    d.core._check_deadlock(d.now[0])
    assert CancelCmd("r1", cmd.goal_id) in d.core._out


def test_the_fleet_deadline_leaves_room_for_the_robot_to_stop_first():
    """핑키는 달리던 이동이 끝나고 설 때까지(최대 3 s) BackUp을 미룬다."""
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    backup_allowance = cmd.distance_m / cmd.speed_mps * 2.0 + 2.0
    assert d.core._retreating[2] - d.now[0] >= backup_allowance + 3.0


def test_a_failed_retreat_off_the_floor_but_not_past_the_exit_still_ends():
    """실패한 물러나기 뒤 바닥 밖(뒤쪽)이나 출구 레인으로는 안 벗어난 로봇: 끝없이 안 기다린다."""
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.core.on_retreat_result("r1", cmd.goal_id, False, "OBSTACLE_BEHIND")
    x, y = d.site.waypoint_pose("F1")  # 실패라고 했지만 다 물러났다(경로 인덱스는 그대로)
    report(d, "r1", x, y, SOUTH)
    assert d.core._off_the_floor(m, "PLAZA") and not d.core._clear_of(m, "PLAZA")
    d.core._out.clear()
    for _ in range(10):
        if d.core._retreating is None:
            break
        d.now[0] = d.core._retreating[2] + 0.1
        report(d, "r1", x, y, SOUTH, n=1)
        d.core._finish_retreat(["r1", "r2"])
    assert d.core._retreating is None
    assert "DEADLOCK_UNRESOLVED" in codes(d.core._out)


def test_trying_a_robot_leaves_its_own_earlier_wait_for_the_spot_alone():
    d, m = at_the_gate()
    assert d.core.zones.request("r2", "F1")  # r2가 F1을 쥐고
    d.core.zones.request("r1", "F1")  # r1은 그 전부터 F1을 기다렸다
    retreat(d, m)
    assert "r1" in d.core.zones.queue("F1"), "시험해 본 것만으로 원래 줄을 잃지 않는다"


def test_no_retreat_that_would_leave_the_lane_floor():
    """30° 안이라도 곧게 물러난 끝이 들어온 레인 바닥(half_width) 밖이면 하지 않는다."""
    d, m = at_the_gate()
    m.yaw = SOUTH + math.radians(20)
    out = retreat(d, m)
    assert not [c for c in out if isinstance(c, RetreatCmd)]
    assert "F1" not in d.core.zones.held_by("r1")


# ---- 1-A 재검수 2 -------------------------------------------------------------------------


def retreat_by_the_cycle(d):
    """교착 감시가 순환을 보고 유예 뒤 r1을 물러나게 한다."""
    d.core._check_deadlock(d.now[0])
    d.now[0] += d.params.deadlock_grace_s
    d.core._check_deadlock(d.now[0])
    (cmd,) = [c for c in d.core._out if isinstance(c, RetreatCmd)]
    return cmd


def test_a_new_cycle_waits_until_the_open_retreat_ends():
    """1-A 재검수 2의 1: 물러나는 중에 순환이 바뀌어도 새 순환의 복구를 시작하지 않는다."""
    d, m = at_the_gate()
    cmd = retreat_by_the_cycle(d)
    d.core.zones.cancel_request("r2", "PLAZA")
    d.core._check_deadlock(d.now[0])  # 순환이 사라졌다
    d.core.zones.request("r2", "PLAZA")
    d.core._out.clear()
    for _ in range(3):  # 같은 로봇들의 순환이 새 일로 다시 생기고 유예가 지난다
        d.core._check_deadlock(d.now[0])
        d.now[0] += d.params.deadlock_grace_s
    assert not {"DEADLOCK_RETREAT", "DEADLOCK_UNRESOLVED"} & set(codes(d.core._out))
    assert m.goal is not None and m.goal.id == cmd.goal_id
    assert d.core._retreating[0] == "r1"


def test_an_old_cycles_retreat_does_not_mark_the_new_cycle_unresolved():
    """1-A 재검수 2의 5: 지난 순환의 물러나기 판정은 새 순환의 미해결 시계를 켜지 않는다."""
    d, m = at_the_gate()
    cmd = retreat_by_the_cycle(d)
    d.core.zones.cancel_request("r2", "PLAZA")
    d.core._check_deadlock(d.now[0])
    d.core.zones.request("r2", "PLAZA")
    d.core._check_deadlock(d.now[0])  # 물러나기가 열린 채 새 순환이 시작된다
    d.core.on_retreat_result("r1", cmd.goal_id, False, "OBSTACLE_BEHIND")
    d.core._out.clear()
    d.now[0] = d.core._retreating[2] + 0.1
    for _ in range(2):  # 처음 판은 새 순환을 알아보고, 다음 판이 지난 물러나기를 판정한다
        d.core._check_deadlock(d.now[0])
        d.now[0] += 0.1
    assert d.core._retreating is None
    assert d.core._unresolved_at is None
    assert "DEADLOCK_UNRESOLVED" not in codes(d.core._out)


def test_a_retreat_whose_result_is_lost_isolates_the_robot_with_its_zones():
    """1-A 재검수 2의 3: 결과를 잃은 물러나기는 로봇이 섰는지 모른다. 구역을 쥔 채 격리한다."""
    d, m = at_the_gate()
    (cmd,) = [c for c in retreat(d, m) if isinstance(c, RetreatCmd)]
    d.now[0] = d.core._retreating[2] + 0.1
    d.core._finish_retreat(["r1", "r2"])
    d.core.on_goal_lost("r1", cmd.goal_id)
    assert m.fault == "ERROR" and m.fault_reason == "RETREAT_LOST"
    assert {"PLAZA", "F1"} <= set(d.core.zones.held_by("r1"))
