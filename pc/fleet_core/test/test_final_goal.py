"""버전 2 4절 horizon 최소판(제안, 팀 합의 전, 계약 0.8-draft MoveTo.final_goal).

관제는 이동 목표마다 그 끝이 임무의 마지막 지점인지 알린다. 잠금 경계에서 끝나는 목표는
final_goal이 거짓이라 로봇이 끝에서 정밀 접근 대신 설 수 있는 속도로만 줄인다.
"""
from fleet_core.common.commands import MoveCmd
from test_lane_path import make, moves, place, send


def test_a_move_cmd_is_final_unless_told_otherwise():
    assert MoveCmd("r1", 1, "A", 1.0, 2.0, 0.5).final_goal is True


def test_only_the_move_to_the_last_point_of_the_mission_is_final():
    core, now = make({"r1": "W"})
    send(core, "r1", "OUTBOUND")
    m = core._missions["r1"]
    seen = []
    for _ in range(40):
        out = core.tick()
        now[0] += 0.1
        for move in moves(out):
            seen.append((move.target_id, move.final_goal))
            place(core, "r1", m.path[m.goal.target_idx], state="IDLE")
            core.on_move_result("r1", move.goal_id, True, "")
        if seen and seen[-1][1]:
            break
    last = core.site.waypoint_id(m.path[-1])
    assert seen[-1] == (last, True)
    assert seen[0][1] is False, "잠금 경계에서 끝나는 첫 목표는 마지막이 아니다"
    assert all(final is (target == last) for target, final in seen)
