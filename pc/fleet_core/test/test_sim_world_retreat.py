"""SimWorld가 RetreatCmd(버전 2 1-A)를 핑키처럼 실행한다: 보는 방향 그대로 뒤로 곧게."""
import math

from fleet_core.common.commands import RetreatCmd
from sim_world import SimWorld


def world_with(robots):
    w = SimWorld(orders=[], robots=robots, run_mode="PAUSE")  # 시험이 명령을 직접 넣는다
    w.step()
    return w


def results(w):
    seen = []
    w.core.traffic.on_retreat_result = lambda *a: seen.append(a)
    return seen


def test_a_retreat_backs_straight_up_and_reports_success():
    w = world_with({"r1": {"start": "P-1", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    seen = results(w)
    r = w.virtual["r1"]
    r.heading = math.pi / 2  # 북쪽을 본다
    x, y = r.x, r.y
    w._apply(RetreatCmd("r1", 901, 0.1, 0.1))
    w.run(2.0)
    assert seen == [("r1", 901, True, "")]
    assert (r.x, r.y) == (x, y - 0.1) or math.dist((r.x, r.y), (x, y - 0.1)) < 1e-9
    assert r.heading == math.pi / 2


def test_a_robot_behind_ends_a_retreat_with_obstacle_behind():
    w = world_with({"r1": {"start": "P-1", "speed_mps": 0.2, "pose_noise_m": 0.0},
                    "r2": {"start": "P-2", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    seen = results(w)
    r1, r2 = w.virtual["r1"], w.virtual["r2"]
    r1.heading = math.atan2(r1.y - r2.y, r1.x - r2.x)  # r2를 등진다
    start = (r1.x, r1.y)
    w._apply(RetreatCmd("r1", 902, 0.4, 0.1))
    w.run(6.0)
    assert seen and seen[0][:3] == ("r1", 902, False) and seen[0][3] == "OBSTACLE_BEHIND"
    assert (r1.x, r1.y) == start, "남은 길 전체를 보고 처음부터 서 있는다"


def test_a_goal_that_replaces_a_retreat_ends_it_canceled():
    w = world_with({"r1": {"start": "P-1", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    seen = results(w)
    w._apply(RetreatCmd("r1", 903, 0.3, 0.05))
    w.step()
    w._preempt(w.virtual["r1"])
    w.step()
    assert seen == [("r1", 903, False, "CANCELED")]  # 이동·도킹 goal도 같은 _preempt를 부른다


def test_the_report_carries_the_heading_as_yaw():
    w = world_with({"r1": {"start": "P-1", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    w.virtual["r1"].heading = 1.0
    w.step()
    assert w.core.traffic._missions["r1"].yaw == 1.0


def test_a_box_behind_ends_a_retreat_with_obstacle_behind():
    """1-A Codex 6: 시나리오 상자도 뒤 물체다."""
    w = world_with({"r1": {"start": "P-1", "speed_mps": 0.2, "pose_noise_m": 0.0}})
    seen = results(w)
    r = w.virtual["r1"]
    r.heading = math.pi / 2
    start = (r.x, r.y)
    w.boxes = ((r.x, r.y - 0.25, 0.06),)
    w._apply(RetreatCmd("r1", 904, 0.3, 0.1))
    w.run(4.0)
    assert seen and seen[0][:3] == ("r1", 904, False) and seen[0][3] == "OBSTACLE_BEHIND"
    assert (r.x, r.y) == start
