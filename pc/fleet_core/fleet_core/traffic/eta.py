"""ETA(도착 예상 시간, 버전 2 2번, 공식판): 남은 길 ÷ 속도 + 꺾임 시간 + 예상 대기.

거리는 레인 좌표로 직접 잰다(lane.length는 계획 비용이라 실제 거리가 아니다, lane.span이 직선
거리). 관측만 하고 아무것도 바꾸지 않는다. 배우는 ETA가 아니다.
"""
from __future__ import annotations

import math


def turn_s(params, angle: float) -> float:
    """제자리 회전 시간. eta_turn_min_rad보다 작은 꺾임은 달리면서 지나간다."""
    return angle * params.eta_turn_s_per_rad if angle >= params.eta_turn_min_rad else 0.0


def _angle(a: float, b: float) -> float:
    """두 방향(rad)의 차이, 0..pi."""
    return abs(math.remainder(a - b, math.tau))


def arrival_offsets(site, params, path, x: float, y: float, yaw: float) -> list[float]:
    """path[i]에 닿는 시각(지금부터 초). path[0]은 지금 구역의 지점이고 로봇은 (x, y)에 있다.

    path[i]에서 도는 시간은 그 지점을 떠나는 쪽에 들어간다. 대기는 넣지 않았다.
    """
    times = [0.0]
    pos, heading, t = (x, y), yaw, 0.0
    for a, b in zip(path, path[1:], strict=False):
        lane = site.lane(a, b)
        goal = site.waypoint_pose(b)
        t += (turn_s(params, _angle(heading, math.atan2(lane.uy, lane.ux)))
              + math.dist(pos, goal) / params.eta_speed_mps)
        times.append(t)
        pos, heading = goal, math.atan2(lane.uy, lane.ux)
    return times
