"""가짜 경로 계산(버전 2 F-1). 실물 핑키의 Nav2 planner_server(compute_path_to_pose) 대신이다.

ROS를 쓰지 않는다. FakePinky의 compute_path_to_pose 액션과 SimWorld가 같은 함수를 쓴다.
시작에서 목표까지 곧은 선이고, 상자(물체)가 그 선에서 `반 크기 + clearance` 안에 있으면 상자
옆의 한 점을 거쳐 꺾는다. 그 선도 상자에 가까우면(짧은 길) 상자 앞뒤 옆의 두 점을 거친다. 바닥 다각형을 주면 바닥 안쪽으로 꺾는다(양쪽이 다 밖이면 None).
점은 step_m 간격으로 촘촘히 낸다(Nav2 경로처럼).
"""
from __future__ import annotations

import math


def _inside(x, y, polygon) -> bool:
    inside = False
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _gap(px, py, a, b) -> tuple[float, float]:
    """점에서 선분 a-b까지 거리와 투영 비율 t(0~1)."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    denom = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((px - a[0]) * vx + (py - a[1]) * vy) / denom)) if denom else 0.0
    return math.hypot(px - (a[0] + t * vx), py - (a[1] + t * vy)), t


def _dense(points, step_m: float) -> list[tuple[float, float]]:
    out = [points[0]]
    for a, b in zip(points, points[1:], strict=False):
        n = max(1, math.ceil(math.dist(a, b) / step_m))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                for k in range(1, n + 1)]
    return out


def plan_path(start, goal, boxes=(), clearance_m: float = 0.12, polygon=None,
              step_m: float = 0.05):
    """(x, y) 점 목록, 못 찾으면 None. boxes는 (x, y, size_m)들."""
    start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
    length = math.dist(start, goal)
    if length == 0.0:
        return [start, goal]
    ux, uy = (goal[0] - start[0]) / length, (goal[1] - start[1]) / length
    hits = []
    for bx, by, size in boxes:
        reach = size / 2 + clearance_m
        gap, t = _gap(bx, by, start, goal)
        if gap < reach and 0.0 < t < 1.0:
            hits.append((t, bx, by, reach))
    if not hits:
        return _dense([start, goal], step_m)
    t, bx, by, reach = min(hits)
    for side in (1.0, -1.0):  # 왼쪽 먼저
        nx, ny = -uy * side * reach * 1.2, ux * side * reach * 1.2
        one = [(bx + nx, by + ny)]  # 상자 옆 한 점
        two = [(bx - ux * reach + nx, by - uy * reach + ny),  # 짧은 길: 상자 앞뒤 두 점
               (bx + ux * reach + nx, by + uy * reach + ny)]
        for vias in (one, two):
            if polygon is not None and not all(_inside(*v, polygon) for v in vias):
                continue
            corners = [start, *vias, goal]
            if all(_gap(ox, oy, a, b)[0] >= s / 2 + clearance_m for ox, oy, s in boxes
                   for a, b in zip(corners, corners[1:], strict=False)):
                return _dense(corners, step_m)
    return None
