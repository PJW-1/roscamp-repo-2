"""코스 맵(지점, 방향 레인, 잠금 단위, 스테이션, 대기 지점)을 YAML에서 읽는다.

로봇은 레인 방향대로 지점에서 지점으로 곧게 달린다. 실제 맵이 아직 정해지지 않아
좌표는 맵 파일에만 둔다. 새 맵은 새 데이터일 뿐이다.

잠금 단위는 한 로봇이 한 번에 쥐는 것이고, 나머지 코드에서는 구역이라 부른다.
지점은 정확히 한 단위에 속하고, 레인은 끝 지점의 단위에 속한다.
  - 한 지점에서 끝나는 레인들은 함께 쓸 수 없어 잠금을 같이 쓰고, 그 대기열이 합류 FIFO다.
  - 양방향 쌍은 묶으면 잠금 하나라서 두 로봇이 마주 달릴 수 없다.
  - 다른 단위인데 두 로봇이 지나기에 너무 가까운 레인은 conflict_pairs()가 찾는다.

지점은 레인 그래프의 꼭짓점이다. 노드라는 말은 ROS용으로 남겨 둔다.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_MAP = Path(__file__).parents[1] / "data" / "lane_map.yaml"


class LaneMapError(ValueError):
    """맵 파일이 맞지 않거나 요청한 경로가 없다."""


@dataclass(frozen=True)
class Station:
    id: str
    dock: object  # 지점
    queue: tuple  # 도크 앞에 줄 서는 (from, to) 레인
    commit: object  # 지점: 이 단위를 쥔 로봇은 더는 다른 곳으로 보낼 수 없다
    server: str  # 이 스테이션 액션의 서버
    arms: tuple[str, ...]  # 그 서버 뒤에서 일하는 팔
    action: str  # fill 또는 swap


@dataclass(frozen=True)
class FreeArea:
    """넓은 빈 바닥(버전 2 F절). 안에서는 레인 대신 로봇 planner가 만든 경로로 다닌다."""
    name: str  # 잠금 단위 이름이기도 하다
    polygon: tuple  # (x, y)들, 닫는 점은 쓰지 않는다
    gates: tuple  # 출입구 지점. 레인 그래프의 지점이다
    cell_m: float  # F-2에서 칸으로 나눌 크기


@dataclass(frozen=True)
class Lane:
    a: object  # 시작 지점
    b: object  # 끝 지점
    ax: float
    ay: float
    bx: float
    by: float
    ux: float  # a에서 b로의 단위 벡터
    uy: float
    length: float  # 경로 계획 비용, 파일에 없으면 직선 거리
    span: float  # 직선 거리


def _waypoint_id(raw):
    """지점 이름은 문자열, 또는 변환된 맵에서는 [col, row]."""
    if isinstance(raw, (list, tuple)):
        return tuple(int(v) for v in raw)
    return str(raw)


def _point_segment(x, y, ax, ay, bx, by) -> float:
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / denom)) if denom else 0.0
    return math.hypot(x - (ax + t * vx), y - (ay + t * vy))


def _inside_polygon(x, y, polygon) -> bool:
    inside = False
    for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _polygon_edges(polygon) -> list[tuple]:
    return [(a, b) for a, b in zip(polygon, polygon[1:] + polygon[:1], strict=True)]


def _box_edges(box) -> list[tuple]:
    x0, y0, x1, y1 = box
    return _polygon_edges(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def _in_box(x, y, box) -> bool:
    return box[0] <= x <= box[2] and box[1] <= y <= box[3]


def _box_segment(box, a, b) -> float:
    """닫힌 사각형과 선분 사이 거리. 끝점이 안이거나 변을 지나면 0."""
    if _in_box(*a, box) or _in_box(*b, box):
        return 0.0
    return min(_segment_distance(p, q, a, b) for p, q in _box_edges(box))


def _box_touches_polygon(box, polygon) -> bool:
    """닫힌 사각형과 닫힌 다각형이 한 점이라도 함께 갖나."""
    corners = [p for p, _ in _box_edges(box)]
    if any(_inside_polygon(x, y, polygon) for x, y in corners):
        return True
    if any(_in_box(x, y, box) for x, y in polygon):
        return True
    return any(_segment_distance(p, q, a, b) <= 1e-9
               for p, q in _box_edges(box) for a, b in _polygon_edges(polygon))


def _segment_distance(a1, a2, b1, b2) -> float:
    """두 선분 사이 최단 거리. 선분은 (x, y) 두 개."""
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    if (side(a1, a2, b1) * side(a1, a2, b2) < 0
            and side(b1, b2, a1) * side(b1, b2, a2) < 0):
        return 0.0
    return min(_point_segment(*a1, *b1, *b2), _point_segment(*a2, *b1, *b2),
               _point_segment(*b1, *a1, *a2), _point_segment(*b2, *a1, *a2))


class LaneMap:
    def __init__(self, data: dict, source_text: str = "", bundle_two_way: bool | None = None):
        self.source_text = source_text
        self.origin = tuple(float(v) for v in data["origin_m"])
        self.half_width = float(data["half_width_m"])
        self.lane_entry = float(data["lane_entry_m"])
        if "waypoints" not in data and "nodes" in data:
            raise LaneMapError("the map lists `nodes:`; since 2026-11-06 the key is `waypoints:`")
        self._pose = {_waypoint_id(n["id"]): (float(n["x"]), float(n["y"]))
                      for n in data["waypoints"]}
        if bundle_two_way is None:
            bundle_two_way = bool(data.get("bundle_two_way", True))
        self.bundle_two_way = bundle_two_way
        two_way = {name: (_waypoint_id(a), _waypoint_id(b))
                   for name, (a, b) in (data.get("two_way") or {}).items()}
        self.lanes = self._build_lanes(data.get("lanes") or [], two_way)
        self.free_areas = self._free_areas(data.get("free_areas") or {})
        self.free_lanes = frozenset(self._add_free_lanes())
        self._out: dict = {}
        self._in: dict = {}
        for lane in self.lanes.values():
            self._out.setdefault(lane.a, []).append(lane)
            self._in.setdefault(lane.b, []).append(lane)
        self.stations = {name: self._station(name, s)
                         for name, s in (data.get("stations") or {}).items()}
        self.parking = tuple(self._known(_waypoint_id(c), "parking")
                             for c in data.get("parking") or [])
        self.docks = {s.dock for s in self.stations.values()}
        units = dict(data.get("units") or {})
        bundled = {w for pair in two_way.values() for w in pair}
        for area in self.free_areas.values():
            if bundled & set(area.gates):
                raise LaneMapError(f"free area {area.name}: a gate cannot be in a two_way pair "
                                   f"{sorted(bundled & set(area.gates))}")
            if area.name in units:
                raise LaneMapError(f"free area {area.name} is also a unit")
            units[area.name] = list(area.gates)
        self.zones = self._build_zones(units, two_way, bundle_two_way)
        self.zone_of = {waypoint: name for name, members in self.zones.items()
                        for waypoint in members}
        self._segments = {zone: self._zone_segments(zone) for zone in self.zones}
        self._cell_cache: dict = {}  # 자유 구역 -> free_cells (F-2)
        self._units: dict | None = None  # 칸 잠금 단위 -> 자유 구역 (cell_units)
        self._approach_lanes: dict = {}  # 진입 레인 단위 -> 레인들
        self._temp: set = set()  # add_temp_zone으로 더한 구역
        for waypoint in self._pose:
            if waypoint not in self._out and waypoint not in self._in:
                raise LaneMapError(f"{waypoint} has no lane")

    def _known(self, waypoint, what: str):
        if waypoint not in self._pose:
            raise LaneMapError(f"{what}: {waypoint} is nowhere on the map")
        return waypoint

    def _build_lanes(self, raw: list[dict], two_way: dict) -> dict:
        lanes: dict = {}

        def add(a, b, length=None):
            self._known(a, "lane")
            self._known(b, "lane")
            if a == b:
                raise LaneMapError(f"lane from {a} to itself")
            if (a, b) in lanes:
                raise LaneMapError(f"lane from {a} to {b} is written twice")
            (ax, ay), (bx, by) = self._pose[a], self._pose[b]
            span = math.hypot(bx - ax, by - ay)
            if span <= 0.0:
                raise LaneMapError(f"lane from {a} to {b} has no length")
            lanes[(a, b)] = Lane(a, b, ax, ay, bx, by, (bx - ax) / span, (by - ay) / span,
                                 float(length) if length is not None else span, span)

        for lane in raw:
            add(_waypoint_id(lane["from"]), _waypoint_id(lane["to"]), lane.get("length"))
        for a, b in lanes:
            if (b, a) in lanes:
                raise LaneMapError(f"lanes {a} to {b} and back must be one two_way entry")
        for name, (a, b) in two_way.items():
            self._known(a, f"two_way {name}")
            self._known(b, f"two_way {name}")
            add(a, b)
            add(b, a)
        return lanes

    def _free_areas(self, raw: dict) -> dict:
        """free_areas: 이름 -> {polygon, gates, cell_m}. 출입구는 바닥 가장자리(half_width 안)."""
        areas = {}
        for name, entry in raw.items():
            polygon = tuple((float(x), float(y)) for x, y in entry.get("polygon") or [])
            if len(polygon) < 3:
                raise LaneMapError(f"free area {name}: the polygon needs three points or more")
            gates = tuple(self._known(_waypoint_id(g), f"free area {name}")
                          for g in entry.get("gates") or [])
            if len(set(gates)) < 2:
                raise LaneMapError(f"free area {name} needs two gates or more")
            cell_m = float(entry.get("cell_m", 0.35))
            if cell_m <= 0.0:
                raise LaneMapError(f"free area {name}: cell_m must be positive")
            for gate in gates:
                x, y = self._pose[gate]
                if self._polygon_gap(polygon, x, y) > self.half_width:
                    raise LaneMapError(f"free area {name}: gate {gate} is not on the edge")
            areas[name] = FreeArea(str(name), polygon, gates, cell_m)
        return areas

    def _add_free_lanes(self) -> list[tuple]:
        """출입구 쌍마다 양방향 가상 레인. 비용은 직선 거리, 실제 경로는 로봇 planner가 낸다."""
        added = []
        for area in self.free_areas.values():
            for a in area.gates:
                for b in area.gates:
                    if a == b or (a, b) in self.lanes:
                        continue
                    (ax, ay), (bx, by) = self._pose[a], self._pose[b]
                    span = math.hypot(bx - ax, by - ay)
                    if span <= 0.0:
                        raise LaneMapError(f"free area {area.name}: gates {a} and {b} coincide")
                    if self._outside(area.polygon, [(ax, ay), (bx, by)]):
                        # 경로 계산 답이 없을 때 이 직선으로 지난다(F-1). 오목한 바닥이면 막는다
                        raise LaneMapError(f"free area {area.name}: the straight line from {a} "
                                           f"to {b} leaves the floor")
                    self.lanes[(a, b)] = Lane(a, b, ax, ay, bx, by, (bx - ax) / span,
                                              (by - ay) / span, span, span)
                    added.append((a, b))
        return added

    @staticmethod
    def _polygon_gap(polygon, x, y) -> float:
        """다각형 바닥까지 거리. 안이면 0."""
        if _inside_polygon(x, y, polygon):
            return 0.0
        return min(_point_segment(x, y, *a, *b) for a, b in _polygon_edges(polygon))

    def free_area_of(self, key: tuple) -> str | None:
        """가상 레인 key가 지나는 자유 구역 이름. 보통 레인이면 None."""
        if key not in self.free_lanes:
            return None
        return self.zone_of[key[1]]

    def free_path_outside(self, area: str, points) -> bool:
        """출입구에서 출입구까지의 꺾은선이 다각형 바닥을 벗어나나(점 사이 선도 본다)."""
        return self._outside(self.free_areas[area].polygon, points)

    def _outside(self, polygon, points, step_m: float = 0.02) -> bool:
        """꺾은선을 step_m 간격으로 보아 바닥 밖이 있나. 양 끝 출입구는 가장자리에서
        half_width까지 떨어져 있을 수 있으므로 끝에서 2 x half_width 안은 half_width까지 봐준다.
        """
        ends = (points[0], points[-1])
        for a, b in list(zip(points, points[1:], strict=False)) or [(points[0], points[0])]:
            n = max(1, math.ceil(math.dist(a, b) / step_m))
            for k in range(n + 1):
                x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
                near_gate = min(math.dist((x, y), e) for e in ends) <= 2 * self.half_width
                if self._polygon_gap(polygon, x, y) > (self.half_width if near_gate else 1e-9):
                    return True
        return False

    def inside_free_floor(self, area: str, x: float, y: float) -> bool:
        return _inside_polygon(x, y, self.free_areas[area].polygon)

    def free_cells(self, area: str) -> dict:
        """F-2 칸: "구역/ix,iy" -> (x0, y0, x1, y1). 다각형을 감싸는 사각형의 왼쪽 아래에서
        cell_m 간격으로 자르고, 바닥에 닿는(경계 포함) 칸만 남긴다. 칸은 잘라내지 않는다.
        """
        free = self.free_areas[area]
        xs, ys = [x for x, _ in free.polygon], [y for _, y in free.polygon]
        x0, y0, c = min(xs), min(ys), free.cell_m
        def count(lo, hi):  # 마지막 칸이 바닥 끝을 덮을 때까지(반올림으로 모자라지 않게)
            n = max(1, math.ceil((hi - lo) / c))
            while lo + n * c < hi:
                n += 1
            return n

        cells = {}
        for ix in range(count(x0, max(xs))):
            for iy in range(count(y0, max(ys))):
                box = (x0 + ix * c, y0 + iy * c, x0 + (ix + 1) * c, y0 + (iy + 1) * c)
                if _box_touches_polygon(box, free.polygon):
                    cells[f"{area}/{ix},{iy}"] = box
        return cells

    def cells_near(self, area: str, points, clearance: float) -> frozenset:
        """꺾은선에서 clearance 안에 드는 칸 이름들(경계 포함, 반올림은 더 잡는 쪽)."""
        segments = list(zip(points, points[1:], strict=False)) or [(points[0], points[0])]
        reach = clearance + 1e-9
        return frozenset(name for name, box in self._cells_of(area).items()
                         if min(_box_segment(box, a, b) for a, b in segments) <= reach)

    def _cells_of(self, area: str) -> dict:
        if area not in self._cell_cache:
            self._cell_cache[area] = self.free_cells(area)
        return self._cell_cache[area]

    # ---- cell locks (버전 2 F-2) ----------------------------------------------------------------

    def cell_units(self) -> dict:
        """칸 잠금 단위 -> 자유 구역. 칸(구역/ix,iy)과 출입구마다 진입 레인(구역/>출입구).

        진입 레인은 바깥 지점에서 출입구로 들어오는 레인들이다. 구역 모드에서는 자유 구역
        바닥이었고(레인은 끝 지점의 단위) 칸이 덮지 않으므로 따로 잠근다.
        """
        if self._units is None:
            self._units, self._approach_lanes = {}, {}
            for area in self.free_areas.values():
                for name in self._cells_of(area.name):
                    self._units[name] = area.name
                for gate in area.gates:
                    lanes = [lane for lane in self._in.get(gate, ())
                             if self.zone_of[lane.a] != area.name]
                    if lanes:
                        name = f"{area.name}/>{self.waypoint_id(gate)}"
                        self._units[name] = area.name
                        self._approach_lanes[name] = lanes
        return dict(self._units)

    def unit_area(self, unit) -> str | None:
        """칸 잠금 단위의 자유 구역. 보통 구역이면 None."""
        return self.cell_units().get(unit) if isinstance(unit, str) else None

    def approach_unit(self, area: str, gate) -> str | None:
        name = f"{area}/>{self.waypoint_id(gate)}"
        return name if name in self.cell_units() else None

    def _unit_distance(self, x: float, y: float, unit: str) -> float:
        self.cell_units()
        if unit in self._approach_lanes:  # 구역 바닥처럼 lane_entry 뒤부터 half_width까지
            nearest = math.inf
            for lane in self._approach_lanes[unit]:
                skip = min(self.lane_entry, lane.span)
                nearest = min(nearest, _point_segment(
                    x, y, lane.ax + lane.ux * skip, lane.ay + lane.uy * skip, lane.bx, lane.by))
            return max(0.0, nearest - self.half_width)
        box = self._cells_of(self._units[unit])[unit]
        return math.hypot(max(box[0] - x, 0.0, x - box[2]), max(box[1] - y, 0.0, y - box[3]))

    def cell_conflict_pairs(self, clearance_m: float, release_m: float) -> set[frozenset]:
        """칸 잠금 단위가 들어간 충돌 쌍. conflict_pairs와 같은 규칙이다.

        - 칸과 바깥 구역 레인: 2r보다 가까우면. 출입구에서 나가는 레인은 앞 release_m을 뺀다.
        - 진입 레인과 바깥 구역 레인, 진입 레인끼리: 레인끼리의 규칙(_too_close).
        - 진입 레인과 칸: 레인이 칸에 2r보다 가까우면(출입구 칸과는 늘).
        - 다른 자유 구역의 칸끼리: 2r보다 가까우면.
        칸끼리는 쌍이 아니다. 떨어뜨리는 것은 경로에서 칸을 고르는 여유(controller)다.
        """
        units = self.cell_units()
        pairs: set[frozenset] = set()
        outside = [lane for lane in self.lanes.values() if (lane.a, lane.b) not in self.free_lanes]
        approaches = list(self._approach_lanes.items())
        for name, area_name in units.items():
            if name in self._approach_lanes:
                continue
            area, box = self.free_areas[area_name], self._cells_of(area_name)[name]
            for lane in outside:
                zone = self.zone_of[lane.b]
                if zone == area_name:
                    continue  # 진입 레인은 아래에서 단위끼리 본다
                skip = release_m if lane.a in area.gates else 0.0
                if skip < lane.span and _box_segment(
                        box, (lane.ax + lane.ux * skip, lane.ay + lane.uy * skip),
                        (lane.bx, lane.by)) < clearance_m:
                    pairs.add(frozenset({name, zone}))
            for unit, lanes in approaches:
                if any(_box_segment(box, (lane.ax, lane.ay), (lane.bx, lane.by)) < clearance_m
                       for lane in lanes):
                    pairs.add(frozenset({name, unit}))
        for i, (unit, lanes) in enumerate(approaches):
            for lane in outside:
                zone = self.zone_of[lane.b]
                if zone != units[unit] and any(
                        self._too_close(mine, lane, clearance_m, release_m) for mine in lanes):
                    pairs.add(frozenset({unit, zone}))
            for other, other_lanes in approaches[i + 1:]:
                if any(self._too_close(a, b, clearance_m, release_m)
                       for a in lanes for b in other_lanes):
                    pairs.add(frozenset({unit, other}))
        areas = list(self.free_areas)
        for i, one in enumerate(areas):
            for other in areas[i + 1:]:
                for a, box_a in self._cells_of(one).items():
                    for b, box_b in self._cells_of(other).items():
                        if min(_box_segment(box_a, p, q) for p, q in _box_edges(box_b)) \
                                < clearance_m:
                            pairs.add(frozenset({a, b}))
        return pairs

    def _station(self, name: str, raw: dict) -> Station:
        dock = self._known(_waypoint_id(raw["dock"]), f"station {name}")
        queue = tuple((_waypoint_id(a), _waypoint_id(b)) for a, b in raw.get("queue") or [])
        for lane in queue:
            if lane not in self.lanes:
                raise LaneMapError(f"station {name}: no lane from {lane[0]} to {lane[1]}")
        commit = self._known(_waypoint_id(raw.get("commit", raw["dock"])), f"station {name}")
        return Station(name, dock, queue, commit, raw["server"], tuple(raw["arms"]), raw["action"])

    def _build_zones(self, units: dict, two_way: dict, bundle: bool) -> dict:
        """지점마다 단위 하나: 적힌 대로, 아니면 묶음으로 합치고, 아니면 혼자."""
        name_of: dict = {}
        for name, raw in units.items():
            for member in raw:
                waypoint = self._known(_waypoint_id(member), f"unit {name}")
                if waypoint in name_of:
                    raise LaneMapError(f"{waypoint} is in more than one unit")
                name_of[waypoint] = name
        for waypoint in self._pose:
            name_of.setdefault(waypoint, self.waypoint_id(waypoint))
        if bundle:
            for name, (a, b) in two_way.items():
                old = {name_of[a], name_of[b]}
                for waypoint, unit in name_of.items():
                    if unit in old:
                        name_of[waypoint] = name
        zones: dict = {}
        for waypoint, name in name_of.items():
            zones.setdefault(name, set()).add(waypoint)
        return {name: frozenset(members) for name, members in zones.items()}

    def _zone_segments(self, zone: str) -> list[tuple]:
        """구역의 바닥: 지점들과, 로봇이 들어선 것으로 치는 점부터의 레인."""
        members = self.zones[zone]
        segments = [(*self._pose[n], *self._pose[n]) for n in members]
        for waypoint in members:
            for lane in self._in.get(waypoint, ()):
                skip = 0.0 if lane.a in members else self.lane_entry
                if skip < lane.span:
                    segments.append((lane.ax + lane.ux * skip, lane.ay + lane.uy * skip,
                                     lane.bx, lane.by))
        return segments

    # ---- geometry ---------------------------------------------------------------------------

    def lane(self, a, b) -> Lane:
        return self.lanes[(a, b)]

    def lanes_of_zone(self, zone: str) -> set:
        return {key for key in self.lanes if key[1] in self.zones[zone]}

    def lane_progress(self, a, b, x: float, y: float) -> tuple[float, float]:
        """s = (p - p_a)·u는 레인을 따라 간 거리, e = |(p - p_a) - s·u|는 벗어난 거리."""
        lane = self.lanes[(a, b)]
        rx, ry = x - lane.ax, y - lane.ay
        s = rx * lane.ux + ry * lane.uy
        return s, math.hypot(rx - s * lane.ux, ry - s * lane.uy)

    def waypoint_pose(self, waypoint) -> tuple[float, float]:
        return self._pose[waypoint]

    def bounds(self) -> tuple[float, float, float, float]:
        """모든 웨이포인트의 min_x, min_y, max_x, max_y."""
        xs = [x for x, _ in self._pose.values()]
        ys = [y for _, y in self._pose.values()]
        return min(xs), min(ys), max(xs), max(ys)

    def waypoint_id(self, waypoint) -> str:
        return f"c{waypoint[0]}_{waypoint[1]}" if isinstance(waypoint, tuple) else waypoint

    def nearest_waypoint(self, x: float, y: float, waypoints=None, radius: float | None = None):
        """자세가 속한 단위의 지점.

        `radius`(기본 half_width) 안이면 그 지점이다. 레인 위면 레인의 끝 지점이다.
        레인을 달리려면 그 지점의 단위가 필요하기 때문이다. 보고 하나는 오차가 있으니
        보고 하나로 판단할 때는 더 넓은 반경을 넘긴다. `waypoints`를 주면 그중 가장 가까운 것.
        """
        if waypoints is not None:
            return min(waypoints, key=lambda c: math.dist((x, y), self._pose[c]))
        waypoint = min(self._pose, key=lambda c: math.dist((x, y), self._pose[c]))
        limit = self.half_width if radius is None else radius
        if math.dist((x, y), self._pose[waypoint]) <= limit:
            return waypoint
        lane = min(self.lanes.values(),
                   key=lambda one: _point_segment(x, y, one.ax, one.ay, one.bx, one.by))
        return lane.b

    def distance_to_zone(self, x: float, y: float, zone: str) -> float:
        """자세에서 구역 바닥까지 거리. 안에 있으면 0. 자유 구역은 다각형 바닥도 바닥이다.

        칸 잠금 단위(F-2)면 칸 정사각형이나 진입 레인 바닥까지 거리.
        """
        if zone not in self._segments and self.unit_area(zone) is not None:
            return self._unit_distance(x, y, zone)
        nearest = min(_point_segment(x, y, *segment) for segment in self._segments[zone])
        distance = max(0.0, nearest - self.half_width)
        if zone in self.free_areas:
            distance = min(distance, self._polygon_gap(self.free_areas[zone].polygon, x, y))
        return distance

    def _floor_without(self, zone: str, key: tuple) -> list[tuple]:
        """구역 바닥에서 나가는 레인 key의 역방향(양방향 레인)을 뺀 선들."""
        members = self.zones[zone]
        segments = [(*self._pose[n], *self._pose[n]) for n in members]
        for waypoint in members:
            for lane in self._in.get(waypoint, ()):
                if (lane.a, lane.b) == (key[1], key[0]):
                    continue
                skip = 0.0 if lane.a in members else self.lane_entry
                if skip < lane.span:
                    segments.append((lane.ax + lane.ux * skip, lane.ay + lane.uy * skip,
                                     lane.bx, lane.by))
        return segments

    def floor_gap(self, zone: str, key: tuple, x: float, y: float) -> float:
        """나가는 레인 key 위의 자세에서 구역의 다른 바닥 선까지 중심선 거리.

        예각 θ로 합류하는 레인이 있으면 s만큼 가도 그 레인과는 s·sinθ만 떨어진다.
        """
        gap = min(_point_segment(x, y, *segment) for segment in self._floor_without(zone, key))
        if zone in self.free_areas:  # 중심선 기준이라 바닥 거리에 half_width를 더한다
            gap = min(gap, self._polygon_gap(self.free_areas[zone].polygon, x, y)
                      + self.half_width)
        return gap

    def exit_floor_gaps(self) -> list[tuple]:
        """구역을 떠나는 레인마다 (a, b, 끝 지점에서 구역의 다른 바닥 선까지 거리)."""
        out = []
        for (a, b), lane in self.lanes.items():
            if self.zone_of[a] != self.zone_of[b]:
                out.append((a, b, self.floor_gap(self.zone_of[a], (a, b), lane.bx, lane.by)))
        return out

    def shortest_exit_lane(self) -> float:
        """단위를 떠나는 가장 짧은 레인 길이. 해제 거리가 이 안에 들어가야 한다."""
        return min((lane.span for (a, b), lane in self.lanes.items()
                    if self.zone_of[a] != self.zone_of[b]), default=math.inf)

    # ---- temporary zones (버전 2 V2-0) ---------------------------------------------------------

    def add_temp_zone(self, name: str, points) -> None:
        """경로 선(점 둘 이상)을 바닥으로 하는 임시 구역. 지점은 없다. 1-C 비켜 가기용."""
        if name in self.zones:
            raise ValueError(f"zone {name} already exists")
        points = [tuple(float(v) for v in p) for p in points]
        if len(points) < 2:
            raise ValueError(f"temporary zone {name} needs two points or more")
        self.zones[name] = frozenset()
        self._segments[name] = [(*a, *b) for a, b in zip(points, points[1:], strict=False)]
        self._temp.add(name)

    def remove_temp_zone(self, name: str) -> None:
        if name not in self._temp:
            raise ValueError(f"zone {name} is not a temporary zone")
        del self.zones[name], self._segments[name]
        self._temp.discard(name)

    def temp_zone_conflicts(self, name: str, clearance_m: float) -> set[str]:
        """임시 구역의 바닥 선이 clearance_m(보통 2r)보다 가깝게 지나는 다른 구역들."""
        mine = self._segments[name]
        found = set()
        for zone, segments in self._segments.items():
            if zone == name:
                continue
            if any(_segment_distance(a[:2], a[2:], b[:2], b[2:]) < clearance_m
                   for a in mine for b in segments):
                found.add(zone)
        for area in self.free_areas.values():
            polygon = area.polygon
            if any(_inside_polygon(*a[:2], polygon) or _inside_polygon(*a[2:], polygon)
                   or any(_segment_distance(a[:2], a[2:], p, q) < clearance_m
                          for p, q in _polygon_edges(polygon)) for a in mine):
                found.add(area.name)
        return found

    @staticmethod
    def _samples(points, sample_m: float) -> tuple[list, float]:
        """경로를 sample_m 이하 간격 점으로 나눈다. (점들, 가장 큰 점 간격)."""
        points = [tuple(float(v) for v in p) for p in points]
        samples, gap = points[:1], 0.0
        for (ax, ay), (bx, by) in zip(points, points[1:], strict=False):
            steps = max(1, math.ceil(math.hypot(bx - ax, by - ay) / sample_m))
            gap = max(gap, math.hypot(bx - ax, by - ay) / steps)
            samples += [(ax + (bx - ax) * k / steps, ay + (by - ay) * k / steps)
                        for k in range(1, steps + 1)]
        return samples, gap

    def _touch_units(self, cells: bool) -> list:
        return [*self.zones, *(self.cell_units() if cells else ())]

    def _touch_gap(self, x: float, y: float, zone: str) -> float:
        """_off_the_floor와 같은 기준의 거리. 구역과 진입 레인은 바닥 거리 + half_width, 칸은
        그대로."""
        pad = 0.0 if self.unit_area(zone) is not None and ">" not in zone else self.half_width
        return self.distance_to_zone(x, y, zone) + pad

    def zones_touched(self, points, sample_m: float, release_m: float,
                      cells: bool = True) -> set[str]:
        """경로가 닿는 구역(버전 2 1-C): 제어기의 _off_the_floor가 아직 놓지 않을 거리 안의 구역.

        구역과 진입 레인은 바닥 거리 + half_width, 칸은 상자 거리 그대로 release_m과 비교한다.
        cells면 칸 잠금 단위(cell_units)도 함께 낸다(칸 모드가 꺼진 잠금에는 칸이 없다). 경로를
        sample_m 이하 간격 점으로 나누고, 점 사이를 지나는 부분을 놓치지 않도록 점 간격의 반만큼
        더 넓게 본다(더 잡는 쪽)."""
        samples, gap = self._samples(points, sample_m)
        reach = release_m + gap / 2
        return {zone for zone in self._touch_units(cells)
                if any(self._touch_gap(x, y, zone) < reach for x, y in samples)}

    def off_floor_points(self, points, sample_m: float, release_m: float,
                         cells: bool = True) -> list:
        """어느 구역에도 닿지 않는 경로 점들(버전 2 1-C). 잠그지 않은 바닥으로 나가는 곳이다.

        닿는다는 기준은 zones_touched와 같되 점 간격 여유를 더하지 않는다(덜 봐주는 쪽)."""
        samples, _ = self._samples(points, sample_m)
        units = self._touch_units(cells)
        return [(x, y) for x, y in samples
                if not any(self._touch_gap(x, y, zone) < release_m for zone in units)]

    # ---- conflict pairs -----------------------------------------------------------------------

    def conflict_pairs(self, clearance_m: float, release_m: float) -> set[frozenset]:
        """함께 쥐면 안 되는 단위 쌍: 레인이 2r보다 가깝게 지난다.

        지점에서 만나는 레인을 다 세면 아무도 앞 로봇을 따라갈 수 없다. 로봇은 다음 레인을
        release_m 달린 뒤에야 뒤 단위를 놓으므로, 공유 지점부터 그 구간은 빼고 나머지만 센다.
        양방향 쌍의 두 방향은 뺀다. 함께 쥐는 것은 묶음이 맡는다.
        """
        pairs: set[frozenset] = set()
        lanes = list(self.lanes.values())
        for i, one in enumerate(lanes):
            for other in lanes[i + 1:]:
                if (one.a, one.b) in self.free_lanes or (other.a, other.b) in self.free_lanes:
                    continue  # 가상 레인은 바닥 안이다. 바깥과의 거리는 아래 다각형 검사가 본다
                zone_a, zone_b = self.zone_of[one.b], self.zone_of[other.b]
                if zone_a != zone_b and self._too_close(one, other, clearance_m, release_m):
                    pairs.add(frozenset({zone_a, zone_b}))
        for area in self.free_areas.values():
            for lane in lanes:
                zone = self.zone_of[lane.b]
                if zone != area.name and self._near_area(lane, area, clearance_m, release_m):
                    pairs.add(frozenset({zone, area.name}))
        areas = list(self.free_areas.values())
        for i, one in enumerate(areas):
            for other in areas[i + 1:]:
                if self._polygons_near(one.polygon, other.polygon, clearance_m):
                    pairs.add(frozenset({one.name, other.name}))
        return pairs

    @staticmethod
    def _polygons_near(one, other, clearance: float) -> bool:
        if any(_inside_polygon(*p, other) for p in one) or any(
                _inside_polygon(*p, one) for p in other):
            return True
        return any(_segment_distance(a, b, p, q) < clearance
                   for a, b in _polygon_edges(one) for p, q in _polygon_edges(other))

    def _near_area(self, lane: Lane, area: FreeArea, clearance: float, release: float) -> bool:
        """레인이 자유 구역 바닥에 clearance보다 가깝다. 출입구에서 나가는 레인은 앞 release를 뺀다.

        출입구로 들어오는 레인은 그 출입구의 단위(= 자유 구역)에 속하므로 여기 오지 않는다.
        """
        skip = release if lane.a in area.gates else 0.0
        if skip >= lane.span:
            return False
        a = (lane.ax + lane.ux * skip, lane.ay + lane.uy * skip)
        b = (lane.bx, lane.by)
        if _inside_polygon(*a, area.polygon) or _inside_polygon(*b, area.polygon):
            return True
        return any(_segment_distance(a, b, p, q) < clearance
                   for p, q in _polygon_edges(area.polygon))

    def _too_close(self, one: Lane, other: Lane, clearance: float, release: float) -> bool:
        if frozenset({one.a, one.b}) == frozenset({other.a, other.b}):
            return False  # 양방향 쌍
        shared = {one.a, one.b} & {other.a, other.b}
        if not shared:
            return self._lane_distance(one, 0.0, other, 0.0) < clearance
        waypoint = next(iter(shared))
        if one.a == waypoint and other.a == waypoint:  # 갈림길: 어느 쪽이든 앞설 수 있다
            return (self._lane_distance(one, release, other, 0.0) < clearance
                    or self._lane_distance(one, 0.0, other, release) < clearance)
        if one.a == waypoint:  # other가 one으로 이어진다
            return self._lane_distance(one, release, other, 0.0) < clearance
        return self._lane_distance(one, 0.0, other, release) < clearance

    @staticmethod
    def _lane_distance(one: Lane, skip_one: float, other: Lane, skip_other: float) -> float:
        """두 레인 사이 거리. 각 레인의 앞 몇 미터는 뺀다."""
        if skip_one >= one.span or skip_other >= other.span:
            return math.inf
        return _segment_distance(
            (one.ax + one.ux * skip_one, one.ay + one.uy * skip_one), (one.bx, one.by),
            (other.ax + other.ux * skip_other, other.ay + other.uy * skip_other),
            (other.bx, other.by))

    # ---- planning -----------------------------------------------------------------------------

    def path_length(self, path: list) -> float:
        return sum(self.lanes[(a, b)].length for a, b in zip(path, path[1:], strict=False))

    def plan(self, start, goal) -> list:
        """레인을 따라가는 최단 경로(미터 기준). 양 끝 지점을 포함한다."""
        found = self.cheapest(start, goal, lambda lane: lane.length)
        if found is None:
            raise LaneMapError(f"no path along the lanes from {start} to {goal}")
        return found[1]

    def cheapest(self, start, goal, lane_cost) -> tuple[float, list] | None:
        """lane_cost(레인)의 합이 가장 작은 (비용, 경로). inf인 레인은 못 지난다. 없으면 None."""
        best = {start: 0.0}
        prev: dict = {start: None}
        seen = 0  # 힙에서 지점을 비교하지 않고 동점을 가른다
        heap = [(0.0, seen, start)]
        while heap:
            cost, _, waypoint = heapq.heappop(heap)
            if waypoint == goal:
                path = []
                while waypoint is not None:
                    path.append(waypoint)
                    waypoint = prev[waypoint]
                return cost, path[::-1]
            if cost > best[waypoint]:
                continue
            for lane in self._out.get(waypoint, ()):
                step = lane_cost(lane)
                if step == math.inf:
                    continue
                total = cost + step
                if total < best.get(lane.b, math.inf):
                    best[lane.b], prev[lane.b] = total, waypoint
                    seen += 1
                    heapq.heappush(heap, (total, seen, lane.b))
        return None


def load_lane_map(path=None, bundle_two_way: bool | None = None) -> LaneMap:
    source = Path(path) if path else DEFAULT_MAP
    text = source.read_text(encoding="utf-8")
    return LaneMap(yaml.safe_load(text), text, bundle_two_way)
