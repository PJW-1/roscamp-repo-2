"""플릿 매니저 정책 값. data/fleet_params.yaml에서 읽는다."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_PARAMS = Path(__file__).parents[1] / "data" / "fleet_params.yaml"


@dataclass(frozen=True)
class FleetParams:
    basket_capacity: int
    ship_partial: bool
    robot_radius_m: float
    release_margin_extra_m: float
    release_distance_m: float
    release_confirm_samples: int
    lane_error_warn_m: float
    move_retry_limit: int
    move_retry_wait_s: float
    arrival_tolerance_m: float
    arrival_settle_s: float
    startup_wait_s: float
    comm_timeout_s: float
    heartbeat_period_s: float
    heartbeat_timeout_s: float
    recover_check_s: float
    arm_cancel_wait_s: float
    arm_retry_wait_s: float
    fill_timeout_s: float
    swap_timeout_s: float
    cancel_retry_s: float
    cancel_warn_after: int
    report_retry_s: float
    zone_wait_warn_s: float
    deadlock_grace_s: float
    score_same_call: float
    score_distance_weight: float
    priority_at_station: int
    priority_has_cargo: int
    # 기본값: 이 값이 없던 기록도 재생된다.
    input_pose_margin_m: float = 1.0
    stall_s: float = 6.0
    stall_move_m: float = 0.05
    stall_smooth_s: float = 1.0
    dock_stall_s: float = 15.0
    blocked_mark_s: float = 5.0
    blocked_timeout_s: float = 60.0
    blocked_rejoin_s: float = 10.0
    queue_gap_m: float = 0.6
    # 우회(지시서 5번). 이 값이 없던 기록은 우회 없이 재생한다. 파일의 기본은 true다
    detour_enabled: bool = False
    station_wait_estimate_s: float = 30.0
    detour_speed_mps: float = 0.2
    off_lane_timeout_s: float = 20.0
    # path: 레인 폴리라인(FollowPath), point: 지점 하나(비교용). 이 값이 없던 기록은 point였다.
    lane_goal_mode: str = "point"
    # 해제 보강: 구역의 다른 바닥 선에서도 d 이상(예각 합류). 이 값이 없던 기록은 예전 규칙
    release_clear_of_floor: bool = False
    # H1: after DEADLOCK_UNRESOLVED, the same wait cycle lasting this long isolates one robot.
    deadlock_unresolved_s: float = 30.0
    # 조용히 서 있는 로봇 감시(1.1 지시서 H2). 0이면 끈다. 이 값이 없던 기록은 감시 없이 재생한다
    idle_watch_s: float = 0.0
    # Proposal (work order 1.1, H3): a failed outbound dispatch is sent again after the wait,
    # and failing more than the limit is an ERROR. Records without these values replay with them
    dispatch_retry_wait_s: float = 2.0
    dispatch_retry_limit: int = 3
    # Proposal (pinky self-recovery, 2026-11-09): a robot that keeps reporting RECOVERING longer
    # than this is taken as BLOCKED. The pinky FSM's two attempts take about 35 s at most.
    # Records from before RECOVERING never report it, so the value does not change their replay
    self_recovery_limit_s: float = 45.0
    # Proposal (version 2 F-1, free areas): how long to wait for the robot planner's path
    # before crossing on the straight gate-to-gate line, and how much longer than that line a
    # planned path may be. Maps without free areas never use them
    free_plan_wait_s: float = 2.0
    free_path_max_ratio: float = 2.0
    # Proposal (version 2 F-2): lock a free area by cells (cell_m) and gate approaches, taking
    # every cell within release_distance_m of the planned path at once. Off keeps one robot at
    # a time (F-1); records without it replay that way
    free_cells: bool = False
    # Proposal (version 2 1-A, contract 0.7-draft Retreat): a robot that must give way in a
    # deadlock but is inside every zone it holds backs up its lane after locking the zone
    # behind. Records without real_retreat replay without it (the old DEADLOCK_UNRESOLVED)
    real_retreat: bool = False
    retreat_max_m: float = 0.4
    retreat_speed_mps: float = 0.08
    retreat_heading_tolerance_deg: float = 30.0
    # Proposal (version 2 section 4, contract 0.8-draft MoveTo.final_goal): mark goals that end
    # at a lock boundary as not final. Records without it replay with every move final
    mark_final_goal: bool = False
    # Proposal (version 2 section 4, default choice 13): path-mode corners become arcs of at
    # most this radius, at most half_width inside the corner. 0 keeps the sharp corners;
    # records without it replay with sharp corners
    corner_radius_m: float = 0.0


def load_params(path=None) -> FleetParams:
    source = Path(path) if path else DEFAULT_PARAMS
    return FleetParams(**yaml.safe_load(source.read_text(encoding="utf-8")))
