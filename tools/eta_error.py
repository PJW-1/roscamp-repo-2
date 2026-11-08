#!/usr/bin/env python3
"""ETA error measurement (version 2 section 2): how far the formula ETA is from the real arrival.

    python3 tools/eta_error.py --out eta_error.csv
    python3 tools/eta_error.py --maps lane_map --scenarios basic_orders --seeds 1 2
    python3 tools/eta_error.py --set eta_turn_s_per_rad=0 eta_fill_base_s=4

Runs the baseline scenarios (tools/baseline.py: same maps, scenarios, seeds, fake clock, SimWorld,
no ROS graph). Every `--every` seconds each robot that is on a leg to a station (TO_WORK,
TO_OUTBOUND) or to its parking spot (TO_PARK) gets an ETA from FleetManager.eta (the end of its
path at that moment). The sample is resolved when the robot's phase becomes DOCKING (arrived at the
dock) or PARKED. A sample is dropped when the leg ends any other way (fault, new destination) and
counted as dropped. An infinite prediction (holder that cannot move) is counted as inf and left out
of the error figures.

error = predicted - actual remaining seconds (positive: the ETA was late). Shown per leg kind and
overall: n, mean error (bias), mean absolute error, and the 50/90/95th percentile and max of the
absolute error. `travel_only_mae` is the same samples predicted without the wait term, so the
gain of the wait model shows.

`--set name=value` overrides a FleetParams value for the ETA only (the run itself is unchanged).
SimWorld robots do not turn, so `--set eta_turn_s_per_rad=0` is the fair setting for these runs.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import baseline  # noqa: E402
from baseline import (  # noqa: E402
    DEFAULT_PLAN,
    DT,
    PATH_ONLY,
    SEEDS,
    Operator,
    build_world,
    load_scenario,
    map_name,
    parse_scenario,
    scenario_name,
)
from fleet_core.eta import EtaEstimator  # noqa: E402
from fleet_core.traffic.mission import (  # noqa: E402
    DOCKING,
    PARKED,
    TO_OUTBOUND,
    TO_PARK,
    TO_WORK,
)

LEGS = (TO_WORK, TO_OUTBOUND, TO_PARK)
ARRIVED = {TO_WORK: DOCKING, TO_OUTBOUND: DOCKING, TO_PARK: PARKED}
SAMPLE_COLUMNS = ["map", "scenario", "mode", "seed", "robot", "leg", "sampled_s", "predicted_s",
                  "travel_s", "wait_s", "actual_s", "error_s"]
SUMMARY_COLUMNS = ["leg", "samples", "dropped", "inf", "mean_error_s", "mae_s", "p50_abs_s",
                   "p90_abs_s", "p95_abs_s", "max_abs_s", "travel_only_mae_s"]


def percentile(values, q) -> float:
    """Linear interpolation between the closest ranks of the sorted values."""
    ordered = sorted(values)
    if not ordered:
        return math.nan
    pos = (len(ordered) - 1) * q
    low = int(pos)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


def summarize(samples, dropped=0, legs=LEGS) -> list[dict]:
    """One row per leg kind plus `all`. samples: dicts with leg, predicted_s, travel_s, actual_s
    (predicted_s may be inf)."""
    rows = []
    for leg in (*legs, "all"):
        chosen = [s for s in samples if leg in ("all", s["leg"])]
        finite = [s for s in chosen if math.isfinite(s["predicted_s"])]
        errors = [s["predicted_s"] - s["actual_s"] for s in finite]
        absolute = [abs(e) for e in errors]
        travel = [abs(s["travel_s"] - s["actual_s"]) for s in finite]
        rows.append({
            "leg": leg, "samples": len(finite), "inf": len(chosen) - len(finite),
            "dropped": dropped if leg == "all" else "",
            "mean_error_s": sum(errors) / len(errors) if errors else math.nan,
            "mae_s": sum(absolute) / len(absolute) if absolute else math.nan,
            "p50_abs_s": percentile(absolute, 0.5), "p90_abs_s": percentile(absolute, 0.9),
            "p95_abs_s": percentile(absolute, 0.95),
            "max_abs_s": max(absolute) if absolute else math.nan,
            "travel_only_mae_s": sum(travel) / len(travel) if travel else math.nan,
        })
    return rows


class Sampler:
    """Reads a world after each step: takes ETA samples and resolves them on arrival."""

    def __init__(self, world, estimator, every_s):
        self.world, self.estimator, self.every_s = world, estimator, every_s
        self.next_at = 0.0
        self.leg: dict[str, tuple] = {}  # robot -> (phase, dest) of the leg being sampled
        self.pending: dict[str, list] = {}
        self.resolved: list[dict] = []
        self.dropped = 0

    def observe(self) -> None:
        world = self.world
        sample_now = world.t >= self.next_at - 1e-9
        if sample_now:
            self.next_at = world.t + self.every_s
        for name, mission in world.core.traffic._missions.items():
            phase, key = mission.phase, (mission.phase, mission.dest)
            before = self.leg.get(name)
            if before is not None and key != before:
                self._close(name, before, phase)
            if phase in LEGS:
                self.leg[name] = key
                if sample_now and not world.virtual[name].removed:
                    eta = self.estimator.eta(name)
                    if eta is not None:
                        self.pending.setdefault(name, []).append(
                            (world.t, eta.total_s, eta.travel_s, eta.wait_s))
            else:
                self.leg.pop(name, None)

    def _close(self, name, before, now_phase) -> None:
        samples = self.pending.pop(name, [])
        if ARRIVED.get(before[0]) == now_phase:
            for t, predicted, travel, wait in samples:
                self.resolved.append({"leg": before[0], "robot": name, "sampled_s": t,
                                      "predicted_s": predicted, "travel_s": travel,
                                      "wait_s": wait, "actual_s": self.world.t - t})
        else:
            self.dropped += len(samples)


def run_one(map_, scenario, seed, mode, overrides, every_s, timeout_s=None):
    """(samples, dropped, error text, world or None) of one baseline run."""
    data = load_scenario(scenario)
    try:
        world = build_world(data, map_, seed, mode)
    except (ValueError, KeyError) as error:
        return [], 0, f"cannot build: {error!r}", None
    _, actions = baseline._faults(data)
    estimator = EtaEstimator(world.site, replace(world.params, **overrides),
                             world.core.traffic, world.core.orchestrator, lambda: world.t)
    sampler, operator = Sampler(world, estimator, every_s), Operator(world, actions)
    limit = timeout_s or parse_scenario(data, world.site).timeout_s
    finish, error = None, ""
    try:
        while world.t < limit - 1e-9:
            operator.act()
            world.step()
            sampler.observe()
            if finish is None and world.all_orders_final():
                finish = world.t
            on_floor = [n for n, r in world.virtual.items() if not r.removed]
            parked = not on_floor or world.all_parked(on_floor)
            if finish is not None and not operator.pending and parked:
                break
    except AssertionError as exc:
        error = f"invariant at {world.t:.1f} s: {exc}"
    return sampler.resolved, sampler.dropped, error, world


def _coerce(text):
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(text.lower(), text)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0],
                                     epilog=__doc__.split("\n", 1)[1],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--maps", nargs="+", default=list(DEFAULT_PLAN))
    parser.add_argument("--scenarios", nargs="+")
    parser.add_argument("--modes", nargs="+", choices=baseline.MODES, default=["path"])
    parser.add_argument("--seeds", nargs="+", type=int, default=list(SEEDS))
    parser.add_argument("--every", type=float, default=2.0, help="seconds between samples")
    parser.add_argument("--set", nargs="*", default=[], metavar="NAME=VALUE",
                        help="FleetParams values for the ETA only")
    parser.add_argument("--out", type=Path, default=Path("eta_error.csv"))
    args = parser.parse_args(argv)
    overrides = {k: _coerce(v) for k, v in (item.split("=", 1) for item in args.set)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    samples, dropped, started = [], 0, time.monotonic()
    for map_, scenario in baseline.plan(args.maps, args.scenarios):
        for mode in args.modes:
            if mode != "path" and scenario_name(scenario) in PATH_ONLY:
                continue
            for seed in args.seeds:
                got, lost, error, world = run_one(map_, scenario, seed, mode, overrides, args.every)
                dropped += lost
                where = {"map": map_name(map_), "scenario": scenario_name(scenario),
                         "mode": mode, "seed": seed}
                samples += [{**where, **s, "error_s": s["predicted_s"] - s["actual_s"]}
                            for s in got]
                print(f"{where['map']} {where['scenario']} {mode} seed {seed}: {len(got)} samples "
                      f"{error}", file=sys.stderr, flush=True)
    with args.out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SAMPLE_COLUMNS)
        writer.writeheader()
        writer.writerows(samples)
    rows = summarize(samples, dropped)
    print("| " + " | ".join(SUMMARY_COLUMNS) + " |\n" + "|---" * len(SUMMARY_COLUMNS) + "|")
    for row in rows:
        print("| " + " | ".join(f"{v:.2f}" if isinstance(v, float) else str(v)
                                for v in (row[c] for c in SUMMARY_COLUMNS)) + " |")
    print(f"{len(samples)} samples in {time.monotonic() - started:.0f} s wall; rows {args.out}; "
          f"overrides {overrides or 'none'}; DT {DT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
