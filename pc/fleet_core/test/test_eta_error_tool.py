"""Check tools/eta_error.py without ROS: percentiles, the summary, and a smoke run."""
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import eta_error  # noqa: E402


def test_percentile_interpolates_between_ranks():
    assert eta_error.percentile([1, 2, 3, 4], 0.5) == pytest.approx(2.5)
    assert eta_error.percentile([5], 0.9) == 5
    assert eta_error.percentile([0, 10], 0.95) == pytest.approx(9.5)
    assert math.isnan(eta_error.percentile([], 0.5))


def sample(leg, predicted, actual, travel=None):
    return {"leg": leg, "predicted_s": predicted, "actual_s": actual,
            "travel_s": actual if travel is None else travel}


def test_the_summary_counts_errors_per_leg_and_overall():
    rows = {r["leg"]: r for r in eta_error.summarize([
        sample("TO_WORK", 12.0, 10.0, travel=6.0),
        sample("TO_WORK", 7.0, 10.0, travel=6.0),
        sample("TO_PARK", 5.0, 5.0),
        sample("TO_PARK", math.inf, 8.0),
    ], dropped=3)}
    work = rows["TO_WORK"]
    assert work["samples"] == 2 and work["mean_error_s"] == pytest.approx(-0.5)
    assert work["mae_s"] == pytest.approx(2.5)
    assert work["travel_only_mae_s"] == pytest.approx(4.0)
    assert rows["TO_PARK"]["samples"] == 1 and rows["TO_PARK"]["inf"] == 1
    assert rows["all"]["samples"] == 3 and rows["all"]["dropped"] == 3
    assert rows["all"]["max_abs_s"] == pytest.approx(3.0)


def test_a_smoke_run_resolves_samples_against_the_real_arrival():
    got, dropped, error, world = eta_error.run_one(
        "feature_map", "feature_basic_orders", 1, "path", {"eta_turn_s_per_rad": 0.0}, 2.0)
    assert error == ""
    assert world is not None and got
    assert all(s["actual_s"] > 0 for s in got)
    legs = {s["leg"] for s in got}
    assert {"TO_WORK", "TO_OUTBOUND"} <= legs
    finite = [s for s in got if math.isfinite(s["predicted_s"])]
    assert finite
    mae = sum(abs(s["predicted_s"] - s["actual_s"]) for s in finite) / len(finite)
    assert mae < 10.0  # a loose bound: the real figure is in the worklog
