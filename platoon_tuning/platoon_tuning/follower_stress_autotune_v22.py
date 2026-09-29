#!/usr/bin/env python3
"""
Adaptive V2.2 follower stress-course autotuner.

Designed for follower_id <- predecessor_id pairs:
  r2 <- r1
  r3 <- r2
  ...

The launch file is expected to accept:
  run_name, results_root, controller_params_file, planner_params_file,
  follower_id, predecessor_id, headless.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

try:
    import optuna
except ImportError as exc:
    raise SystemExit(
        "Optuna is required. Install it in the same Python environment used "
        "by ROS 2 before running this tuner."
    ) from exc

try:
    import yaml
except ImportError as exc:
    raise SystemExit(
        "PyYAML is required. Install python3-yaml / PyYAML before running."
    ) from exc


# ---------------------------------------------------------------------------
# Seed configuration.
# R2 defaults intentionally start from the frozen tuned R1 gains.
# Follower formation uses the V2.2 along-path bumper-gap definition.
# A later follower can override these with --seed-controller / --seed-planner.
# ---------------------------------------------------------------------------

BASE_CONTROLLER = {
    "heading_kp": 1388.5269897997275,
    "heading_ki": 26.45969942764789,
    "heading_kd": 201.59814882921654,

    "speed_kp": 77.49640833032895,
    "speed_ki": 16.564166376336193,
    "speed_kd": 49.19260599818982,

    "distance_kp": 0.35,
    "distance_ki": 0.02,
    "distance_kd": 0.10,

    "brake_kp": 177.8935198208073,
    "brake_ki": 60.634764417119634,
    "brake_kd": 10.647182804717701,

    "catchup_distance": 6.0,
    "catchup_min_speed": 1.20,
    "max_distance_speed_correction": 0.70,

    # Heading/guidance stage. These are controller parameters rather
    # than planner geometry; tuning them is necessary to reduce the
    # large outside-corner excursions seen in RViz.
    "lookahead_distance": 5.103256211030906,
    "follow_tangent_half_window": 0.75,
    "cross_track_heading_gain": 0.12,
    "max_cross_track_correction_deg": 35.0,
}

BASE_PLANNER = {
    "historical_preview_decel": 0.18,
    "historical_preview_max_speed": 1.50,
}


# ---------------------------------------------------------------------------
# Objective weights and adaptive acceptance thresholds.
#
# Formation gap, heading/guidance, speed and braking are tuned in that
# hierarchy. Once a stage satisfies its PASS threshold repeatedly, it is
# locked. Locked stages remain monitored; if an accepted later-stage
# controller drifts beyond an UNLOCK threshold, that earlier stage is
# immediately reopened and becomes the next stage to tune.
# ---------------------------------------------------------------------------

MAX_PENALTY = 1e6

# V2.2 formation metric: bumper-to-bumper distance ALONG the raw
# predecessor breadcrumb chain.
TARGET_GAP_M = 5.0

# Catch-up acquisition is not declared on a single crossing. The follower
# must be on breadcrumb guidance and remain inside the 5 m target band for
# a short hold. This prevents the temporary/negative path-gap values that
# occur while the breadcrumb trail is being established from falsely
# counting as "leader acquired".
ACQUISITION_BAND_M = 0.25
ACQUISITION_HOLD_S = 2.0
LOST_GAP_M = 5.75

# Exact oriented-hull geometry is SAFETY ONLY.
# A warning/avoidance event is penalized but does not fail a trial.
# Actual polygon contact (clearance approximately zero) is a hard failure.
COLLISION_CONTACT_EPS_M = 0.01

W_GAP_RMSE = 4.00
W_GAP_P95 = 2.00
W_GAP_BIAS = 1.50
W_GAP_MAX = 0.20

W_CTE_RMSE = 1.50
W_CTE_P95 = 0.45
W_CTE_MAX = 0.10
W_HEADING_RMSE_DEG = 0.050

W_SPEED_RMSE = 0.80
W_SPEED_P95 = 0.30

# Catch-up is deliberately important:
# acquire the leader quickly, then do not lose it again.
W_INITIAL_CATCHUP_TIME = 0.080
W_LOST_GAP_TIME = 0.40
W_LOST_GAP_EPISODE = 1.50

# Warnings are not a hard rejection because the course geometry can
# naturally bring hulls closer in corners. Avoidance is more serious
# than a warning, but still not equivalent to a collision.
W_COLLISION_WARNING_TIME = 0.25
W_AVOIDANCE_TIME = 1.00

W_TERMINAL_SPEED = 2.00
W_TERMINAL_GAP_ERROR = 0.50
W_FORMATION_SETTLE_TIME = 0.10
W_MISSION_TIME = 0.0015

LOCK_CONFIRM_RUNS = 1

PASS_THRESHOLDS = {
    "gap": {
        "path_gap_rmse_m": 0.25,
        "path_gap_p95_abs_error_m": 0.50,
        "abs_path_gap_bias_m": 0.10,
        "initial_catchup_time_s": 40.0,
        "lost_gap_time_s": 2.0,
        "lost_gap_episodes": 1,
    },
    "heading": {
        "cte_rmse_m": 0.90,
        "cte_p95_m": 2.50,
        "max_abs_cte_m": 4.00,
        "heading_rmse_deg": 2.50,
    },
    "speed": {
        "speed_rmse_mps": 0.20,
        "speed_p95_abs_error_mps": 0.35,
    },
    "brake": {
        "terminal_speed_mps": 0.10,
        "terminal_path_gap_error_m": 0.75,
        "formation_settle_time_s": 3.0,
    },
}

# A locked stage is reopened as soon as an ACCEPTED later-stage
# controller moves it back outside the same threshold that locked it.
# This implements "keep an eye on it; retune it next trial if it drifts".
UNLOCK_THRESHOLDS = PASS_THRESHOLDS

# ---------------------------------------------------------------------------
# Search bounds.
# ---------------------------------------------------------------------------

BOUNDS = {
    "heading_kp": (500.0, 1900.0),
    "heading_ki": (0.0, 60.0),
    "heading_kd": (40.0, 320.0),

    "speed_kp": (40.0, 240.0),
    "speed_ki": (0.0, 50.0),
    "speed_kd": (8.0, 140.0),

    "distance_kp": (0.05, 1.30),
    "distance_ki": (0.0, 0.18),
    "distance_kd": (0.0, 0.55),

    "brake_kp": (100.0, 380.0),
    "brake_ki": (0.0, 95.0),
    "brake_kd": (0.0, 45.0),

    "catchup_distance": (5.25, 6.50),
    "catchup_min_speed": (0.95, 1.50),
    "max_distance_speed_correction": (0.30, 1.00),

    "lookahead_distance": (2.0, 7.0),
    "follow_tangent_half_window": (0.25, 1.50),
    "cross_track_heading_gain": (0.05, 0.40),
    "max_cross_track_correction_deg": (20.0, 55.0),

    "historical_preview_decel": (0.08, 0.35),
}

STAGE_HIERARCHY = (
    "gap",
    "heading",
    "speed",
    "brake",
)


# ---------------------------------------------------------------------------
# Utilities.

# ---------------------------------------------------------------------------
# Utilities.
# ---------------------------------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {
        "1", "true", "yes", "y", "on",
    }


def f(row, key, default=float("nan")):
    try:
        return float(row.get(key, default))
    except Exception:
        return default


def finite(values):
    return [v for v in values if math.isfinite(v)]


def rmse(values):
    vals = finite(values)
    if not vals:
        return float("nan")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def percentile(values, fraction):
    vals = sorted(finite(values))
    if not vals:
        return float("nan")
    idx = int(round(fraction * (len(vals) - 1)))
    idx = max(0, min(len(vals) - 1, idx))
    return vals[idx]


def read_csv_rows(path: Path):
    if not path.exists() or path.stat().st_size < 40:
        return []
    try:
        with path.open(newline="") as fobj:
            return list(csv.DictReader(fobj))
    except Exception:
        return []


def append_jsonl(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as fobj:
        fobj.write(json.dumps(payload, sort_keys=True) + "\n")


def write_yaml(path: Path, node_name: str, values: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        node_name: {
            "ros__parameters": values,
        }
    }
    path.write_text(
        yaml.safe_dump(
            payload,
            sort_keys=True,
            default_flow_style=False,
        )
    )


def _extract_ros_parameters(data):
    if not isinstance(data, dict):
        return {}

    if "ros__parameters" in data:
        params = data.get("ros__parameters")
        return dict(params or {}) if isinstance(params, dict) else {}

    for value in data.values():
        if isinstance(value, dict):
            if "ros__parameters" in value:
                params = value.get("ros__parameters")
                return dict(params or {}) if isinstance(params, dict) else {}
            nested = _extract_ros_parameters(value)
            if nested:
                return nested

    return {}


def load_seed_yaml(path_text):
    if not path_text:
        return {}
    path = Path(path_text).expanduser()
    if not path.exists():
        raise SystemExit(f"Seed YAML does not exist: {path}")
    data = yaml.safe_load(path.read_text()) or {}
    return _extract_ros_parameters(data)


def locate_trial_csv(run_dir: Path, trial_name: str):
    exact = list(run_dir.rglob(f"{trial_name}.csv"))
    exact = [
        p for p in exact
        if p.name != "live_score.csv"
    ]
    if exact:
        return max(exact, key=lambda p: p.stat().st_mtime)

    candidates = [
        p for p in run_dir.rglob("*.csv")
        if p.name != "live_score.csv"
    ]
    if not candidates:
        return None

    return max(candidates, key=lambda p: p.stat().st_mtime)


def sample_dt(rows):
    times = finite([f(r, "time_s") for r in rows])
    if len(times) < 3:
        return 0.1
    diffs = [
        b - a
        for a, b in zip(times, times[1:])
        if math.isfinite(a)
        and math.isfinite(b)
        and 0.001 < (b - a) < 2.0
    ]
    if not diffs:
        return 0.1
    diffs.sort()
    return diffs[len(diffs) // 2]


def first_true_time(rows, key):
    times = [
        f(r, "time_s")
        for r in rows
        if parse_bool(r.get(key, False))
        and math.isfinite(f(r, "time_s"))
    ]
    return min(times) if times else float("nan")


def episode_count(values, target):
    count = 0
    previous = None
    for value in values:
        if value == target and previous != target:
            count += 1
        previous = value
    return count


# ---------------------------------------------------------------------------
# Metrics and scoring.
# ---------------------------------------------------------------------------

def _metric_value(metrics, key):
    value = metrics.get(key, float("nan"))
    try:
        return float(value)
    except Exception:
        return float("nan")


def stage_pass(stage, metrics):
    if not metrics:
        return False

    limits = PASS_THRESHOLDS[stage]

    for key, limit in limits.items():
        value = _metric_value(metrics, key)
        if not math.isfinite(value) or value > limit:
            return False

    return True


def stage_within_unlock(stage, metrics):
    if not metrics:
        return False

    limits = UNLOCK_THRESHOLDS[stage]

    for key, limit in limits.items():
        value = _metric_value(metrics, key)
        if not math.isfinite(value) or value > limit:
            return False

    return True


def next_unlocked_stage(locked):
    for stage in STAGE_HIERARCHY:
        if not locked.get(stage, False):
            return stage
    return None


def stage_objective(stage, metrics):
    """Small stage-local objective used when deciding whether to adopt
    a candidate that newly satisfies a threshold."""
    if stage == "gap":
        return (
            4.0 * metrics["path_gap_rmse_m"]
            + 2.0 * metrics["path_gap_p95_abs_error_m"]
            + 1.5 * metrics["abs_path_gap_bias_m"]
            + 0.05 * metrics["initial_catchup_time_s"]
            + 0.50 * metrics["lost_gap_time_s"]
            + 1.50 * metrics["lost_gap_episodes"]
        )

    if stage == "heading":
        return (
            1.5 * metrics["cte_rmse_m"]
            + 0.45 * metrics["cte_p95_m"]
            + 0.10 * metrics["max_abs_cte_m"]
            + 0.05 * metrics["heading_rmse_deg"]
        )

    if stage == "speed":
        return (
            0.8 * metrics["speed_rmse_mps"]
            + 0.3 * metrics["speed_p95_abs_error_mps"]
        )

    if stage == "brake":
        return (
            2.0 * metrics["terminal_speed_mps"]
            + 0.5 * metrics["terminal_path_gap_error_m"]
            + 0.1 * metrics["formation_settle_time_s"]
        )

    return MAX_PENALTY


def compute_metrics(rows):
    if not rows:
        return None

    dt = sample_dt(rows)

    # -------------------------------------------------------
    # Establish a real acquisition event first.
    #
    # The raw path-gap topic can be numerically valid before the
    # breadcrumb trail is ready. Those early values can even be
    # negative, so a simple "gap <= 5.25" test falsely declares the
    # leader acquired and then calls the genuine catch-up a loss.
    #
    # Acquisition therefore requires:
    #   1. active BREADCRUMB guidance,
    #   2. a finite V2.2 path gap,
    #   3. gap inside 5.00 +/- 0.25 m,
    #   4. continuous residence in that band for 2 s.
    # -------------------------------------------------------

    follow_gap_rows = [
        r for r in rows
        if r.get("mission_state") == "FOLLOW"
        and r.get("follower_path_source") == "BREADCRUMB"
        and parse_bool(r.get("path_gap_valid", False))
        and math.isfinite(f(r, "path_gap_m"))
        and not parse_bool(
            r.get("predecessor_target_capture", False)
        )
    ]

    if not follow_gap_rows:
        return None

    release_time = f(
        follow_gap_rows[0],
        "time_s",
        float("nan"),
    )

    band_low = TARGET_GAP_M - ACQUISITION_BAND_M
    band_high = TARGET_GAP_M + ACQUISITION_BAND_M

    acquisition_index = None
    band_start_index = None
    band_start_time = None

    for i, row in enumerate(follow_gap_rows):
        gap = f(row, "path_gap_m")
        t = f(row, "time_s")

        inside_band = (
            band_low <= gap <= band_high
            and math.isfinite(t)
        )

        if not inside_band:
            band_start_index = None
            band_start_time = None
            continue

        if band_start_index is None:
            band_start_index = i
            band_start_time = t

        if (
            band_start_time is not None
            and t - band_start_time
            >= ACQUISITION_HOLD_S
        ):
            # Metrics start when the target band has actually been
            # confirmed, not on the first accidental crossing.
            acquisition_index = i
            break

    if acquisition_index is None:
        return {
            "path_gap_rmse_m": float("nan"),
            "path_gap_p95_abs_error_m": float("nan"),
            "path_gap_bias_m": float("nan"),
            "abs_path_gap_bias_m": float("nan"),
            "maximum_abs_path_gap_error_m": float("nan"),
            "mean_path_gap_m": float("nan"),
            "minimum_hitbox_clearance_m": min(
                finite([
                    f(r, "hitbox_clearance_m")
                    for r in rows
                ])
                or [float("nan")]
            ),
            "collision_detected": False,
            "collision_warning_samples": sum(
                parse_bool(r.get("collision_warning", False))
                for r in rows
            ),
            "collision_warning_time_s": sum(
                parse_bool(r.get("collision_warning", False))
                for r in rows
            ) * dt,
            "avoidance_active_samples": sum(
                parse_bool(r.get("avoidance_active", False))
                for r in rows
            ),
            "avoidance_active_time_s": sum(
                parse_bool(r.get("avoidance_active", False))
                for r in rows
            ) * dt,
            "cte_rmse_m": float("nan"),
            "cte_p95_m": float("nan"),
            "max_abs_cte_m": float("nan"),
            "heading_rmse_deg": float("nan"),
            "speed_rmse_mps": float("nan"),
            "speed_p95_abs_error_mps": float("nan"),
            "initial_catchup_time_s": 99.0,
            "lost_gap_time_s": 99.0,
            "lost_gap_episodes": 99,
            "first_follow_time_s": 99.0,
            "catchup_time_s": 99.0,
            "catchup_episodes": 99,
            "predecessor_success_time_s": first_true_time(
                rows,
                "predecessor_success",
            ),
            "follower_success_time_s": first_true_time(
                rows,
                "follower_success",
            ),
            "terminal_speed_mps": 99.0,
            "terminal_path_gap_error_m": 99.0,
            "formation_settle_time_s": 99.0,
            "mission_time_s": max(
                finite([f(r, "time_s", 0.0) for r in rows])
                or [0.0]
            ),
            "following_samples": 0,
            "rows": len(rows),
            "sample_dt_s": dt,
        }

    acquisition_row = follow_gap_rows[acquisition_index]
    acquisition_time = f(
        acquisition_row,
        "time_s",
        99.0,
    )

    initial_catchup_time = (
        max(
            0.0,
            acquisition_time - release_time,
        )
        if math.isfinite(release_time)
        else 99.0
    )

    # Everything after confirmed acquisition is the steady formation
    # window. This is independent of the logger's tunable CATCHUP/
    # FOLLOWING classification.
    following = follow_gap_rows[
        acquisition_index:
    ]

    path_gap_errors = finite([
        f(r, "path_gap_m") - TARGET_GAP_M
        for r in following
    ])

    path_gaps = finite([
        f(r, "path_gap_m")
        for r in following
    ])

    hull_clearances = finite([
        f(r, "hitbox_clearance_m")
        for r in rows
    ])

    cte = finite([
        f(r, "follower_cross_track_error_m")
        for r in following
    ])

    heading_rad = finite([
        f(r, "follower_heading_error_rad")
        for r in following
    ])

    speed_errors = finite([
        f(r, "follower_speed_error_mps")
        for r in following
    ])

    lost_flags = [
        f(r, "path_gap_m") > LOST_GAP_M
        for r in following
    ]

    lost_gap_time = sum(lost_flags) * dt
    lost_gap_episodes = episode_count(
        lost_flags,
        True,
    )

    predecessor_success_time = first_true_time(
        rows,
        "predecessor_success",
    )

    follower_success_time = first_true_time(
        rows,
        "follower_success",
    )

    t_end = max(
        finite([f(r, "time_s", 0.0) for r in rows])
        or [0.0]
    )

    mission_time = (
        predecessor_success_time
        if math.isfinite(predecessor_success_time)
        else t_end
    )

    terminal_row = None

    if math.isfinite(predecessor_success_time):
        candidates = [
            r for r in rows
            if f(r, "time_s", -1.0)
            <= predecessor_success_time + 1e-6
        ]
        if candidates:
            terminal_row = candidates[-1]

    terminal_speed = (
        abs(f(terminal_row, "follower_speed_mps"))
        if terminal_row is not None
        else 99.0
    )

    terminal_path_gap_error = (
        abs(
            f(terminal_row, "path_gap_m")
            - TARGET_GAP_M
        )
        if (
            terminal_row is not None
            and math.isfinite(f(terminal_row, "path_gap_m"))
        )
        else 99.0
    )

    target_capture_time = first_true_time(
        rows,
        "predecessor_target_capture",
    )

    hold_times = [
        f(r, "time_s")
        for r in rows
        if r.get("follower_longitudinal_state") == "HOLD"
        and math.isfinite(f(r, "time_s"))
        and (
            not math.isfinite(target_capture_time)
            or f(r, "time_s") >= target_capture_time
        )
    ]

    if math.isfinite(target_capture_time) and hold_times:
        formation_settle_time = max(
            0.0,
            min(hold_times) - target_capture_time,
        )
    else:
        formation_settle_time = 99.0

    collision_warning_samples = sum(
        parse_bool(r.get("collision_warning", False))
        for r in rows
    )

    avoidance_active_samples = sum(
        parse_bool(r.get("avoidance_active", False))
        for r in rows
    )

    warning_time = collision_warning_samples * dt
    avoidance_time = avoidance_active_samples * dt

    minimum_hull_clearance = (
        min(hull_clearances)
        if hull_clearances
        else float("nan")
    )

    collision_detected = (
        math.isfinite(minimum_hull_clearance)
        and minimum_hull_clearance
        <= COLLISION_CONTACT_EPS_M
    )

    mean_gap_error = (
        sum(path_gap_errors) / len(path_gap_errors)
        if path_gap_errors
        else float("nan")
    )

    return {
        "path_gap_rmse_m":
            rmse(path_gap_errors),

        "path_gap_p95_abs_error_m":
            percentile(
                [abs(v) for v in path_gap_errors],
                0.95,
            ),

        "path_gap_bias_m":
            mean_gap_error,

        "abs_path_gap_bias_m":
            abs(mean_gap_error)
            if math.isfinite(mean_gap_error)
            else float("nan"),

        "maximum_abs_path_gap_error_m":
            max([abs(v) for v in path_gap_errors])
            if path_gap_errors
            else float("nan"),

        "mean_path_gap_m":
            (sum(path_gaps) / len(path_gaps))
            if path_gaps
            else float("nan"),

        "minimum_hitbox_clearance_m":
            minimum_hull_clearance,

        "collision_detected":
            collision_detected,

        "collision_warning_samples":
            collision_warning_samples,

        "collision_warning_time_s":
            warning_time,

        "avoidance_active_samples":
            avoidance_active_samples,

        "avoidance_active_time_s":
            avoidance_time,

        "cte_rmse_m":
            rmse(cte),

        "cte_p95_m":
            percentile(
                [abs(v) for v in cte],
                0.95,
            ),

        "max_abs_cte_m":
            max([abs(v) for v in cte])
            if cte
            else float("nan"),

        "heading_rmse_deg":
            math.degrees(rmse(heading_rad))
            if heading_rad
            else float("nan"),

        "speed_rmse_mps":
            rmse(speed_errors),

        "speed_p95_abs_error_mps":
            percentile(
                [abs(v) for v in speed_errors],
                0.95,
            ),

        "initial_catchup_time_s":
            initial_catchup_time,

        "lost_gap_time_s":
            lost_gap_time,

        "lost_gap_episodes":
            lost_gap_episodes,

        # Backward-readable aliases for old reports.
        "first_follow_time_s":
            initial_catchup_time,

        "catchup_time_s":
            lost_gap_time,

        "catchup_episodes":
            lost_gap_episodes,

        "predecessor_success_time_s":
            predecessor_success_time,

        "follower_success_time_s":
            follower_success_time,

        "terminal_speed_mps":
            terminal_speed,

        "terminal_path_gap_error_m":
            terminal_path_gap_error,

        "formation_settle_time_s":
            formation_settle_time,

        "mission_time_s":
            mission_time,

        "following_samples":
            len(following),

        "rows":
            len(rows),

        "sample_dt_s":
            dt,
    }


def core_score(metrics):
    required = [
        "path_gap_rmse_m",
        "path_gap_p95_abs_error_m",
        "path_gap_bias_m",
        "maximum_abs_path_gap_error_m",
        "cte_rmse_m",
        "cte_p95_m",
        "max_abs_cte_m",
        "heading_rmse_deg",
        "speed_rmse_mps",
        "speed_p95_abs_error_mps",
    ]

    if any(
        not math.isfinite(metrics.get(key, float("nan")))
        for key in required
    ):
        return MAX_PENALTY

    return (
        W_GAP_RMSE
        * metrics["path_gap_rmse_m"]

        + W_GAP_P95
        * metrics["path_gap_p95_abs_error_m"]

        + W_GAP_BIAS
        * abs(metrics["path_gap_bias_m"])

        + W_GAP_MAX
        * metrics["maximum_abs_path_gap_error_m"]

        + W_CTE_RMSE
        * metrics["cte_rmse_m"]

        + W_CTE_P95
        * metrics["cte_p95_m"]

        + W_CTE_MAX
        * metrics["max_abs_cte_m"]

        + W_HEADING_RMSE_DEG
        * metrics["heading_rmse_deg"]

        + W_SPEED_RMSE
        * metrics["speed_rmse_mps"]

        + W_SPEED_P95
        * metrics["speed_p95_abs_error_mps"]

        + W_INITIAL_CATCHUP_TIME
        * metrics["initial_catchup_time_s"]

        + W_LOST_GAP_TIME
        * metrics["lost_gap_time_s"]

        + W_LOST_GAP_EPISODE
        * metrics["lost_gap_episodes"]
    )


def analyse(csv_path: Path):
    rows = read_csv_rows(csv_path)

    if not rows:
        return {
            "success": False,
            "reason": "NO_DATA",
            "score": MAX_PENALTY,
        }

    metrics = compute_metrics(rows)

    if metrics is None:
        return {
            "success": False,
            "reason": "NO_VALID_FOLLOWING_DATA",
            "score": MAX_PENALTY,
        }

    predecessor_success = any(
        parse_bool(r.get("predecessor_success", False))
        for r in rows
    )

    if metrics["collision_detected"]:
        return {
            "success": False,
            "reason": "COLLISION",
            "score": MAX_PENALTY,
            "core_score": MAX_PENALTY,
            **metrics,
        }

    success = predecessor_success

    score = (
        core_score(metrics)

        + W_COLLISION_WARNING_TIME
        * metrics["collision_warning_time_s"]

        + W_AVOIDANCE_TIME
        * metrics["avoidance_active_time_s"]

        + W_TERMINAL_SPEED
        * metrics["terminal_speed_mps"]

        + W_TERMINAL_GAP_ERROR
        * metrics["terminal_path_gap_error_m"]

        + W_FORMATION_SETTLE_TIME
        * metrics["formation_settle_time_s"]

        + W_MISSION_TIME
        * metrics["mission_time_s"]
    )

    if not predecessor_success:
        score += 200.0

    if (
        math.isfinite(metrics["max_abs_cte_m"])
        and metrics["max_abs_cte_m"] > 6.0
    ):
        score += (
            50.0
            + 10.0
            * (metrics["max_abs_cte_m"] - 6.0)
        )

    reason = (
        "SUCCESS"
        if predecessor_success
        else "NO_PREDECESSOR_SUCCESS"
    )

    return {
        "success": success,
        "reason": reason,
        "score": min(score, MAX_PENALTY),
        "core_score": core_score(metrics),
        **metrics,
    }

# ---------------------------------------------------------------------------
# Process management.
# ---------------------------------------------------------------------------

def terminate_group(proc):
    if proc.poll() is not None:
        return

    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=8.0)
        return
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass

    try:
        proc.wait(timeout=3.0)
    except Exception:
        pass


def cleanup_leftovers():
    # First request graceful termination, then force-kill anything that
    # survives. This prevents a stale Gazebo/logger from overlapping the
    # next trial and reproducing the old "waiting for follower logger"
    # failure.
    patterns = [
        "follower_stress_tuning.launch.py",
        "follower_stress_pathgap_v22.launch.py",
        "follower_stress_visual.launch.py",
        "gz sim.*sydney_regatta",
        "platoon_planner.*/stress_course_planner",
        "platoon_control.*/leader_stress_controller",
        "platoon_planner.*/proactive_follower_planner_v21",
        "platoon_planner.*/proactive_follower_planner_v22",
        "platoon_control.*/proactive_follower_controller_v21",
        "platoon_control.*/proactive_follower_controller_v22",
        "platoon_monitor.*/follower_logger",
        "platoon_monitor.*/follower_logger_v22",
        "platoon_state.*/multi_vehicle_state",
        "platoon_monitor.*/r1_stress_logger",
        "platoon_logging.*/r1_stress_logger",
    ]

    for pattern in patterns:
        subprocess.run(
            ["pkill", "-TERM", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    time.sleep(1.0)

    for pattern in patterns:
        subprocess.run(
            ["pkill", "-KILL", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    time.sleep(1.0)

# ---------------------------------------------------------------------------
# Live monitoring and conservative pruning.
# ---------------------------------------------------------------------------

LIVE_FIELDS = [
    "sim_time_s",
    "partial_core_score",
    "early_stop_threshold",
    "bad_streak",

    "path_gap_rmse_m",
    "path_gap_p95_abs_error_m",
    "path_gap_bias_m",
    "maximum_abs_path_gap_error_m",

    "minimum_hitbox_clearance_m",
    "collision_warning_time_s",
    "avoidance_active_time_s",

    "cte_rmse_m",
    "cte_p95_m",
    "heading_rmse_deg",

    "speed_rmse_mps",
    "speed_p95_abs_error_mps",

    "initial_catchup_time_s",
    "lost_gap_time_s",
    "lost_gap_episodes",
]


def append_live_score(path: Path, row: dict):
    new_file = not path.exists()
    with path.open("a", newline="") as fobj:
        writer = csv.DictWriter(
            fobj,
            fieldnames=LIVE_FIELDS,
        )
        if new_file:
            writer.writeheader()
        writer.writerow({
            key: row.get(key)
            for key in LIVE_FIELDS
        })


def early_stop_threshold(
    best_score,
    sim_t,
    min_sim,
    factor_start,
    factor_end,
    margin,
):
    if not math.isfinite(best_score):
        return float("inf")

    progress = clamp(
        (sim_t - min_sim)
        / max(1.0, 300.0 - min_sim),
        0.0,
        1.0,
    )

    factor = (
        factor_start
        + progress * (factor_end - factor_start)
    )

    return best_score * factor + margin


def monitor_trial(
    *,
    proc,
    run_dir: Path,
    trial_name: str,
    live_score_path: Path,
    wall_timeout: float,
    startup_timeout: float,
    label: str,
    best_score: float,
    early_stop_enabled: bool,
    early_min_sim: float,
    early_patience: int,
    early_check_period: float,
    early_factor_start: float,
    early_factor_end: float,
    early_margin: float,
    sim_timeout: float,
):
    started = time.monotonic()
    last_print_sim = -999.0
    last_startup_print = -999.0
    last_score_check_sim = -999.0
    bad_streak = 0
    last_partial = None

    while True:
        wall_elapsed = time.monotonic() - started

        if proc.poll() is not None:
            return {
                "reason": "PROCESS_EXIT",
                "partial": last_partial,
            }

        if wall_elapsed > wall_timeout:
            print(
                f"  [{label}] wall timeout after "
                f"{wall_elapsed:.1f}s",
                flush=True,
            )
            return {
                "reason": "WALL_TIMEOUT",
                "partial": last_partial,
            }

        csv_path = locate_trial_csv(
            run_dir,
            trial_name,
        )

        rows = (
            read_csv_rows(csv_path)
            if csv_path is not None
            else []
        )

        if not rows:
            if wall_elapsed >= startup_timeout:
                print(
                    f"  [{label}] startup failure: no follower "
                    f"CSV data after {wall_elapsed:.1f}s wall",
                    flush=True,
                )
                return {
                    "reason": "STARTUP_FAILURE",
                    "partial": last_partial,
                }

            if wall_elapsed - last_startup_print >= 5.0:
                print(
                    f"  [{label}] starting simulation... "
                    f"waiting for follower logger "
                    f"({wall_elapsed:.0f}s wall)",
                    flush=True,
                )
                last_startup_print = wall_elapsed

            time.sleep(0.7)
            continue

        recent = rows[-min(120, len(rows)):]
        last = rows[-1]

        sim_t = f(last, "time_s", 0.0)
        phase = last.get("follow_phase", "")
        mission = last.get("mission_state", "")

        path_gap = f(last, "path_gap_m")
        hull = f(last, "hitbox_clearance_m")
        cte = f(last, "follower_cross_track_error_m")

        heading_rad = f(
            last,
            "follower_heading_error_rad",
        )
        heading_deg = (
            math.degrees(heading_rad)
            if math.isfinite(heading_rad)
            else float("nan")
        )

        speed = f(last, "follower_speed_mps")
        target_speed = f(
            last,
            "follower_target_speed_mps",
        )

        if (
            sim_t - last_print_sim >= 1.0
            or parse_bool(
                last.get("follower_success", False)
            )
        ):
            print(
                f"  sim={sim_t:7.2f}s "
                f"mission={mission:<16} "
                f"phase={phase:<9} "
                f"gap={path_gap:5.2f} "
                f"hull={hull:5.2f} "
                f"CTE={cte:6.3f} "
                f"head={heading_deg:6.2f}deg "
                f"v={speed:4.2f}/{target_speed:4.2f}",
                flush=True,
            )
            last_print_sim = sim_t

        if parse_bool(
            last.get("predecessor_success", False)
        ):
            print(
                f"  [{label}] predecessor complete; "
                "ending trial before optional follower parking",
                flush=True,
            )
            return {
                "reason": "SUCCESS",
                "partial": last_partial,
            }

        # Actual hull contact is the ONLY safety condition that
        # immediately terminates the run at maximum penalty.
        recent_clearances = finite([
            f(r, "hitbox_clearance_m")
            for r in recent
        ])

        if (
            recent_clearances
            and min(recent_clearances)
            <= COLLISION_CONTACT_EPS_M
        ):
            print(
                f"  [{label}] COLLISION: hull clearance "
                f"{min(recent_clearances):.3f} m",
                flush=True,
            )
            return {
                "reason": "COLLISION",
                "partial": last_partial,
            }

        # Warning and avoidance flags are deliberately NOT early-stop
        # conditions. The scorer applies modest penalties instead.

        recent_following = [
            r for r in recent
            if r.get("follow_phase") == "FOLLOWING"
            and parse_bool(
                r.get("distance_error_valid", False)
            )
        ]

        recent_cte = finite([
            abs(
                f(
                    r,
                    "follower_cross_track_error_m",
                )
            )
            for r in recent_following
        ])

        if (
            len(recent_cte) >= 40
            and max(recent_cte) > 6.0
        ):
            print(
                f"  [{label}] early stop: path departure "
                f"{max(recent_cte):.2f} m",
                flush=True,
            )
            return {
                "reason": "PATH_DEPARTURE",
                "partial": last_partial,
            }

        recent_heading = finite([
            abs(
                f(
                    r,
                    "follower_heading_error_rad",
                )
            )
            for r in recent_following[-60:]
        ])

        if (
            len(recent_heading) >= 40
            and sum(v > 1.40 for v in recent_heading)
            >= int(0.80 * len(recent_heading))
        ):
            print(
                f"  [{label}] early stop: "
                "persistent heading divergence",
                flush=True,
            )
            return {
                "reason": "HEADING_DIVERGENCE",
                "partial": last_partial,
            }

        if sim_t > sim_timeout:
            print(
                f"  [{label}] simulation timeout "
                f"at {sim_t:.1f}s",
                flush=True,
            )
            return {
                "reason": "SIM_TIMEOUT",
                "partial": last_partial,
            }

        if (
            early_stop_enabled
            and sim_t >= early_min_sim
            and (
                sim_t - last_score_check_sim
                >= early_check_period
            )
        ):
            metrics = compute_metrics(rows)

            if metrics is not None:
                partial_core = core_score(metrics)

                threshold = early_stop_threshold(
                    best_score=best_score,
                    sim_t=sim_t,
                    min_sim=early_min_sim,
                    factor_start=early_factor_start,
                    factor_end=early_factor_end,
                    margin=early_margin,
                )

                if partial_core > threshold:
                    bad_streak += 1
                else:
                    bad_streak = 0

                last_partial = {
                    "sim_time_s": sim_t,
                    "partial_core_score": partial_core,
                    "early_stop_threshold": threshold,
                    "bad_streak": bad_streak,
                    **{
                        key: metrics.get(key)
                        for key in LIVE_FIELDS
                        if key not in {
                            "sim_time_s",
                            "partial_core_score",
                            "early_stop_threshold",
                            "bad_streak",
                        }
                    },
                }

                append_live_score(
                    live_score_path,
                    last_partial,
                )

                if bad_streak > 0:
                    print(
                        f"  [{label}] partial "
                        f"core={partial_core:.3f} "
                        f"limit={threshold:.3f} "
                        f"bad={bad_streak}/"
                        f"{early_patience}",
                        flush=True,
                    )

                if bad_streak >= early_patience:
                    return {
                        "reason": "SCORE_EARLY_STOP",
                        "partial": last_partial,
                    }

            last_score_check_sim = sim_t

        time.sleep(0.7)


# ---------------------------------------------------------------------------
# Search spaces.
# ---------------------------------------------------------------------------

def suggest(trial, name):
    lo, hi = BOUNDS[name]
    return trial.suggest_float(name, lo, hi)


def local_suggest(
    trial,
    name,
    base,
    fraction=0.15,
):
    lo, hi = BOUNDS[name]

    span = max(
        abs(float(base)) * fraction,
        (hi - lo) * 0.05,
    )

    local_lo = max(
        lo,
        float(base) - span,
    )

    local_hi = min(
        hi,
        float(base) + span,
    )

    if local_hi <= local_lo:
        return float(base)

    return trial.suggest_float(
        name,
        local_lo,
        local_hi,
    )


def gap_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "distance_kp",
        "distance_ki",
        "distance_kd",
        "catchup_distance",
        "catchup_min_speed",
        "max_distance_speed_correction",
    ):
        controller[name] = suggest(
            trial,
            name,
        )

    return controller, planner


def heading_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    # Heading PID + the guidance parameters that create the desired
    # heading. Without these, a tiny heading error can still coexist
    # with a large outside-corner CTE.
    for name in (
        "heading_kp",
        "heading_ki",
        "heading_kd",
        "lookahead_distance",
        "follow_tangent_half_window",
        "cross_track_heading_gain",
        "max_cross_track_correction_deg",
    ):
        controller[name] = suggest(
            trial,
            name,
        )

    return controller, planner


def speed_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "speed_kp",
        "speed_ki",
        "speed_kd",
    ):
        controller[name] = suggest(
            trial,
            name,
        )

    planner["historical_preview_decel"] = suggest(
        trial,
        "historical_preview_decel",
    )

    return controller, planner


def brake_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "brake_kp",
        "brake_ki",
        "brake_kd",
    ):
        controller[name] = suggest(
            trial,
            name,
        )

    return controller, planner


SPACE_BY_STAGE = {
    "gap": gap_space,
    "heading": heading_space,
    "speed": speed_space,
    "brake": brake_space,
}


# ---------------------------------------------------------------------------
# Trial execution.

# ---------------------------------------------------------------------------
# Trial execution.
# ---------------------------------------------------------------------------

def run_trial(
    *,
    workspace: Path,
    launch_file: str,
    results_root: Path,
    active_dir: Path,
    bad_runs_path: Path,
    trial_name: str,
    stage_name: str,
    follower_id: str,
    predecessor_id: str,
    controller_values: dict,
    planner_values: dict,
    wall_timeout: float,
    progress_label: str,
    best_score: float,
    early_stop_args: dict,
    sim_timeout: float,
):
    run_dir = results_root / trial_name

    if run_dir.exists():
        import shutil
        shutil.rmtree(run_dir)

    run_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    controller_yaml = (
        active_dir
        / f"{trial_name}_controller.yaml"
    )
    planner_yaml = (
        active_dir
        / f"{trial_name}_planner.yaml"
    )

    write_yaml(
        controller_yaml,
        f"/{follower_id}/follower_controller",
        controller_values,
    )
    write_yaml(
        planner_yaml,
        f"/{follower_id}/follower_planner",
        planner_values,
    )

    launch_log = run_dir / "launch.log"
    live_score_path = (
        run_dir / "live_score.csv"
    )

    cleanup_leftovers()

    setup = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {workspace}/install/setup.bash && "
        "exec ros2 launch "
        "platoon_bringup "
        f"{launch_file} "
        f"run_name:={trial_name} "
        f"results_root:={run_dir} "
        f"controller_params_file:={controller_yaml} "
        f"planner_params_file:={planner_yaml} "
        f"follower_id:={follower_id} "
        f"predecessor_id:={predecessor_id} "
        "headless:=True"
    )

    env = os.environ.copy()

    with launch_log.open("w") as log:
        proc = subprocess.Popen(
            ["bash", "-lc", setup],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            preexec_fn=os.setsid,
            text=True,
        )

        monitor = monitor_trial(
            proc=proc,
            run_dir=run_dir,
            trial_name=trial_name,
            live_score_path=live_score_path,
            wall_timeout=wall_timeout,
            label=progress_label,
            best_score=best_score,
            sim_timeout=sim_timeout,
            **early_stop_args,
        )

        terminate_group(proc)

    csv_path = locate_trial_csv(
        run_dir,
        trial_name,
    )

    if csv_path is not None:
        result = analyse(csv_path)
    else:
        result = {
            "success": False,
            "reason": "NO_CSV",
            "score": 1e6,
        }

    result["termination_reason"] = (
        monitor["reason"]
    )
    result["trial_name"] = trial_name
    result["stage"] = stage_name
    result["controller_values"] = (
        controller_values
    )
    result["planner_values"] = planner_values
    result["csv_path"] = (
        str(csv_path)
        if csv_path is not None
        else None
    )

    if (
        monitor["reason"] != "SUCCESS"
        or not result.get("success", False)
    ):
        append_jsonl(
            bad_runs_path,
            {
                "trial_name": trial_name,
                "stage": stage_name,
                "monitor_reason": monitor["reason"],
                "result": result,
                "partial": monitor.get("partial"),
            },
        )

    (
        run_dir / "result.json"
    ).write_text(
        json.dumps(
            result,
            indent=2,
            sort_keys=True,
        )
    )

    return result


# ---------------------------------------------------------------------------
# History / checkpoint.
# ---------------------------------------------------------------------------

HISTORY_FIELDS = [
    "trial",
    "stage",
    "trial_name",
    "accepted",
    "success",
    "score",
    "termination_reason",

    "gap_locked",
    "heading_locked",
    "speed_locked",
    "brake_locked",

    "path_gap_rmse_m",
    "path_gap_p95_abs_error_m",
    "path_gap_bias_m",
    "maximum_abs_path_gap_error_m",

    "minimum_hitbox_clearance_m",
    "collision_warning_time_s",
    "avoidance_active_time_s",
    "collision_detected",

    "cte_rmse_m",
    "cte_p95_m",
    "max_abs_cte_m",
    "heading_rmse_deg",

    "speed_rmse_mps",
    "speed_p95_abs_error_mps",

    "initial_catchup_time_s",
    "lost_gap_time_s",
    "lost_gap_episodes",

    "terminal_speed_mps",
    "terminal_path_gap_error_m",
    "formation_settle_time_s",
    "mission_time_s",

    "controller_json",
    "planner_json",
]


def ensure_history(path: Path):
    if path.exists() and path.stat().st_size > 0:
        return

    with path.open("w", newline="") as fobj:
        csv.DictWriter(
            fobj,
            fieldnames=HISTORY_FIELDS,
        ).writeheader()


def append_history(
    path: Path,
    *,
    trial_number,
    result,
):
    row = {
        key: result.get(key)
        for key in HISTORY_FIELDS
    }
    row["trial"] = trial_number
    row["controller_json"] = json.dumps(
        result.get("controller_values", {}),
        sort_keys=True,
    )
    row["planner_json"] = json.dumps(
        result.get("planner_values", {}),
        sort_keys=True,
    )

    with path.open("a", newline="") as fobj:
        csv.DictWriter(
            fobj,
            fieldnames=HISTORY_FIELDS,
        ).writerow(row)


def write_checkpoint(
    path: Path,
    *,
    seed_done,
    next_schedule_index,
    completed_optimization_trials,
    best_score,
    best_controller,
    best_planner,
    best_result,
):
    payload = {
        "version": 1,
        "seed_done": seed_done,
        "next_schedule_index": (
            next_schedule_index
        ),
        "completed_optimization_trials": (
            completed_optimization_trials
        ),
        "best_score": best_score,
        "best_controller": best_controller,
        "best_planner": best_planner,
        "best_result": best_result,
    }

    tmp = path.with_suffix(".tmp")
    tmp.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
        )
    )
    tmp.replace(path)


def validate_launch_args(
    workspace: Path,
    launch_file: str,
):
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {workspace}/install/setup.bash && "
        "ros2 launch platoon_bringup "
        f"{launch_file} --show-args"
    )

    result = subprocess.run(
        ["bash", "-lc", command],
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        raise SystemExit(
            "Could not inspect tuning launch:\n"
            + result.stdout
            + result.stderr
        )

    required = (
        "controller_params_file",
        "planner_params_file",
        "follower_id",
        "predecessor_id",
        "run_name",
        "results_root",
        "headless",
    )

    missing = [
        name
        for name in required
        if f"'{name}'" not in result.stdout
    ]

    if missing:
        raise SystemExit(
            "Tuning launch is missing required "
            "arguments: "
            + ", ".join(missing)
        )


# ---------------------------------------------------------------------------
# Main.
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Adaptive V2.2 Optuna stress-course autotuner "
            "for one platoon follower."
        )
    )

    parser.add_argument(
        "--trials",
        type=int,
        default=48,
        help=(
            "maximum optimization trials, excluding "
            "the reference seed run"
        ),
    )

    parser.add_argument(
        "--study-name",
        default="r2_follow_adaptive_v22_01",
    )

    parser.add_argument(
        "--workspace",
        default=str(
            Path.home() / "vrx_ws"
        ),
    )

    parser.add_argument(
        "--launch-file",
        default="follower_stress_pathgap_v22.launch.py",
    )

    parser.add_argument(
        "--follower-id",
        default="r2",
    )

    parser.add_argument(
        "--predecessor-id",
        default="r1",
    )

    parser.add_argument(
        "--seed-controller",
        default=None,
    )

    parser.add_argument(
        "--seed-planner",
        default=None,
    )

    parser.add_argument(
        "--skip-seed-validation",
        action="store_true",
    )

    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=420.0,
    )

    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--sim-timeout",
        type=float,
        default=390.0,
    )

    parser.add_argument(
        "--resume",
        action="store_true",
        help=(
            "continue from checkpoint.json and adaptive_state.json "
            "after a previous stop or Ctrl+C pause"
        ),
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
    )

    parser.add_argument(
        "--no-early-stop",
        action="store_true",
    )

    parser.add_argument(
        "--early-min-sim",
        type=float,
        default=90.0,
    )

    parser.add_argument(
        "--early-patience",
        type=int,
        default=3,
    )

    parser.add_argument(
        "--early-check-period",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--early-factor-start",
        type=float,
        default=1.55,
    )

    parser.add_argument(
        "--early-factor-end",
        type=float,
        default=1.25,
    )

    parser.add_argument(
        "--early-margin",
        type=float,
        default=1.0,
    )

    args = parser.parse_args()

    if args.trials < 0:
        raise SystemExit(
            "--trials must be >= 0"
        )

    if args.resume and args.overwrite:
        raise SystemExit(
            "Use either --resume or --overwrite, not both."
        )

    workspace = Path(
        args.workspace
    ).expanduser()

    validate_launch_args(
        workspace,
        args.launch_file,
    )

    project = (
        workspace
        / "src/vrx_platooning"
    )

    out = (
        project
        / "results/autotune_followers"
        / args.study_name
    )

    checkpoint_path = (
        out / "checkpoint.json"
    )

    adaptive_state_path = (
        out / "adaptive_state.json"
    )

    history_path = (
        out / "history.csv"
    )

    bad_runs_path = (
        out / "bad_runs.jsonl"
    )

    storage_path = (
        out / "optuna.sqlite3"
    )

    trials_root = (
        out / "trials"
    )

    active_dir = (
        out / "active_params"
    )

    if out.exists() and any(out.iterdir()):
        if args.overwrite:
            import shutil
            shutil.rmtree(out)
        elif not args.resume:
            raise SystemExit(
                "ERROR: study directory already exists and is not empty:\n"
                f"  {out}\n"
                "Use --resume or --overwrite."
            )

    out.mkdir(
        parents=True,
        exist_ok=True,
    )

    trials_root.mkdir(
        parents=True,
        exist_ok=True,
    )

    active_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    ensure_history(history_path)

    locked = {
        stage: False
        for stage in STAGE_HIERARCHY
    }

    pass_streak = {
        stage: 0
        for stage in STAGE_HIERARCHY
    }

    if args.resume:
        if not checkpoint_path.exists():
            raise SystemExit(
                "--resume requested but checkpoint.json is missing"
            )

        checkpoint = json.loads(
            checkpoint_path.read_text()
        )

        seed_done = bool(
            checkpoint["seed_done"]
        )

        completed_optimization_trials = int(
            checkpoint[
                "completed_optimization_trials"
            ]
        )

        best_global_score = float(
            checkpoint["best_score"]
        )

        controller_best = dict(
            checkpoint["best_controller"]
        )

        planner_best = dict(
            checkpoint["best_planner"]
        )

        best_global_result = (
            checkpoint.get("best_result")
        )

        if adaptive_state_path.exists():
            adaptive = json.loads(
                adaptive_state_path.read_text()
            )

            locked.update(
                adaptive.get("locked", {})
            )

            pass_streak.update(
                adaptive.get(
                    "pass_streak",
                    {},
                )
            )

    else:
        seed_done = bool(
            args.skip_seed_validation
        )

        completed_optimization_trials = 0

        controller_best = dict(
            BASE_CONTROLLER
        )

        planner_best = dict(
            BASE_PLANNER
        )

        controller_best.update(
            load_seed_yaml(
                args.seed_controller
            )
        )

        planner_best.update(
            load_seed_yaml(
                args.seed_planner
            )
        )

        best_global_score = float("inf")
        best_global_result = None

        write_yaml(
            out / "starting_controller.yaml",
            f"/{args.follower_id}/follower_controller",
            controller_best,
        )

        write_yaml(
            out / "starting_planner.yaml",
            f"/{args.follower_id}/follower_planner",
            planner_best,
        )

        write_checkpoint(
            checkpoint_path,
            seed_done=seed_done,
            next_schedule_index=0,
            completed_optimization_trials=(
                completed_optimization_trials
            ),
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

    def write_adaptive_state(current_stage=None):
        adaptive_state_path.write_text(
            json.dumps(
                {
                    "version": 1,
                    "hierarchy": list(
                        STAGE_HIERARCHY
                    ),
                    "current_stage":
                        current_stage,
                    "locked":
                        locked,
                    "pass_streak":
                        pass_streak,
                    "lock_confirm_runs":
                        LOCK_CONFIRM_RUNS,
                    "pass_thresholds":
                        PASS_THRESHOLDS,
                    "unlock_thresholds":
                        UNLOCK_THRESHOLDS,
                },
                indent=2,
                sort_keys=True,
            )
        )

    early_stop_args = {
        "startup_timeout":
            args.startup_timeout,

        "early_stop_enabled":
            not args.no_early_stop,

        "early_min_sim":
            args.early_min_sim,

        "early_patience":
            args.early_patience,

        "early_check_period":
            args.early_check_period,

        "early_factor_start":
            args.early_factor_start,

        "early_factor_end":
            args.early_factor_end,

        "early_margin":
            args.early_margin,
    }

    sampler = optuna.samplers.TPESampler(
        seed=42,
        n_startup_trials=6,
        multivariate=True,
        group=True,
        warn_independent_sampling=False,
    )

    study = optuna.create_study(
        study_name=(
            f"{args.study_name}_v22"
        ),
        storage=(
            f"sqlite:///{storage_path}"
        ),
        direction="minimize",
        sampler=sampler,
        load_if_exists=True,
    )

    print()
    print(
        "============================================================"
    )
    print(
        " ADAPTIVE FOLLOWER STRESS AUTOTUNER V2.2"
    )
    print(
        "============================================================"
    )
    print(
        f"Pair        : "
        f"{args.follower_id} <- "
        f"{args.predecessor_id}"
    )
    print(
        f"Study       : {args.study_name}"
    )
    print(
        "Hierarchy   : "
        + " -> ".join(
            STAGE_HIERARCHY
        )
    )
    print(
        f"Lock confirm: "
        f"{LOCK_CONFIRM_RUNS} passing runs"
    )
    print(
        "Collision   : contact=MAX PENALTY; "
        "warning/avoidance=soft penalty"
    )
    print(
        "Catch-up    : BREADCRUMB + 5.00+/-0.25 m for 2 s; "
        "then strong penalty for gap > 5.75 m"
    )
    print(
        f"Max trials  : {args.trials}"
    )
    print(
        f"Launch      : {args.launch_file}"
    )
    print(
        f"Resume      : "
        f"{'YES' if args.resume else 'NO'}"
    )
    print(
        f"Output      : {out}"
    )
    print(
        "============================================================"
    )
    print()

    # -------------------------------------------------------
    # Validate reference seed.
    # -------------------------------------------------------

    if not seed_done:
        seed_name = (
            f"{args.study_name}_seed"
        )

        print(
            f"[seed] {seed_name}",
            flush=True,
        )

        try:
            seed = run_trial(
                workspace=workspace,
                launch_file=args.launch_file,
                results_root=trials_root,
                active_dir=active_dir,
                bad_runs_path=bad_runs_path,
                trial_name=seed_name,
                stage_name="seed",
                follower_id=args.follower_id,
                predecessor_id=args.predecessor_id,
                controller_values=controller_best,
                planner_values=planner_best,
                wall_timeout=args.wall_timeout,
                progress_label="seed",
                best_score=float("inf"),
                sim_timeout=args.sim_timeout,
                early_stop_args={
                    **early_stop_args,
                    "early_stop_enabled": False,
                },
            )
        except KeyboardInterrupt:
            cleanup_leftovers()
            write_adaptive_state(
                next_unlocked_stage(locked)
            )
            print(
                "\nPAUSED during seed validation. "
                "Run again with --resume to continue.",
                flush=True,
            )
            return

        if not seed["success"]:
            raise SystemExit(
                "Seed validation failed. "
                f"Reason="
                f"{seed.get('termination_reason')}/"
                f"{seed.get('reason')}"
            )

        seed["accepted"] = True

        best_global_score = float(
            seed["score"]
        )

        best_global_result = seed
        seed_done = True

        # The seed can provide the first confirmation for GAP only.
        # Later stages are intentionally not skipped out of hierarchy.
        if stage_pass("gap", seed):
            pass_streak["gap"] = 1
            if pass_streak["gap"] >= LOCK_CONFIRM_RUNS:
                locked["gap"] = True
                print(
                    "  LOCK gap: seed already satisfies "
                    "the V2.2 gap threshold",
                    flush=True,
                )

        for stage in STAGE_HIERARCHY:
            seed[f"{stage}_locked"] = (
                locked[stage]
            )

        append_history(
            history_path,
            trial_number=0,
            result=seed,
        )

        write_yaml(
            out / "best_controller.yaml",
            f"/{args.follower_id}/follower_controller",
            controller_best,
        )

        write_yaml(
            out / "best_planner.yaml",
            f"/{args.follower_id}/follower_planner",
            planner_best,
        )

        (
            out / "best_trial_result.json"
        ).write_text(
            json.dumps(
                best_global_result,
                indent=2,
                sort_keys=True,
            )
        )

        write_checkpoint(
            checkpoint_path,
            seed_done=seed_done,
            next_schedule_index=0,
            completed_optimization_trials=(
                completed_optimization_trials
            ),
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

        write_adaptive_state(
            next_unlocked_stage(locked)
        )

        print(
            f"[seed] score="
            f"{best_global_score:.4f} | "
            f"gap_pass="
            f"{stage_pass('gap', seed)}",
            flush=True,
        )

    # -------------------------------------------------------
    # Adaptive hierarchical optimization.
    # -------------------------------------------------------

    while (
        completed_optimization_trials
        < args.trials
    ):
        stage = next_unlocked_stage(
            locked
        )

        if stage is None:
            print(
                "All controller stages are locked inside "
                "their acceptance thresholds. Tuning complete.",
                flush=True,
            )
            break

        trial = study.ask()

        controller_candidate = dict(
            controller_best
        )

        planner_candidate = dict(
            planner_best
        )

        (
            controller_candidate,
            planner_candidate,
        ) = SPACE_BY_STAGE[stage](
            trial,
            controller_candidate,
            planner_candidate,
        )

        optimization_index = (
            completed_optimization_trials + 1
        )

        trial_name = (
            f"{args.study_name}_"
            f"{optimization_index:03d}_"
            f"{stage}"
        )

        print(
            f"[{optimization_index:03d}/"
            f"{args.trials:03d}] "
            f"stage={stage} "
            f"locks="
            + ",".join(
                f"{name}:{'L' if locked[name] else '-'}"
                for name in STAGE_HIERARCHY
            ),
            flush=True,
        )

        try:
            result = run_trial(
                workspace=workspace,
                launch_file=args.launch_file,
                results_root=trials_root,
                active_dir=active_dir,
                bad_runs_path=bad_runs_path,
                trial_name=trial_name,
                stage_name=stage,
                follower_id=args.follower_id,
                predecessor_id=args.predecessor_id,
                controller_values=(
                    controller_candidate
                ),
                planner_values=(
                    planner_candidate
                ),
                wall_timeout=args.wall_timeout,
                progress_label=(
                    f"{optimization_index:03d}/{stage}"
                ),
                best_score=best_global_score,
                sim_timeout=args.sim_timeout,
                early_stop_args=(
                    early_stop_args
                ),
            )
        except KeyboardInterrupt:
            cleanup_leftovers()

            try:
                study.tell(
                    trial,
                    state=optuna.trial.TrialState.FAIL,
                )
            except Exception:
                pass

            write_checkpoint(
                checkpoint_path,
                seed_done=seed_done,
                next_schedule_index=0,
                completed_optimization_trials=(
                    completed_optimization_trials
                ),
                best_score=best_global_score,
                best_controller=controller_best,
                best_planner=planner_best,
                best_result=best_global_result,
            )

            write_adaptive_state(
                next_unlocked_stage(locked)
            )

            print(
                "\nPAUSED. The interrupted candidate was discarded. "
                "Run again with --resume to continue from the last "
                "completed trial.",
                flush=True,
            )
            break

        score = float(
            result.get(
                "score",
                MAX_PENALTY,
            )
        )

        study.tell(
            trial,
            score,
        )

        baseline_pass = (
            stage_pass(
                stage,
                best_global_result,
            )
            if best_global_result
            else False
        )

        candidate_pass = stage_pass(
            stage,
            result,
        )

        baseline_stage_score = (
            stage_objective(
                stage,
                best_global_result,
            )
            if best_global_result
            else float("inf")
        )

        candidate_stage_score = (
            stage_objective(
                stage,
                result,
            )
            if result.get("success", False)
            else float("inf")
        )

        # Accept when total score improves, OR when this candidate
        # newly gets the active stage inside threshold. This allows the
        # hierarchy to progress instead of requiring every local stage
        # improvement to also be an immediate global-score improvement.
        accepted = (
            result.get("success", False)
            and (
                score < best_global_score
                or (
                    candidate_pass
                    and not baseline_pass
                )
                or (
                    candidate_pass
                    and baseline_pass
                    and candidate_stage_score
                    < baseline_stage_score
                )
            )
        )

        result["accepted"] = accepted

        if accepted:
            best_global_score = score

            controller_best = dict(
                controller_candidate
            )

            planner_best = dict(
                planner_candidate
            )

            best_global_result = result

            # Monitor every locked stage on the accepted controller.
            # If another PID/guidance change pushed it outside the wider
            # unlock threshold, reopen it. Because stage selection always
            # chooses the earliest unlocked stage, it is tuned NEXT.
            for locked_stage in STAGE_HIERARCHY:
                if (
                    locked.get(
                        locked_stage,
                        False,
                    )
                    and not stage_within_unlock(
                        locked_stage,
                        result,
                    )
                ):
                    locked[locked_stage] = False
                    pass_streak[locked_stage] = 0

                    print(
                        f"  UNLOCK {locked_stage}: "
                        "accepted later-stage change caused drift",
                        flush=True,
                    )

            write_yaml(
                out / "best_controller.yaml",
                f"/{args.follower_id}/follower_controller",
                controller_best,
            )

            write_yaml(
                out / "best_planner.yaml",
                f"/{args.follower_id}/follower_planner",
                planner_best,
            )

            (
                out / "best_trial_result.json"
            ).write_text(
                json.dumps(
                    best_global_result,
                    indent=2,
                    sort_keys=True,
                )
            )

        # Confirm the active-stage threshold. A passing non-accepted
        # trial can count as repeatability only if the current accepted
        # baseline is itself already inside the threshold.
        accepted_baseline_pass = (
            stage_pass(
                stage,
                best_global_result,
            )
            if best_global_result
            else False
        )

        if (
            candidate_pass
            and accepted_baseline_pass
        ):
            pass_streak[stage] += 1
        else:
            pass_streak[stage] = 0

        if (
            not locked[stage]
            and pass_streak[stage]
            >= LOCK_CONFIRM_RUNS
            and accepted_baseline_pass
        ):
            locked[stage] = True

            print(
                f"  LOCK {stage}: "
                f"{pass_streak[stage]} consecutive "
                "threshold confirmations",
                flush=True,
            )

        completed_optimization_trials += 1

        for stage_name in STAGE_HIERARCHY:
            result[f"{stage_name}_locked"] = (
                locked[stage_name]
            )

        append_history(
            history_path,
            trial_number=(
                optimization_index
            ),
            result=result,
        )

        write_checkpoint(
            checkpoint_path,
            seed_done=seed_done,
            next_schedule_index=0,
            completed_optimization_trials=(
                completed_optimization_trials
            ),
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

        write_adaptive_state(
            next_unlocked_stage(
                locked
            )
        )

        print(
            f"  score={score:.4f} "
            f"working={best_global_score:.4f} "
            f"accepted={accepted} "
            f"pass={candidate_pass} "
            f"streak={pass_streak[stage]}/"
            f"{LOCK_CONFIRM_RUNS}",
            flush=True,
        )

    summary = {
        "study_name":
            args.study_name,

        "follower_id":
            args.follower_id,

        "predecessor_id":
            args.predecessor_id,

        "hierarchy":
            list(STAGE_HIERARCHY),

        "locked":
            locked,

        "pass_streak":
            pass_streak,

        "completed_optimization_trials":
            completed_optimization_trials,

        "working_score":
            best_global_score,

        "best_controller":
            controller_best,

        "best_planner":
            planner_best,

        "best_result":
            best_global_result,

        "optuna_best_value":
            (
                study.best_value
                if len(study.trials) > 0
                else None
            ),

        "optuna_best_params":
            (
                study.best_params
                if len(study.trials) > 0
                else {}
            ),

        "pass_thresholds":
            PASS_THRESHOLDS,

        "unlock_thresholds":
            UNLOCK_THRESHOLDS,

        "early_stop":
            early_stop_args,
    }

    (
        out / "study_summary.json"
    ).write_text(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True,
        )
    )

    print()
    print(
        "============================================================"
    )
    print(
        " ADAPTIVE FOLLOWER AUTOTUNE COMPLETE"
    )
    print(
        "============================================================"
    )
    print(
        "Locks       : "
        + ", ".join(
            f"{stage}="
            f"{'LOCKED' if locked[stage] else 'OPEN'}"
            for stage in STAGE_HIERARCHY
        )
    )
    print(
        f"Working score: "
        f"{best_global_score:.4f}"
    )
    print(
        f"Controller  : "
        f"{out / 'best_controller.yaml'}"
    )
    print(
        f"Planner     : "
        f"{out / 'best_planner.yaml'}"
    )
    print(
        f"History     : "
        f"{history_path}"
    )
    print(
        f"Adaptive    : "
        f"{adaptive_state_path}"
    )
    print(
        "============================================================"
    )


if __name__ == "__main__":
    main()
