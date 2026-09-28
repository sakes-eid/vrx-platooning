#!/usr/bin/env python3
"""
Generic follower stress-course autotuner.

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
# Follower-only formation parameters use the validated V2.1 values.
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
}

BASE_PLANNER = {
    "historical_preview_decel": 0.18,
    "historical_preview_max_speed": 1.50,
}


# ---------------------------------------------------------------------------
# Objective weights.
# Formation accuracy is intentionally dominant.
# ---------------------------------------------------------------------------

W_CLEARANCE_RMSE = 5.00
W_CLEARANCE_P95 = 2.00
W_CLEARANCE_BIAS = 1.50

W_CTE_RMSE = 0.90
W_HEADING_RMSE_DEG = 0.050
W_SPEED_RMSE = 0.50

W_CATCHUP_TIME = 0.018
W_CATCHUP_EPISODE = 0.012
W_ACQUISITION_TIME = 0.012
W_TERMINAL_SPEED = 0.50
W_TERMINAL_CLEARANCE = 0.50
W_FORMATION_SETTLE_TIME = 0.05
W_MISSION_TIME = 0.0015

TARGET_CLEARANCE_M = 5.0
COLLISION_HARD_CLEARANCE_M = 3.0
SOFT_CLEARANCE_FLOOR_M = 3.6


# ---------------------------------------------------------------------------
# Search bounds.
# ---------------------------------------------------------------------------

BOUNDS = {
    "heading_kp": (500.0, 1800.0),
    "heading_ki": (0.0, 55.0),
    "heading_kd": (40.0, 300.0),

    "speed_kp": (40.0, 240.0),
    "speed_ki": (0.0, 45.0),
    "speed_kd": (8.0, 130.0),

    "distance_kp": (0.05, 1.20),
    "distance_ki": (0.0, 0.15),
    "distance_kd": (0.0, 0.50),

    "brake_kp": (100.0, 380.0),
    "brake_ki": (0.0, 95.0),
    "brake_kd": (0.0, 45.0),

    "catchup_distance": (5.4, 7.0),
    "catchup_min_speed": (0.90, 1.50),
    "max_distance_speed_correction": (0.30, 1.00),

    "historical_preview_decel": (0.08, 0.35),
}


STAGE_CYCLE = (
    "gap",
    "gap",
    "gap",

    "heading",
    "heading",
    "heading",

    "speed",
    "speed",
    "speed",

    "joint",
    "joint",
    "joint",
)


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

def compute_metrics(rows):
    if not rows:
        return None

    dt = sample_dt(rows)

    following = [
        r for r in rows
        if r.get("follow_phase") == "FOLLOWING"
        and parse_bool(r.get("distance_error_valid", False))
    ]

    if len(following) < 20:
        return None

    clearance_errors = finite([
        f(r, "hitbox_clearance_error_m")
        for r in following
    ])
    clearances_all = finite([
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

    first_follow_time = min(
        f(r, "time_s")
        for r in following
        if math.isfinite(f(r, "time_s"))
    )

    post_acquisition = [
        r for r in rows
        if f(r, "time_s", -1.0) >= first_follow_time
    ]

    catchup_rows = [
        r for r in post_acquisition
        if r.get("follow_phase") == "CATCHUP"
    ]
    catchup_episodes = episode_count(
        [r.get("follow_phase", "") for r in post_acquisition],
        "CATCHUP",
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

    # Tuning ends when the predecessor has completed its mission.
    # Follower parking after this point is optional and deliberately excluded.
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

    terminal_clearance_error = (
        abs(
            f(terminal_row, "hitbox_clearance_m")
            - TARGET_CLEARANCE_M
        )
        if terminal_row is not None
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

    if (
        math.isfinite(target_capture_time)
        and hold_times
    ):
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

    mean_clearance_error = (
        sum(clearance_errors) / len(clearance_errors)
        if clearance_errors else float("nan")
    )

    return {
        "clearance_rmse_m": rmse(clearance_errors),
        "clearance_p95_abs_error_m": percentile(
            [abs(v) for v in clearance_errors],
            0.95,
        ),
        "clearance_bias_m": mean_clearance_error,
        "mean_hitbox_clearance_m": (
            TARGET_CLEARANCE_M + mean_clearance_error
            if math.isfinite(mean_clearance_error)
            else float("nan")
        ),
        "minimum_hitbox_clearance_m": (
            min(clearances_all)
            if clearances_all
            else float("nan")
        ),

        "cte_rmse_m": rmse(cte),
        "cte_p95_m": percentile(
            [abs(v) for v in cte],
            0.95,
        ),
        "max_abs_cte_m": (
            max([abs(v) for v in cte])
            if cte else float("nan")
        ),
        "heading_rmse_deg": math.degrees(
            rmse(heading_rad)
        ) if heading_rad else float("nan"),
        "speed_rmse_mps": rmse(speed_errors),

        "first_follow_time_s": first_follow_time,
        "catchup_time_s": len(catchup_rows) * dt,
        "catchup_episodes": catchup_episodes,

        "predecessor_success_time_s": predecessor_success_time,
        "follower_success_time_s": follower_success_time,
        "terminal_speed_mps": terminal_speed,
        "terminal_clearance_error_m": terminal_clearance_error,
        "formation_settle_time_s": formation_settle_time,
        "mission_time_s": mission_time,

        "collision_warning_samples": collision_warning_samples,
        "avoidance_active_samples": avoidance_active_samples,

        "following_samples": len(following),
        "rows": len(rows),
        "sample_dt_s": dt,
    }


def core_score(metrics):
    required = [
        "clearance_rmse_m",
        "clearance_p95_abs_error_m",
        "clearance_bias_m",
        "cte_rmse_m",
        "heading_rmse_deg",
        "speed_rmse_mps",
    ]

    if any(
        not math.isfinite(metrics.get(key, float("nan")))
        for key in required
    ):
        return 1e6

    return (
        W_CLEARANCE_RMSE
        * metrics["clearance_rmse_m"]

        + W_CLEARANCE_P95
        * metrics["clearance_p95_abs_error_m"]

        + W_CLEARANCE_BIAS
        * abs(metrics["clearance_bias_m"])

        + W_CTE_RMSE
        * metrics["cte_rmse_m"]

        + W_HEADING_RMSE_DEG
        * metrics["heading_rmse_deg"]

        + W_SPEED_RMSE
        * metrics["speed_rmse_mps"]

        + W_CATCHUP_TIME
        * metrics["catchup_time_s"]

        + W_CATCHUP_EPISODE
        * metrics["catchup_episodes"]

        + W_ACQUISITION_TIME
        * metrics["first_follow_time_s"]
    )


def analyse(csv_path: Path):
    rows = read_csv_rows(csv_path)

    if not rows:
        return {
            "success": False,
            "reason": "NO_DATA",
            "score": 1e6,
        }

    metrics = compute_metrics(rows)

    if metrics is None:
        return {
            "success": False,
            "reason": "NO_VALID_FOLLOWING_DATA",
            "score": 1e6,
        }

    predecessor_success = any(
        parse_bool(r.get("predecessor_success", False))
        for r in rows
    )

    unsafe = (
        metrics["collision_warning_samples"] > 0
        or metrics["avoidance_active_samples"] > 0
        or (
            math.isfinite(
                metrics["minimum_hitbox_clearance_m"]
            )
            and metrics["minimum_hitbox_clearance_m"]
            < COLLISION_HARD_CLEARANCE_M
        )
    )

    success = predecessor_success and not unsafe

    score = (
        core_score(metrics)
        + W_TERMINAL_SPEED
        * metrics["terminal_speed_mps"]
        + W_TERMINAL_CLEARANCE
        * metrics["terminal_clearance_error_m"]
        + W_FORMATION_SETTLE_TIME
        * metrics["formation_settle_time_s"]
        + W_MISSION_TIME
        * metrics["mission_time_s"]
    )

    minimum_clearance = metrics["minimum_hitbox_clearance_m"]
    if (
        math.isfinite(minimum_clearance)
        and minimum_clearance < SOFT_CLEARANCE_FLOOR_M
    ):
        shortfall = SOFT_CLEARANCE_FLOOR_M - minimum_clearance
        score += 20.0 * shortfall * shortfall

    if not predecessor_success:
        score += 200.0

    if unsafe:
        score += 500.0

    if (
        math.isfinite(metrics["max_abs_cte_m"])
        and metrics["max_abs_cte_m"] > 6.0
    ):
        score += (
            50.0
            + 10.0 * (
                metrics["max_abs_cte_m"] - 6.0
            )
        )

    reason = "SUCCESS"
    if unsafe:
        reason = "UNSAFE"
    elif not predecessor_success:
        reason = "NO_PREDECESSOR_SUCCESS"

    return {
        "success": success,
        "reason": reason,
        "score": score,
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
    patterns = [
        "follower_stress_tuning.launch.py",
        "gz sim.*sydney_regatta",
        "platoon_planner.*/stress_course_planner",
        "platoon_control.*/leader_stress_controller",
        "platoon_planner.*/proactive_follower_planner_v21",
        "platoon_control.*/proactive_follower_controller_v21",
        "platoon_monitor.*/follower_logger",
        "platoon_state.*/multi_vehicle_state",
        "platoon_logging.*/r1_stress_logger",
    ]

    for pattern in patterns:
        subprocess.run(
            ["pkill", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    time.sleep(1.2)


# ---------------------------------------------------------------------------
# Live monitoring and conservative pruning.
# ---------------------------------------------------------------------------

LIVE_FIELDS = [
    "sim_time_s",
    "partial_core_score",
    "early_stop_threshold",
    "bad_streak",
    "clearance_rmse_m",
    "clearance_p95_abs_error_m",
    "clearance_bias_m",
    "minimum_hitbox_clearance_m",
    "cte_rmse_m",
    "heading_rmse_deg",
    "speed_rmse_mps",
    "catchup_time_s",
    "catchup_episodes",
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
            if (
                wall_elapsed - last_startup_print
                >= 5.0
            ):
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
        clearance = f(last, "hitbox_clearance_m")
        cte = f(last, "follower_cross_track_error_m")
        heading_deg = math.degrees(
            f(last, "follower_heading_error_rad")
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
                f"gap={clearance:5.2f} "
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

        # Hard safety rejection.
        if any(
            parse_bool(r.get("collision_warning", False))
            or parse_bool(r.get("avoidance_active", False))
            for r in recent
        ):
            print(
                f"  [{label}] early stop: "
                "collision warning / avoidance active",
                flush=True,
            )
            return {
                "reason": "UNSAFE_CLEARANCE",
                "partial": last_partial,
            }

        recent_clearances = finite([
            f(r, "hitbox_clearance_m")
            for r in recent
        ])

        if (
            len(recent_clearances) >= 10
            and min(recent_clearances)
            < COLLISION_HARD_CLEARANCE_M
        ):
            print(
                f"  [{label}] early stop: "
                f"clearance {min(recent_clearances):.2f} m",
                flush=True,
            )
            return {
                "reason": "UNSAFE_CLEARANCE",
                "partial": last_partial,
            }

        # Path/heading divergence is meaningful only after the
        # follower has actually acquired the predecessor trail.
        # Large trail-relative CTE is normal during initial CATCHUP.
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
                f"  [{label}] early stop: "
                f"path departure {max(recent_cte):.2f} m",
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
    local_lo = max(lo, float(base) - span)
    local_hi = min(hi, float(base) + span)
    if local_hi <= local_lo:
        return float(base)
    return trial.suggest_float(
        name,
        local_lo,
        local_hi,
    )


def heading_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "heading_kp",
        "heading_ki",
        "heading_kd",
    ):
        controller[name] = suggest(trial, name)

    return controller, planner


def speed_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "speed_kp",
        "speed_ki",
        "speed_kd",
    ):
        controller[name] = suggest(trial, name)

    planner["historical_preview_decel"] = suggest(
        trial,
        "historical_preview_decel",
    )

    return controller, planner


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
        controller[name] = suggest(trial, name)

    return controller, planner


def brake_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "brake_kp",
        "brake_ki",
        "brake_kd",
    ):
        controller[name] = suggest(trial, name)

    return controller, planner


def joint_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    for name in (
        "heading_kp",
        "heading_ki",
        "heading_kd",
        "speed_kp",
        "speed_ki",
        "speed_kd",
        "distance_kp",
        "distance_ki",
        "distance_kd",
        "catchup_distance",
        "catchup_min_speed",
        "max_distance_speed_correction",
    ):
        controller[name] = local_suggest(
            trial,
            name,
            controller[name],
        )

    planner["historical_preview_decel"] = (
        local_suggest(
            trial,
            "historical_preview_decel",
            planner["historical_preview_decel"],
        )
    )

    return controller, planner


SPACE_BY_STAGE = {
    "gap": gap_space,
    "heading": heading_space,
    "speed": speed_space,
    "joint": joint_space,
}


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
    "success",
    "score",
    "termination_reason",
    "clearance_rmse_m",
    "clearance_p95_abs_error_m",
    "clearance_bias_m",
    "minimum_hitbox_clearance_m",
    "cte_rmse_m",
    "heading_rmse_deg",
    "speed_rmse_mps",
    "first_follow_time_s",
    "catchup_time_s",
    "catchup_episodes",
    "terminal_lag_s",
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
            "Generic, resumable Optuna stress-course "
            "autotuner for one platoon follower."
        )
    )

    parser.add_argument(
        "--trials",
        type=int,
        default=36,
        help=(
            "optimization trials, excluding "
            "the reference seed run"
        ),
    )
    parser.add_argument(
        "--study-name",
        default="r2_follow_tune_01",
    )
    parser.add_argument(
        "--workspace",
        default=str(
            Path.home() / "vrx_ws"
        ),
    )
    parser.add_argument(
        "--launch-file",
        default="follower_stress_tuning.launch.py",
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
        help=(
            "start optimization directly from supplied seed parameters "
            "without running a separate validation simulation"
        ),
    )

    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=420.0,
    )
    parser.add_argument(
        "--sim-timeout",
        type=float,
        default=390.0,
    )

    parser.add_argument(
        "--resume",
        action="store_true",
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
            "Use either --resume or --overwrite, "
            "not both."
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
                "ERROR: study directory already "
                "exists and is not empty:\n"
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

    if args.resume:
        if not checkpoint_path.exists():
            raise SystemExit(
                "--resume requested but "
                "checkpoint.json is missing"
            )

        checkpoint = json.loads(
            checkpoint_path.read_text()
        )

        seed_done = bool(
            checkpoint["seed_done"]
        )
        next_schedule_index = int(
            checkpoint[
                "next_schedule_index"
            ]
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

    else:
        seed_done = bool(args.skip_seed_validation)
        next_schedule_index = 0
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

        if args.skip_seed_validation:
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
            next_schedule_index=next_schedule_index,
            completed_optimization_trials=(
                completed_optimization_trials
            ),
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

    early_stop_args = {
        "early_stop_enabled": (
            not args.no_early_stop
        ),
        "early_min_sim": args.early_min_sim,
        "early_patience": args.early_patience,
        "early_check_period": (
            args.early_check_period
        ),
        "early_factor_start": (
            args.early_factor_start
        ),
        "early_factor_end": (
            args.early_factor_end
        ),
        "early_margin": args.early_margin,
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
            f"{args.study_name}_v1"
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
        " GENERIC FOLLOWER STRESS AUTOTUNER V1"
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
        f"Trials      : {args.trials} optimization"
        + (
            " | seed validation SKIPPED"
            if args.skip_seed_validation
            else " + seed"
        )
    )
    print(
        "Schedule    : "
        + " -> ".join(STAGE_CYCLE)
        + " -> ..."
    )
    print(
        "Mode        : LIGHT / HEADLESS"
    )
    print(
        "Sampler     : Optuna TPE | "
        "multivariate=True | group=True"
    )
    print(
        "Early stop  : "
        f"{'OFF' if args.no_early_stop else 'ON'} "
        f"(min_sim={args.early_min_sim:.0f}s, "
        f"patience={args.early_patience})"
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
    # Validate reference seed before optimization.
    # -------------------------------------------------------

    if not seed_done:
        seed_name = (
            f"{args.study_name}_seed"
        )

        print(
            f"[seed] {seed_name}",
            flush=True,
        )

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

        append_history(
            history_path,
            trial_number=0,
            result=seed,
        )

        if not seed["success"]:
            raise SystemExit(
                "Seed validation did not complete "
                "successfully. Do not tune around "
                "an unvalidated seed. "
                f"Reason="
                f"{seed.get('termination_reason')}/"
                f"{seed.get('reason')}"
            )

        best_global_score = float(
            seed["score"]
        )
        best_global_result = seed
        seed_done = True

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
            out
            / "best_trial_result.json"
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
            next_schedule_index=next_schedule_index,
            completed_optimization_trials=(
                completed_optimization_trials
            ),
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

        print(
            f"[seed] score="
            f"{best_global_score:.4f}",
            flush=True,
        )

    # -------------------------------------------------------
    # Optimization loop.
    # -------------------------------------------------------

    remaining = (
        args.trials
        - completed_optimization_trials
    )

    if remaining <= 0:
        print(
            "Requested optimization trial count "
            "already completed."
        )

    for _ in range(max(0, remaining)):
        stage = STAGE_CYCLE[
            next_schedule_index
            % len(STAGE_CYCLE)
        ]

        trial = study.ask()

        controller_candidate = dict(
            controller_best
        )
        planner_candidate = dict(
            planner_best
        )

        controller_candidate, planner_candidate = (
            SPACE_BY_STAGE[stage](
                trial,
                controller_candidate,
                planner_candidate,
            )
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
            f"optuna={trial.number}",
            flush=True,
        )

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
                f"{optimization_index:03d}"
                f"/{stage}"
            ),
            best_score=best_global_score,
            sim_timeout=args.sim_timeout,
            early_stop_args=(
                early_stop_args
            ),
        )

        score = float(
            result.get("score", 1e6)
        )

        study.tell(
            trial,
            score,
        )

        append_history(
            history_path,
            trial_number=(
                optimization_index
            ),
            result=result,
        )

        improved = (
            result.get("success", False)
            and score < best_global_score
        )

        if improved:
            best_global_score = score
            controller_best = dict(
                controller_candidate
            )
            planner_best = dict(
                planner_candidate
            )
            best_global_result = result

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
                out
                / "best_trial_result.json"
            ).write_text(
                json.dumps(
                    best_global_result,
                    indent=2,
                    sort_keys=True,
                )
            )

            print(
                f"  NEW BEST score="
                f"{best_global_score:.4f}",
                flush=True,
            )
        else:
            print(
                f"  score={score:.4f} "
                f"best={best_global_score:.4f} "
                f"success="
                f"{result.get('success', False)}",
                flush=True,
            )

        completed_optimization_trials += 1
        next_schedule_index += 1

        write_checkpoint(
            checkpoint_path,
            seed_done=seed_done,
            next_schedule_index=(
                next_schedule_index
            ),
            completed_optimization_trials=(
                completed_optimization_trials
            ),
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

    summary = {
        "study_name": args.study_name,
        "follower_id": args.follower_id,
        "predecessor_id": (
            args.predecessor_id
        ),
        "completed_optimization_trials": (
            completed_optimization_trials
        ),
        "best_score": best_global_score,
        "best_controller": controller_best,
        "best_planner": planner_best,
        "best_result": best_global_result,
        "optuna_best_value": (
            study.best_value
            if len(study.trials) > 0
            else None
        ),
        "optuna_best_params": (
            study.best_params
            if len(study.trials) > 0
            else {}
        ),
        "early_stop": early_stop_args,
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
        " FOLLOWER AUTOTUNE COMPLETE"
    )
    print(
        "============================================================"
    )
    print(
        f"Best score  : "
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
        f"Checkpoint  : "
        f"{checkpoint_path}"
    )
    print(
        "============================================================"
    )


if __name__ == "__main__":
    main()
