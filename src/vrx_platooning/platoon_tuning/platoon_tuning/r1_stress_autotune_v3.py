#!/usr/bin/env python3
"""
R1 stress-course autotuner V3.

Key changes from V2
-------------------
1. Cyclic schedule:
      guidance -> speed -> joint -> guidance -> speed -> joint -> ...
   with braking deliberately left until the end.

   For 36 optimization trials the default plan is:
      G4 S2 J3 | G4 S2 J3 | G4 S2 J3 | G3 J3 | B3

   Totals:
      guidance = 15
      speed    = 6
      joint    = 12
      brake    = 3

2. Robust speed scoring:
   isolated velocity spikes are excluded from speed-RMSE and straight-speed
   calculations, but are still reported in the trial result.

3. Continuous partial-score monitoring:
   obviously poor candidates can be terminated early after a conservative
   warm-up period. A candidate must remain clearly worse than the current
   best for several consecutive checks before it is stopped.

4. Bad-run registry:
   early-stopped / failed runs are written to bad_runs.jsonl for later review.

5. True checkpoint/resume:
   --resume continues the same V3 study from checkpoint.json without deleting
   previous trials or repeating the validated seed.

6. Seed YAML support:
   --seed-controller and --seed-planner can point at the best YAML files from
   Tune 05 (or any earlier study).

This tuner still uses the same full ordered stress course and does NOT tune
course geometry or ordered-path safety parameters.
"""

import argparse
import csv
import json
import math
import os
import signal
import statistics
import subprocess
import time
from pathlib import Path

try:
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
except Exception as exc:
    raise SystemExit(
        "Optuna is required. Check with: "
        "python3 -c 'import optuna; print(optuna.__version__)'"
    ) from exc

try:
    import yaml
except Exception as exc:
    raise SystemExit(
        "PyYAML is required. Check with: "
        "python3 -c 'import yaml; print(yaml.__version__)'"
    ) from exc


# =============================================================
# Validated starting defaults
# =============================================================

BASE_CONTROLLER = {
    'heading_kp': 911.4007441588406,
    'heading_ki': 25.65165107782674,
    'heading_kd': 98.10493190966012,

    'speed_kp': 127.83877941647718,
    'speed_ki': 18.383775431748894,
    'speed_kd': 49.388692087475725,

    'brake_kp': 177.8935198208073,
    'brake_ki': 60.634764417119634,
    'brake_kd': 10.647182804717701,

    'heading_slowdown_angle': 0.8,

    'max_planner_speed': 2.0,
    'max_forward_thrust': 700.0,
    'max_turn_thrust': 400.0,
    'max_total_thrust': 1000.0,

    'max_brake_thrust': 500.0,
    'stop_speed_tolerance': 0.05,

    'recovery_target_speed': 0.25,
    'recovery_brake_distance': 0.75,
    'recovery_align_tolerance': 0.15,
    'recovery_realign_tolerance': 0.35,
    'recovery_max_forward_thrust': 250.0,
    'recovery_max_turn_thrust': 250.0,

    'hold_duration': 5.0,

    # Ordered-path safety: NEVER tune these.
    'waypoint_capture_radius': 0.45,
    'progress_corridor_m': 3.5,
    'max_waypoint_advances_per_cycle': 8,
}

BASE_PLANNER = {
    # Course geometry: NEVER tune the test itself.
    'point_spacing': 0.50,
    'straight_length': 80.0,
    'turn_90_radius': 5.0,
    'semicircle_small_radius': 6.0,
    'semicircle_large_radius': 10.0,
    'coverage_lane_length': 14.0,
    'coverage_tight_radius': 5.0,
    'coverage_wide_radius': 9.0,

    'max_speed': 2.0,
    'minimum_turn_speed': 0.65,
    'lateral_accel_limit': 0.30,
    'accel_limit': 0.45,
    'decel_limit': 0.55,

    'preview_distance': 20.0,
    'turn_curvature_threshold': 0.040,
    'tight_curvature_threshold': 0.140,
    'medium_curvature_threshold': 0.070,

    'lookahead_min': 2.2,
    'lookahead_max': 7.0,
    'lookahead_speed_gain': 1.8,
    'tight_lookahead': 2.6,
    'medium_lookahead': 3.6,

    'advisory_rate_hz': 10.0,
}


# =============================================================
# Score weights
#
# Guidance remains dominant.
# The scale stays close to the V2 score (~6 for a strong run).
# =============================================================

W_CTE_RMSE = 5.00
W_CTE_P95 = 1.80
W_MAX_CTE = 0.75
W_HEADING_RMSE_DEG = 0.040
W_SPEED_RMSE = 0.55
W_STRAIGHT_SPEED_SHORTFALL = 0.75

W_TRACK_TIME = 0.0030
W_TERMINAL_TIME = 0.020
W_RECOVERY_TIME = 0.10
W_RECOVERY_EPISODE = 0.35

STRAIGHT_SPEED_TARGET = 1.75


# =============================================================
# Basic utilities
# =============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def write_yaml(path: Path, node_name: str, values: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f'{node_name}:', '  ros__parameters:']
    for key in sorted(values):
        value = values[key]
        if isinstance(value, bool):
            value_text = 'true' if value else 'false'
        elif isinstance(value, str):
            value_text = json.dumps(value)
        else:
            value_text = repr(value)
        lines.append(f'    {key}: {value_text}')
    path.write_text('\n'.join(lines) + '\n')


def load_seed_yaml(path_text, node_name):
    if not path_text:
        return {}
    path = Path(path_text).expanduser()
    if not path.exists():
        raise SystemExit(f'Seed YAML does not exist: {path}')
    data = yaml.safe_load(path.read_text()) or {}
    return dict(data.get(node_name, {}).get('ros__parameters', {}) or {})


def read_csv_rows(path: Path):
    if not path.exists() or path.stat().st_size < 40:
        return []
    try:
        with path.open(newline='') as fobj:
            return list(csv.DictReader(fobj))
    except Exception:
        return []


def f(row, key, default=float('nan')):
    try:
        return float(row.get(key, default))
    except Exception:
        return default


def rmse(values):
    if not values:
        return 99.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def percentile_sorted(sorted_values, fraction):
    if not sorted_values:
        return 99.0
    idx = min(
        len(sorted_values) - 1,
        max(0, int(round(fraction * (len(sorted_values) - 1)))),
    )
    return sorted_values[idx]


def trimmed_mean(values, trim_fraction=0.05):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return 0.0
    if len(values) < 20:
        return sum(values) / len(values)

    cut = int(len(values) * trim_fraction)
    if cut <= 0 or 2 * cut >= len(values):
        return sum(values) / len(values)

    kept = values[cut:len(values) - cut]
    return sum(kept) / len(kept)


# =============================================================
# Robust speed-sample handling
# =============================================================

def isolated_series_outlier_indices(values, threshold):
    """
    Detect isolated spikes relative to a 5-sample local median.

    Persistent changes are intentionally retained: when neighboring values
    move with the candidate, the local median follows them.
    """
    bad = set()

    for i, v in enumerate(values):
        if not math.isfinite(v):
            bad.add(i)
            continue

        lo = max(0, i - 2)
        hi = min(len(values), i + 3)
        window = [
            x for x in values[lo:hi]
            if math.isfinite(x)
        ]

        if len(window) < 3:
            continue

        med = statistics.median(window)

        if abs(v - med) > threshold:
            bad.add(i)

    return bad


def isolated_speed_outlier_indices(
    track_rows,
    speed_threshold_mps=2.0,
    error_threshold_mps=2.0,
):
    """
    Build a robust mask for estimator / diagnostic spikes.

    Important logger detail discovered in Tune 05:
      - an anomalous speed_mps sample can appear at time t,
      - the corresponding controller speed_error_mps anomaly can be logged
        one sample later.

    Therefore:
      1. detect isolated spikes independently in speed and speed-error;
      2. combine both masks;
      3. expand each isolated anomaly by one sample on either side.

    This removes single-sample estimator / logger alignment artefacts while
    preserving sustained real overspeed or poor speed tracking.
    """
    speeds = [
        f(r, 'speed_mps')
        for r in track_rows
    ]

    speed_errors = [
        f(r, 'speed_error_mps')
        for r in track_rows
    ]

    bad = (
        isolated_series_outlier_indices(
            speeds,
            speed_threshold_mps,
        )
        | isolated_series_outlier_indices(
            speed_errors,
            error_threshold_mps,
        )
    )

    expanded = set()

    for i in bad:
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(track_rows):
                expanded.add(j)

    return expanded


# =============================================================
# Metrics and scoring
# =============================================================

def compute_metrics(rows):
    track = [r for r in rows if r.get('controller_state') == 'TRACK']

    if not track:
        return None

    cte = [
        f(r, 'cross_track_error_m')
        for r in track
        if math.isfinite(f(r, 'cross_track_error_m'))
    ]
    heading = [
        f(r, 'heading_error_rad')
        for r in track
        if math.isfinite(f(r, 'heading_error_rad'))
    ]

    abs_cte = sorted(abs(x) for x in cte)

    cte_rmse = rmse(cte)
    cte_p95 = percentile_sorted(abs_cte, 0.95)
    max_cte = max(abs_cte) if abs_cte else 99.0
    heading_rmse_deg = math.degrees(rmse(heading))

    # Raw speed metrics are preserved for diagnostics.
    raw_speed_errors = [
        f(r, 'speed_error_mps')
        for r in track
        if math.isfinite(f(r, 'speed_error_mps'))
    ]
    raw_speeds = [
        f(r, 'speed_mps')
        for r in track
        if math.isfinite(f(r, 'speed_mps'))
    ]

    speed_outliers = isolated_speed_outlier_indices(track)

    robust_speed_errors = []
    robust_straight_speeds = []

    for i, row in enumerate(track):
        if i in speed_outliers:
            continue

        serr = f(row, 'speed_error_mps')
        if math.isfinite(serr):
            robust_speed_errors.append(serr)

        if (
            row.get('planner_turn_severity') == 'STRAIGHT'
            and f(row, 'planner_distance_to_turn_m', 0.0) > 12.0
        ):
            speed = f(row, 'speed_mps')
            if math.isfinite(speed):
                robust_straight_speeds.append(speed)

    speed_rmse_robust = rmse(robust_speed_errors)
    speed_rmse_raw = rmse(raw_speed_errors)
    straight_speed = trimmed_mean(robust_straight_speeds, 0.05)

    t_track_end = max(f(r, 'time_s', 0.0) for r in track)

    max_wp = 0
    for r in rows:
        try:
            max_wp = max(
                max_wp,
                int(float(r.get('active_waypoint_number', 0)))
            )
        except Exception:
            pass

    recovery_rows = [
        r for r in rows
        if r.get('controller_state') == 'RECOVER'
    ]

    recoveries = 0
    previous_state = None
    for r in rows:
        state = r.get('controller_state')
        if state == 'RECOVER' and previous_state != 'RECOVER':
            recoveries += 1
        previous_state = state

    return {
        'cte_rmse_m': cte_rmse,
        'cte_p95_m': cte_p95,
        'max_abs_cte_m': max_cte,
        'heading_rmse_deg': heading_rmse_deg,

        'speed_rmse_mps': speed_rmse_robust,
        'speed_rmse_raw_mps': speed_rmse_raw,
        'straight_mean_speed_mps': straight_speed,

        'speed_outlier_count': len(speed_outliers),
        'speed_outlier_fraction': (
            len(speed_outliers) / len(track)
            if track else 0.0
        ),
        'max_speed_raw_mps': (
            max(raw_speeds)
            if raw_speeds else float('nan')
        ),

        'track_time_s': t_track_end,
        'recovery_time_s': 0.1 * len(recovery_rows),
        'recovery_episodes': recoveries,
        'max_waypoint_number': max_wp,
        'track_samples': len(track),
        'rows': len(rows),
    }


def core_score(metrics):
    """
    Score terms that can be evaluated while TRACK is still running.
    No completion/time penalties are included here.

    Used both as part of the final score and for conservative early stopping.
    """
    return (
        W_CTE_RMSE * metrics['cte_rmse_m']
        + W_CTE_P95 * metrics['cte_p95_m']
        + W_MAX_CTE * metrics['max_abs_cte_m']
        + W_HEADING_RMSE_DEG * metrics['heading_rmse_deg']
        + W_SPEED_RMSE * metrics['speed_rmse_mps']
        + W_STRAIGHT_SPEED_SHORTFALL
        * max(
            0.0,
            STRAIGHT_SPEED_TARGET
            - metrics['straight_mean_speed_mps']
        )
    )


def analyse(csv_path: Path):
    rows = read_csv_rows(csv_path)

    if not rows:
        return {
            'success': False,
            'reason': 'NO_DATA',
            'score': 1e6,
        }

    metrics = compute_metrics(rows)

    if metrics is None:
        return {
            'success': False,
            'reason': 'NO_TRACK_DATA',
            'score': 1e6,
        }

    success = any(
        r.get('controller_state') == 'SUCCESS'
        for r in rows
    )

    t_end = max(f(r, 'time_s', 0.0) for r in rows)
    terminal_time = max(
        0.0,
        t_end - metrics['track_time_s']
    )

    score = (
        core_score(metrics)
        + W_TRACK_TIME * metrics['track_time_s']
        + W_TERMINAL_TIME * terminal_time
        + W_RECOVERY_TIME * metrics['recovery_time_s']
        + W_RECOVERY_EPISODE * metrics['recovery_episodes']
    )

    # Completion dominates.
    if not success:
        score += 100.0
        if metrics['max_waypoint_number']:
            score += (
                max(
                    0.0,
                    467.0 - metrics['max_waypoint_number']
                )
                * 0.20
            )

    # Severe path departure never wins just because it is fast.
    if metrics['max_abs_cte_m'] > 6.0:
        score += (
            25.0
            + 5.0 * (
                metrics['max_abs_cte_m'] - 6.0
            )
        )

    result = {
        'success': success,
        'reason': 'SUCCESS' if success else 'NO_SUCCESS',
        'score': score,
        'core_score': core_score(metrics),
        'terminal_time_s': terminal_time,
        **metrics,
    }

    return result


# =============================================================
# Process management
# =============================================================

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
    """
    Clean only processes belonging to this R1 stress-test stack.

    Do NOT run V2/Tune05 and V3 simultaneously; V3 intentionally kills
    stale stress-test children before each trial.
    """
    patterns = [
        'leader_stress_tuning.launch.py',
        'gz sim.*sydney_regatta',
        'platoon_planner.*/stress_course_planner',
        'platoon_control.*/leader_stress_controller',
        'platoon_state.*/multi_vehicle_state',
        'platoon_logging.*/r1_stress_logger',
    ]

    for pattern in patterns:
        subprocess.run(
            ['pkill', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    time.sleep(1.2)


# =============================================================
# Bad-run / live-score logging
# =============================================================

def append_jsonl(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as fobj:
        fobj.write(json.dumps(payload, sort_keys=True) + '\n')


def append_live_score(path: Path, row: dict):
    fields = [
        'sim_time_s',
        'waypoint',
        'partial_core_score',
        'early_stop_threshold',
        'bad_streak',
        'cte_rmse_m',
        'cte_p95_m',
        'max_abs_cte_m',
        'heading_rmse_deg',
        'speed_rmse_mps',
        'straight_mean_speed_mps',
        'speed_outlier_count',
    ]

    new_file = not path.exists()

    with path.open('a', newline='') as fobj:
        writer = csv.DictWriter(fobj, fieldnames=fields)
        if new_file:
            writer.writeheader()
        writer.writerow({k: row.get(k) for k in fields})


# =============================================================
# Conservative continuous early-stop logic
# =============================================================

def early_stop_threshold(
    best_score,
    sim_t,
    min_sim,
    factor_start,
    factor_end,
    margin,
):
    if not math.isfinite(best_score):
        return float('inf')

    # Tighten slowly as more of the course has been observed.
    progress = clamp(
        (sim_t - min_sim) / max(1.0, 210.0 - min_sim),
        0.0,
        1.0,
    )

    factor = (
        factor_start
        + progress * (factor_end - factor_start)
    )

    return best_score * factor + margin


def monitor_trial(
    proc,
    csv_path: Path,
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
):
    """
    Monitor one trial.

    Score-based pruning is intentionally conservative:
      - disabled during startup
      - uses robust speed metrics
      - compares only the partial core score against the best full score
      - requires several consecutive bad checks
    """
    started = time.monotonic()
    last_wp = -1
    last_wp_sim = 0.0
    last_print_sim = -999.0
    last_startup_print = -999.0
    last_score_check_sim = -999.0
    bad_streak = 0
    last_partial = None

    while True:
        wall_elapsed = time.monotonic() - started

        if proc.poll() is not None:
            print(
                f'  [{label}] process exited before SUCCESS',
                flush=True,
            )
            return {
                'reason': 'PROCESS_EXIT',
                'partial': last_partial,
            }

        if wall_elapsed > wall_timeout:
            print(
                f'  [{label}] wall timeout after '
                f'{wall_elapsed:.1f}s',
                flush=True,
            )
            return {
                'reason': 'WALL_TIMEOUT',
                'partial': last_partial,
            }

        rows = read_csv_rows(csv_path)

        if not rows:
            if wall_elapsed - last_startup_print >= 5.0:
                print(
                    f'  [{label}] starting simulation... '
                    f'waiting for logger data '
                    f'({wall_elapsed:.0f}s wall)',
                    flush=True,
                )
                last_startup_print = wall_elapsed

            time.sleep(0.7)
            continue

        recent = rows[-min(120, len(rows)):]
        last = rows[-1]
        sim_t = f(last, 'time_s', 0.0)
        state = last.get('controller_state', '')

        try:
            wp = int(
                float(
                    last.get(
                        'active_waypoint_number',
                        -1,
                    )
                )
            )
        except Exception:
            wp = -1

        cte = f(last, 'cross_track_error_m')
        heading_deg = math.degrees(
            f(last, 'heading_error_rad')
        )
        speed = f(last, 'speed_mps')
        target_speed = f(last, 'target_speed_mps')
        turn = last.get(
            'planner_turn_severity',
            'UNKNOWN',
        )
        dturn = f(
            last,
            'planner_distance_to_turn_m',
        )

        if (
            sim_t - last_print_sim >= 1.0
            or state == 'SUCCESS'
        ):
            cte_txt = (
                f'{cte:6.3f}m'
                if math.isfinite(cte)
                else '   nan'
            )
            hdg_txt = (
                f'{heading_deg:6.2f}deg'
                if math.isfinite(heading_deg)
                else '    nan'
            )
            v_txt = (
                f'{speed:4.2f}'
                if math.isfinite(speed)
                else ' nan'
            )
            vt_txt = (
                f'{target_speed:4.2f}'
                if math.isfinite(target_speed)
                else ' nan'
            )
            dt_txt = (
                f'{dturn:5.1f}m'
                if math.isfinite(dturn)
                else '  nan'
            )

            print(
                f'  sim={sim_t:7.2f}s '
                f'state={state:<8} '
                f'WP={wp:>3} '
                f'CTE={cte_txt} '
                f'head={hdg_txt} '
                f'v={v_txt}/{vt_txt} '
                f'turn={turn:<8} '
                f'd={dt_txt}',
                flush=True,
            )

            last_print_sim = sim_t

        if state == 'SUCCESS':
            return {
                'reason': 'SUCCESS',
                'partial': last_partial,
            }

        # -----------------------------------------------------
        # Hard early stops
        # -----------------------------------------------------

        recent_cte = [
            abs(f(r, 'cross_track_error_m'))
            for r in recent
            if math.isfinite(
                f(r, 'cross_track_error_m')
            )
        ]

        if (
            len(recent_cte) >= 40
            and max(recent_cte) > 6.0
        ):
            print(
                f'  [{label}] early stop: '
                f'path departure '
                f'({max(recent_cte):.2f} m)',
                flush=True,
            )
            return {
                'reason': 'PATH_DEPARTURE',
                'partial': last_partial,
            }

        if wp > last_wp:
            last_wp = wp
            last_wp_sim = sim_t

        elif (
            state == 'TRACK'
            and wp > 1
            and sim_t - last_wp_sim > 35.0
        ):
            print(
                f'  [{label}] early stop: '
                f'waypoint {wp} stalled for '
                f'{sim_t - last_wp_sim:.1f} '
                f'simulated seconds',
                flush=True,
            )
            return {
                'reason': 'WAYPOINT_STALL',
                'partial': last_partial,
            }

        if sim_t > 300.0:
            print(
                f'  [{label}] simulation timeout '
                f'at {sim_t:.1f}s',
                flush=True,
            )
            return {
                'reason': 'SIM_TIMEOUT',
                'partial': last_partial,
            }

        # -----------------------------------------------------
        # Continuous score-based early stop
        # -----------------------------------------------------

        if (
            early_stop_enabled
            and sim_t >= early_min_sim
            and sim_t - last_score_check_sim
            >= early_check_period
            and state == 'TRACK'
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
                    'sim_time_s': sim_t,
                    'waypoint': wp,
                    'partial_core_score': partial_core,
                    'early_stop_threshold': threshold,
                    'bad_streak': bad_streak,
                    **{
                        k: metrics.get(k)
                        for k in [
                            'cte_rmse_m',
                            'cte_p95_m',
                            'max_abs_cte_m',
                            'heading_rmse_deg',
                            'speed_rmse_mps',
                            'straight_mean_speed_mps',
                            'speed_outlier_count',
                        ]
                    },
                }

                append_live_score(
                    live_score_path,
                    last_partial,
                )

                if bad_streak > 0:
                    print(
                        f'  [{label}] partial '
                        f'core={partial_core:.3f} '
                        f'limit={threshold:.3f} '
                        f'bad={bad_streak}/'
                        f'{early_patience}',
                        flush=True,
                    )

                if bad_streak >= early_patience:
                    print(
                        f'  [{label}] early stop: '
                        f'partial score remained '
                        f'noncompetitive '
                        f'({partial_core:.3f} > '
                        f'{threshold:.3f})',
                        flush=True,
                    )
                    return {
                        'reason': 'SCORE_EARLY_STOP',
                        'partial': last_partial,
                    }

            last_score_check_sim = sim_t

        time.sleep(0.7)


# =============================================================
# Trial execution
# =============================================================

def run_trial(
    workspace: Path,
    results_root: Path,
    active_dir: Path,
    bad_runs_path: Path,
    trial_name: str,
    stage_name: str,
    controller_values: dict,
    planner_values: dict,
    wall_timeout: float,
    progress_label: str,
    best_score: float,
    early_stop_args: dict,
):
    run_dir = results_root / trial_name

    if run_dir.exists():
        import shutil
        shutil.rmtree(run_dir)

    run_dir.mkdir(parents=True, exist_ok=True)

    controller_yaml = (
        active_dir
        / f'{trial_name}_controller.yaml'
    )
    planner_yaml = (
        active_dir
        / f'{trial_name}_planner.yaml'
    )

    write_yaml(
        controller_yaml,
        'leader_pid_controller',
        controller_values,
    )
    write_yaml(
        planner_yaml,
        'trajectory_planner',
        planner_values,
    )

    log_path = run_dir / 'launch.log'
    csv_path = run_dir / 'r1_stress.csv'
    live_score_path = run_dir / 'live_score.csv'

    cleanup_leftovers()

    env = os.environ.copy()

    setup = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {workspace}/install/setup.bash && "
        f"exec ros2 launch "
        f"platoon_bringup "
        f"leader_stress_tuning.launch.py "
        f"run_name:={trial_name} "
        f"results_root:={results_root} "
        f"controller_params_file:={controller_yaml} "
        f"planner_params_file:={planner_yaml}"
    )

    with log_path.open('w') as log:
        proc = subprocess.Popen(
            ['bash', '-lc', setup],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            preexec_fn=os.setsid,
            text=True,
        )

        monitor = monitor_trial(
            proc=proc,
            csv_path=csv_path,
            live_score_path=live_score_path,
            wall_timeout=wall_timeout,
            label=progress_label,
            best_score=best_score,
            **early_stop_args,
        )

        terminate_group(proc)

    reason = monitor['reason']

    if reason != 'SUCCESS':
        try:
            log_lines = (
                log_path
                .read_text(errors='replace')
                .splitlines()
            )
            if log_lines:
                print(
                    '  ---- last launch-log lines ----',
                    flush=True,
                )
                for line in log_lines[-12:]:
                    print(
                        f'  {line}',
                        flush=True,
                    )
                print(
                    '  --------------------------------',
                    flush=True,
                )
        except Exception:
            pass

    time.sleep(1.0)

    result = analyse(csv_path)
    result['termination_reason'] = reason
    result['trial_name'] = trial_name
    result['stage'] = stage_name
    result['controller'] = controller_values
    result['planner'] = planner_values
    result['partial_at_stop'] = monitor.get('partial')

    (
        run_dir
        / 'trial_result.json'
    ).write_text(
        json.dumps(result, indent=2)
    )

    if reason != 'SUCCESS':
        append_jsonl(
            bad_runs_path,
            {
                'trial_name': trial_name,
                'stage': stage_name,
                'termination_reason': reason,
                'final_score': result.get('score'),
                'partial_at_stop': monitor.get('partial'),
                'controller': controller_values,
                'planner': planner_values,
            },
        )

    return result


# =============================================================
# Search spaces
# =============================================================

def guidance_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    controller['heading_kp'] = trial.suggest_float(
        'heading_kp',
        350.0,
        1450.0,
    )
    controller['heading_ki'] = trial.suggest_float(
        'heading_ki',
        0.0,
        42.0,
    )
    controller['heading_kd'] = trial.suggest_float(
        'heading_kd',
        35.0,
        190.0,
    )
    controller['heading_slowdown_angle'] = (
        trial.suggest_float(
            'heading_slowdown_angle',
            0.35,
            1.05,
        )
    )

    planner['minimum_turn_speed'] = (
        trial.suggest_float(
            'minimum_turn_speed',
            0.42,
            0.95,
        )
    )
    planner['lateral_accel_limit'] = (
        trial.suggest_float(
            'lateral_accel_limit',
            0.16,
            0.58,
        )
    )
    planner['lookahead_min'] = (
        trial.suggest_float(
            'lookahead_min',
            1.1,
            3.0,
        )
    )
    planner['tight_lookahead'] = (
        trial.suggest_float(
            'tight_lookahead',
            1.2,
            3.2,
        )
    )
    planner['medium_lookahead'] = (
        trial.suggest_float(
            'medium_lookahead',
            2.0,
            4.6,
        )
    )
    planner['lookahead_speed_gain'] = (
        trial.suggest_float(
            'lookahead_speed_gain',
            0.5,
            2.7,
        )
    )

    planner['tight_lookahead'] = min(
        planner['tight_lookahead'],
        planner['medium_lookahead'],
    )
    planner['lookahead_max'] = max(
        5.0,
        planner['medium_lookahead'] + 1.0,
        planner['lookahead_min'] + 2.0,
    )

    return controller, planner


def speed_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    controller['speed_kp'] = trial.suggest_float(
        'speed_kp',
        70.0,
        360.0,
    )
    controller['speed_ki'] = trial.suggest_float(
        'speed_ki',
        0.0,
        45.0,
    )
    controller['speed_kd'] = trial.suggest_float(
        'speed_kd',
        5.0,
        110.0,
    )

    planner['accel_limit'] = trial.suggest_float(
        'accel_limit',
        0.25,
        0.90,
    )
    planner['decel_limit'] = trial.suggest_float(
        'decel_limit',
        0.30,
        1.00,
    )

    return controller, planner


def brake_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    controller['brake_kp'] = trial.suggest_float(
        'brake_kp',
        100.0,
        650.0,
    )
    controller['brake_ki'] = trial.suggest_float(
        'brake_ki',
        0.0,
        90.0,
    )
    controller['brake_kd'] = trial.suggest_float(
        'brake_kd',
        0.0,
        40.0,
    )

    controller['recovery_target_speed'] = (
        trial.suggest_float(
            'recovery_target_speed',
            0.12,
            0.38,
        )
    )
    controller['recovery_brake_distance'] = (
        trial.suggest_float(
            'recovery_brake_distance',
            0.55,
            0.88,
        )
    )

    return controller, planner


def joint_space(trial, controller, planner):
    """
    Local consolidation around the CURRENT global best.

    Unlike V2, joint tuning includes integral terms and several guidance/speed
    coupling variables instead of only a small subset.
    """
    controller = dict(controller)
    planner = dict(planner)

    def around(name, current, frac, lo, hi):
        low = clamp(
            current * (1.0 - frac),
            lo,
            hi,
        )
        high = clamp(
            current * (1.0 + frac),
            lo,
            hi,
        )

        if high <= low + 1e-9:
            return current

        return trial.suggest_float(
            name,
            low,
            high,
        )

    controller['heading_kp'] = around(
        'heading_kp',
        controller['heading_kp'],
        0.18,
        300.0,
        1600.0,
    )
    controller['heading_ki'] = around(
        'heading_ki',
        max(controller['heading_ki'], 0.5),
        0.25,
        0.0,
        55.0,
    )
    controller['heading_kd'] = around(
        'heading_kd',
        controller['heading_kd'],
        0.20,
        20.0,
        220.0,
    )
    controller['heading_slowdown_angle'] = around(
        'heading_slowdown_angle',
        controller['heading_slowdown_angle'],
        0.16,
        0.25,
        1.20,
    )

    controller['speed_kp'] = around(
        'speed_kp',
        controller['speed_kp'],
        0.20,
        50.0,
        420.0,
    )
    controller['speed_ki'] = around(
        'speed_ki',
        max(controller['speed_ki'], 0.5),
        0.25,
        0.0,
        55.0,
    )
    controller['speed_kd'] = around(
        'speed_kd',
        controller['speed_kd'],
        0.25,
        0.0,
        140.0,
    )

    planner['minimum_turn_speed'] = around(
        'minimum_turn_speed',
        planner['minimum_turn_speed'],
        0.15,
        0.38,
        1.00,
    )
    planner['lateral_accel_limit'] = around(
        'lateral_accel_limit',
        planner['lateral_accel_limit'],
        0.18,
        0.12,
        0.70,
    )
    planner['lookahead_min'] = around(
        'lookahead_min',
        planner['lookahead_min'],
        0.15,
        1.0,
        3.2,
    )
    planner['tight_lookahead'] = around(
        'tight_lookahead',
        planner['tight_lookahead'],
        0.18,
        1.0,
        3.5,
    )
    planner['medium_lookahead'] = around(
        'medium_lookahead',
        planner['medium_lookahead'],
        0.15,
        1.8,
        4.8,
    )
    planner['lookahead_speed_gain'] = around(
        'lookahead_speed_gain',
        planner['lookahead_speed_gain'],
        0.18,
        0.35,
        3.0,
    )

    planner['accel_limit'] = around(
        'accel_limit',
        planner['accel_limit'],
        0.18,
        0.20,
        1.00,
    )
    planner['decel_limit'] = around(
        'decel_limit',
        planner['decel_limit'],
        0.20,
        0.25,
        1.20,
    )

    planner['tight_lookahead'] = min(
        planner['tight_lookahead'],
        planner['medium_lookahead'],
    )
    planner['lookahead_max'] = max(
        5.0,
        planner['medium_lookahead'] + 1.0,
        planner['lookahead_min'] + 2.0,
    )

    return controller, planner


SPACE_BY_STAGE = {
    'guidance': guidance_space,
    'speed': speed_space,
    'joint': joint_space,
    'brake': brake_space,
}


# =============================================================
# Cyclic stage plan
# =============================================================

def stage_counts_v3(total):
    if total < 12:
        # Still keep braking last and guidance dominant.
        brake = 1
        speed = 1
        joint = max(1, round(total * 0.30))
        guidance = max(
            1,
            total - brake - speed - joint,
        )
        return {
            'guidance': guidance,
            'speed': speed,
            'joint': joint,
            'brake': brake,
        }

    # 36 -> G15 / S6 / J12 / B3
    brake = max(2, round(total * 0.08))
    guidance = max(4, round(total * 0.42))
    speed = max(2, round(total * 0.17))
    joint = total - brake - guidance - speed

    if joint < 2:
        deficit = 2 - joint
        guidance = max(4, guidance - deficit)
        joint = total - brake - guidance - speed

    return {
        'guidance': guidance,
        'speed': speed,
        'joint': joint,
        'brake': brake,
    }


def build_stage_plan(total):
    counts = stage_counts_v3(total)

    g = counts['guidance']
    s = counts['speed']
    j = counts['joint']
    b = counts['brake']

    plan = []

    # Cyclic refinement. For 36:
    # G4 S2 J3 | G4 S2 J3 | G4 S2 J3 | G3 J3 | B3
    while g > 0 or s > 0 or j > 0:
        if g > 0:
            n = min(4, g)
            plan.extend(['guidance'] * n)
            g -= n

        if s > 0:
            n = min(2, s)
            plan.extend(['speed'] * n)
            s -= n

        if j > 0:
            n = min(3, j)
            plan.extend(['joint'] * n)
            j -= n

    # Brakes intentionally last.
    plan.extend(['brake'] * b)

    # Defensive correction.
    return plan[:total]


def compact_plan(plan):
    if not plan:
        return ''

    short = {
        'guidance': 'G',
        'speed': 'S',
        'joint': 'J',
        'brake': 'B',
    }

    blocks = []
    current = plan[0]
    count = 1

    for stage in plan[1:]:
        if stage == current:
            count += 1
        else:
            blocks.append(
                f'{short[current]}{count}'
            )
            current = stage
            count = 1

    blocks.append(
        f'{short[current]}{count}'
    )

    return ' -> '.join(blocks)


# =============================================================
# Checkpoint / history
# =============================================================

HISTORY_FIELDS = [
    'stage',
    'trial',
    'score',
    'success',
    'core_score',

    'cte_rmse_m',
    'cte_p95_m',
    'max_abs_cte_m',
    'heading_rmse_deg',

    'speed_rmse_mps',
    'speed_rmse_raw_mps',
    'speed_outlier_count',
    'speed_outlier_fraction',
    'max_speed_raw_mps',
    'straight_mean_speed_mps',

    'track_time_s',
    'terminal_time_s',
    'recovery_time_s',
    'recovery_episodes',

    'termination_reason',
    'trial_name',
]


def ensure_history(path: Path):
    if path.exists() and path.stat().st_size > 0:
        return

    with path.open('w', newline='') as fobj:
        csv.DictWriter(
            fobj,
            fieldnames=HISTORY_FIELDS,
        ).writeheader()


def append_history(path: Path, row: dict):
    with path.open('a', newline='') as fobj:
        csv.DictWriter(
            fobj,
            fieldnames=HISTORY_FIELDS,
        ).writerow(
            {
                k: row.get(k)
                for k in HISTORY_FIELDS
            }
        )


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
        'version': 3,
        'seed_done': seed_done,
        'next_schedule_index': next_schedule_index,
        'completed_optimization_trials': completed_optimization_trials,
        'best_score': best_score,
        'best_controller': best_controller,
        'best_planner': best_planner,
        'best_result': best_result,
    }

    tmp = path.with_suffix('.tmp')
    tmp.write_text(
        json.dumps(
            payload,
            indent=2,
        )
    )
    tmp.replace(path)


# =============================================================
# Main
# =============================================================

def main():
    parser = argparse.ArgumentParser(
        description=(
            'V3 cyclic, robust, resumable R1 stress-course autotuner.'
        )
    )

    parser.add_argument(
        '--trials',
        type=int,
        default=36,
        help='optimization trials, excluding the seed validation run',
    )
    parser.add_argument(
        '--wall-timeout',
        type=float,
        default=260.0,
    )
    parser.add_argument(
        '--study-name',
        default='r1_stress_tune_06',
    )
    parser.add_argument(
        '--workspace',
        default=str(
            Path.home() / 'vrx_ws'
        ),
    )

    parser.add_argument(
        '--seed-controller',
        default=None,
    )
    parser.add_argument(
        '--seed-planner',
        default=None,
    )

    parser.add_argument(
        '--resume',
        action='store_true',
        help='resume from checkpoint.json in the existing study directory',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='delete an existing V3 study directory and start clean',
    )

    parser.add_argument(
        '--no-early-stop',
        action='store_true',
        help='disable continuous score-based early stopping',
    )
    parser.add_argument(
        '--early-min-sim',
        type=float,
        default=65.0,
        help='do not score-prune before this simulated time',
    )
    parser.add_argument(
        '--early-patience',
        type=int,
        default=3,
        help='consecutive bad score checks required before stopping',
    )
    parser.add_argument(
        '--early-check-period',
        type=float,
        default=5.0,
        help='simulated seconds between partial-score checks',
    )
    parser.add_argument(
        '--early-factor-start',
        type=float,
        default=1.55,
        help='early partial-score multiplier versus current best',
    )
    parser.add_argument(
        '--early-factor-end',
        type=float,
        default=1.25,
        help='late partial-score multiplier versus current best',
    )
    parser.add_argument(
        '--early-margin',
        type=float,
        default=0.75,
        help='absolute score margin added to early-stop threshold',
    )

    args = parser.parse_args()

    if args.trials < 1:
        raise SystemExit('--trials must be >= 1')

    if args.resume and args.overwrite:
        raise SystemExit(
            'Use either --resume or --overwrite, not both.'
        )

    workspace = Path(
        args.workspace
    ).expanduser()

    project = (
        workspace
        / 'src/vrx_platooning'
    )

    out = (
        project
        / 'results/autotune_r1_stress'
        / args.study_name
    )

    checkpoint_path = out / 'checkpoint.json'
    history_path = out / 'history.csv'
    bad_runs_path = out / 'bad_runs.jsonl'
    storage_path = out / 'optuna_v3.sqlite3'

    # ---------------------------------------------------------
    # Directory mode
    # ---------------------------------------------------------

    if out.exists() and any(out.iterdir()):
        if args.overwrite:
            import shutil
            shutil.rmtree(out)

        elif not args.resume:
            raise SystemExit(
                'ERROR: study directory already exists and is not empty:\n'
                f'  {out}\n'
                'Use --resume to continue it, or --overwrite deliberately.'
            )

    out.mkdir(
        parents=True,
        exist_ok=True,
    )

    trials_root = out / 'trials'
    active_dir = out / 'active_configs'

    trials_root.mkdir(
        parents=True,
        exist_ok=True,
    )
    active_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    ensure_history(history_path)

    plan = build_stage_plan(
        args.trials
    )

    # ---------------------------------------------------------
    # Restore or initialize checkpoint
    # ---------------------------------------------------------

    if args.resume:
        if not checkpoint_path.exists():
            raise SystemExit(
                'Cannot resume: checkpoint.json is missing:\n'
                f'  {checkpoint_path}'
            )

        checkpoint = json.loads(
            checkpoint_path.read_text()
        )

        seed_done = bool(
            checkpoint.get(
                'seed_done',
                False,
            )
        )
        next_schedule_index = int(
            checkpoint.get(
                'next_schedule_index',
                0,
            )
        )
        completed_optimization_trials = int(
            checkpoint.get(
                'completed_optimization_trials',
                next_schedule_index,
            )
        )

        controller_best = dict(
            checkpoint['best_controller']
        )
        planner_best = dict(
            checkpoint['best_planner']
        )
        best_global_score = float(
            checkpoint['best_score']
        )
        best_global_result = (
            checkpoint.get(
                'best_result'
            )
        )

    else:
        seed_done = False
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
                args.seed_controller,
                'leader_pid_controller',
            )
        )
        planner_best.update(
            load_seed_yaml(
                args.seed_planner,
                'trajectory_planner',
            )
        )

        best_global_score = float('inf')
        best_global_result = None

        write_checkpoint(
            checkpoint_path,
            seed_done=seed_done,
            next_schedule_index=next_schedule_index,
            completed_optimization_trials=completed_optimization_trials,
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

    early_stop_args = {
        'early_stop_enabled': (
            not args.no_early_stop
        ),
        'early_min_sim': args.early_min_sim,
        'early_patience': args.early_patience,
        'early_check_period': args.early_check_period,
        'early_factor_start': args.early_factor_start,
        'early_factor_end': args.early_factor_end,
        'early_margin': args.early_margin,
    }

    # ---------------------------------------------------------
    # Single Optuna study across the cyclic schedule.
    #
    # group=True is useful because stages expose conditional subsets
    # of the overall parameter space.
    # ---------------------------------------------------------

    sampler = optuna.samplers.TPESampler(
        seed=42,
        n_startup_trials=6,
        multivariate=True,
        group=True,
        warn_independent_sampling=False,
    )

    study = optuna.create_study(
        study_name=f'{args.study_name}_v3',
        storage=f'sqlite:///{storage_path}',
        direction='minimize',
        sampler=sampler,
        load_if_exists=True,
    )

    print()
    print('============================================================')
    print(' R1 STRESS AUTOTUNER V3')
    print('============================================================')
    print(f'Study       : {args.study_name}')
    print(f'Trials      : {args.trials} optimization + seed')
    print(f'Plan        : {compact_plan(plan)}')
    print(f'Counts      : {stage_counts_v3(args.trials)}')
    print('Mode        : LIGHT / HEADLESS / NO RVIZ')
    print('Sampler     : Optuna TPE | multivariate=True | group=True')
    print(
        'Scoring     : guidance-dominant + robust isolated-speed filter'
    )
    print(
        'Early stop  : '
        f'{"OFF" if args.no_early_stop else "ON"} '
        f'(min_sim={args.early_min_sim:.0f}s, '
        f'patience={args.early_patience})'
    )
    print(f'Resume      : {"YES" if args.resume else "NO"}')
    print(f'Output      : {out}')
    print('============================================================')
    print()

    # ---------------------------------------------------------
    # Seed validation
    # ---------------------------------------------------------

    if not seed_done:
        seed_name = f'{args.study_name}_seed'

        print(
            f'[seed] {seed_name}',
            flush=True,
        )

        seed = run_trial(
            workspace=workspace,
            results_root=trials_root,
            active_dir=active_dir,
            bad_runs_path=bad_runs_path,
            trial_name=seed_name,
            stage_name='seed',
            controller_values=controller_best,
            planner_values=planner_best,
            wall_timeout=args.wall_timeout,
            progress_label='seed',
            best_score=float('inf'),
            early_stop_args={
                **early_stop_args,
                # Never score-prune the reference seed.
                'early_stop_enabled': False,
            },
        )

        if not seed['success']:
            raise SystemExit(
                'Seed validation did not complete successfully. '
                'Do not tune around an unvalidated seed. '
                f"Reason={seed.get('termination_reason')}"
            )

        best_global_score = seed['score']
        best_global_result = seed

        seed_row = {
            **seed,
            'stage': 'seed',
            'trial': 0,
        }
        append_history(
            history_path,
            seed_row,
        )

        write_yaml(
            out / 'best_controller.yaml',
            'leader_pid_controller',
            controller_best,
        )
        write_yaml(
            out / 'best_planner.yaml',
            'trajectory_planner',
            planner_best,
        )
        (
            out
            / 'best_trial_result.json'
        ).write_text(
            json.dumps(
                best_global_result,
                indent=2,
            )
        )

        seed_done = True

        write_checkpoint(
            checkpoint_path,
            seed_done=True,
            next_schedule_index=next_schedule_index,
            completed_optimization_trials=completed_optimization_trials,
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

        print(
            f"  seed score={seed['score']:.3f} "
            f"cte={seed['cte_rmse_m']:.3f}m "
            f"max={seed['max_abs_cte_m']:.3f}m "
            f"heading={seed['heading_rmse_deg']:.2f}deg "
            f"speed={seed['speed_rmse_mps']:.3f}m/s "
            f"speed_outliers={seed['speed_outlier_count']}",
            flush=True,
        )

    else:
        print(
            f'[resume] current best score='
            f'{best_global_score:.3f}; '
            f'next optimization trial='
            f'{next_schedule_index + 1}',
            flush=True,
        )

    # ---------------------------------------------------------
    # Cyclic optimization
    # ---------------------------------------------------------

    previous_stage = None

    for schedule_index in range(
        next_schedule_index,
        len(plan),
    ):
        stage_name = plan[schedule_index]

        if stage_name != previous_stage:
            print()
            print(
                f'===== {stage_name.upper()} BLOCK '
                f'(trial {schedule_index + 1}/{args.trials}) =====',
                flush=True,
            )
            previous_stage = stage_name

        # Every trial starts from the current global best.
        fixed_controller = dict(
            controller_best
        )
        fixed_planner = dict(
            planner_best
        )

        optuna_trial = study.ask()

        space_fn = SPACE_BY_STAGE[
            stage_name
        ]

        controller, planner = space_fn(
            optuna_trial,
            fixed_controller,
            fixed_planner,
        )

        global_trial_number = (
            schedule_index + 1
        )

        trial_name = (
            f'{args.study_name}_'
            f'{global_trial_number:03d}_'
            f'{stage_name}'
        )

        print(
            f'[{global_trial_number:03d}/'
            f'{args.trials}] '
            f'{stage_name}: '
            f'{trial_name}',
            flush=True,
        )

        changed = {
            **{
                k: v
                for k, v in controller.items()
                if fixed_controller.get(k) != v
            },
            **{
                f'planner.{k}': v
                for k, v in planner.items()
                if fixed_planner.get(k) != v
            },
        }

        if changed:
            print(
                '  sampled: '
                + ', '.join(
                    (
                        f'{k}={v:.4g}'
                        if isinstance(v, float)
                        else f'{k}={v}'
                    )
                    for k, v
                    in changed.items()
                ),
                flush=True,
            )

        result = run_trial(
            workspace=workspace,
            results_root=trials_root,
            active_dir=active_dir,
            bad_runs_path=bad_runs_path,
            trial_name=trial_name,
            stage_name=stage_name,
            controller_values=controller,
            planner_values=planner,
            wall_timeout=args.wall_timeout,
            progress_label=(
                f'{global_trial_number:03d}/'
                f'{args.trials} '
                f'{stage_name}'
            ),
            best_score=best_global_score,
            early_stop_args=early_stop_args,
        )

        optuna_trial.set_user_attr(
            'stage',
            stage_name,
        )
        optuna_trial.set_user_attr(
            'controller',
            controller,
        )
        optuna_trial.set_user_attr(
            'planner',
            planner,
        )
        optuna_trial.set_user_attr(
            'termination_reason',
            result.get(
                'termination_reason'
            ),
        )
        optuna_trial.set_user_attr(
            'result',
            {
                k: v
                for k, v in result.items()
                if k not in (
                    'controller',
                    'planner',
                )
            },
        )

        # Tell Optuna even for an early-stopped candidate.
        # Failed/non-successful runs already receive the completion penalty,
        # so the sampler learns that this region is poor.
        study.tell(
            optuna_trial,
            result['score'],
        )

        print(
            f"  score={result['score']:.3f} "
            f"success={result['success']} "
            f"cte={result.get('cte_rmse_m', float('nan')):.3f}m "
            f"p95={result.get('cte_p95_m', float('nan')):.3f}m "
            f"max={result.get('max_abs_cte_m', float('nan')):.3f}m "
            f"heading={result.get('heading_rmse_deg', float('nan')):.2f}deg "
            f"speed={result.get('speed_rmse_mps', float('nan')):.3f} "
            f"(raw={result.get('speed_rmse_raw_mps', float('nan')):.3f}) "
            f"outliers={result.get('speed_outlier_count', 0)} "
            f"straight_v={result.get('straight_mean_speed_mps', float('nan')):.2f} "
            f"stop={result.get('termination_reason')}",
            flush=True,
        )

        print(
            f'  current best score='
            f'{best_global_score:.3f}',
            flush=True,
        )

        append_history(
            history_path,
            {
                **result,
                'stage': stage_name,
                'trial': global_trial_number,
            },
        )

        if (
            result['success']
            and result['score']
            < best_global_score
        ):
            best_global_score = (
                result['score']
            )
            best_global_result = result

            controller_best = dict(
                controller
            )
            planner_best = dict(
                planner
            )

            write_yaml(
                out / 'best_controller.yaml',
                'leader_pid_controller',
                controller_best,
            )
            write_yaml(
                out / 'best_planner.yaml',
                'trajectory_planner',
                planner_best,
            )
            (
                out
                / 'best_trial_result.json'
            ).write_text(
                json.dumps(
                    best_global_result,
                    indent=2,
                )
            )

            print(
                '  >>> NEW GLOBAL BEST',
                flush=True,
            )

        completed_optimization_trials = (
            schedule_index + 1
        )
        next_schedule_index = (
            schedule_index + 1
        )

        write_checkpoint(
            checkpoint_path,
            seed_done=True,
            next_schedule_index=next_schedule_index,
            completed_optimization_trials=completed_optimization_trials,
            best_score=best_global_score,
            best_controller=controller_best,
            best_planner=planner_best,
            best_result=best_global_result,
        )

    # ---------------------------------------------------------
    # Final outputs
    # ---------------------------------------------------------

    write_yaml(
        out / 'best_controller.yaml',
        'leader_pid_controller',
        controller_best,
    )
    write_yaml(
        out / 'best_planner.yaml',
        'trajectory_planner',
        planner_best,
    )

    summary = {
        'version': 3,
        'study_name': args.study_name,
        'requested_trials': args.trials,
        'stage_plan': plan,
        'stage_plan_compact': compact_plan(plan),
        'stage_counts': stage_counts_v3(args.trials),
        'sampler': {
            'name': 'TPESampler',
            'multivariate': True,
            'group': True,
            'n_startup_trials': 6,
        },
        'scoring': {
            'guidance_dominant': True,
            'isolated_speed_outlier_filter': True,
            'weights': {
                'cte_rmse': W_CTE_RMSE,
                'cte_p95': W_CTE_P95,
                'max_cte': W_MAX_CTE,
                'heading_rmse_deg': W_HEADING_RMSE_DEG,
                'speed_rmse': W_SPEED_RMSE,
                'straight_speed_shortfall': W_STRAIGHT_SPEED_SHORTFALL,
                'track_time': W_TRACK_TIME,
                'terminal_time': W_TERMINAL_TIME,
                'recovery_time': W_RECOVERY_TIME,
                'recovery_episode': W_RECOVERY_EPISODE,
            },
        },
        'early_stop': early_stop_args,
        'best_score': best_global_score,
        'best_controller': controller_best,
        'best_planner': planner_best,
        'best_result': best_global_result,
    }

    (
        out
        / 'summary.json'
    ).write_text(
        json.dumps(
            summary,
            indent=2,
        )
    )

    print()
    print('============================================================')
    print(' AUTOTUNING V3 COMPLETE')
    print('============================================================')
    print(
        f'Best score       : '
        f'{best_global_score:.3f}'
    )

    if best_global_result:
        print(
            f"CTE RMSE         : "
            f"{best_global_result.get('cte_rmse_m', float('nan')):.3f} m"
        )
        print(
            f"CTE P95          : "
            f"{best_global_result.get('cte_p95_m', float('nan')):.3f} m"
        )
        print(
            f"Max |CTE|        : "
            f"{best_global_result.get('max_abs_cte_m', float('nan')):.3f} m"
        )
        print(
            f"Heading RMSE     : "
            f"{best_global_result.get('heading_rmse_deg', float('nan')):.2f} deg"
        )
        print(
            f"Speed RMSE       : "
            f"{best_global_result.get('speed_rmse_mps', float('nan')):.3f} m/s"
        )
        print(
            f"Raw speed RMSE   : "
            f"{best_global_result.get('speed_rmse_raw_mps', float('nan')):.3f} m/s"
        )
        print(
            f"Speed outliers   : "
            f"{best_global_result.get('speed_outlier_count', 0)}"
        )
        print(
            f"Straight speed   : "
            f"{best_global_result.get('straight_mean_speed_mps', float('nan')):.3f} m/s"
        )

    print(
        f'Controller YAML  : '
        f'{out / "best_controller.yaml"}'
    )
    print(
        f'Planner YAML     : '
        f'{out / "best_planner.yaml"}'
    )
    print(
        f'History          : '
        f'{history_path}'
    )
    print(
        f'Bad runs         : '
        f'{bad_runs_path}'
    )
    print(
        f'Checkpoint       : '
        f'{checkpoint_path}'
    )
    print(
        f'Summary          : '
        f'{out / "summary.json"}'
    )
    print('============================================================')


if __name__ == '__main__':
    main()
