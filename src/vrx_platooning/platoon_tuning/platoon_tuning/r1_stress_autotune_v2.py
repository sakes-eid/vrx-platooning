#!/usr/bin/env python3
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

import yaml

try:
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
except Exception as exc:
    raise SystemExit(
        "Optuna is required. It was already used by the earlier R1 tuner; "
        "if this environment cannot import it, run: python3 -c 'import optuna; print(optuna.__version__)'"
    ) from exc


BASE_CONTROLLER = {
    # Last repeatedly validated R1 gains from the earlier simple-course tuner.
    'heading_kp': 911.4007441588406,
    'heading_ki': 25.65165107782674,
    'heading_kd': 98.10493190966012,
    'speed_kp': 127.83877941647718,
    'speed_ki': 18.383775431748894,
    'speed_kd': 49.388692087475725,
    'brake_kp': 177.8935198208073,
    'brake_ki': 60.634764417119634,
    'brake_kd': 10.647182804717701,

    # Preserve proven controller structure / safety limits.
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
    # Geometry: NEVER tune the test itself.
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


def write_yaml(path: Path, node_name: str, values: dict):
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


def read_csv_rows(path: Path):
    if not path.exists() or path.stat().st_size < 40:
        return []
    try:
        with path.open(newline='') as f:
            return list(csv.DictReader(f))
    except Exception:
        return []


def f(row, key, default=float('nan')):
    try:
        return float(row.get(key, default))
    except Exception:
        return default


def analyse(csv_path: Path):
    rows = read_csv_rows(csv_path)
    if not rows:
        return {
            'success': False,
            'reason': 'NO_DATA',
            'score': 1e6,
        }

    track = [r for r in rows if r.get('controller_state') == 'TRACK']
    if not track:
        return {
            'success': False,
            'reason': 'NO_TRACK_DATA',
            'score': 1e6,
        }

    success = any(r.get('controller_state') == 'SUCCESS' for r in rows)

    def finite_values(key, source):
        out = []
        for r in source:
            v = f(r, key)
            if math.isfinite(v):
                out.append(v)
        return out

    cte = finite_values('cross_track_error_m', track)
    heading = finite_values('heading_error_rad', track)
    speed_error = finite_values('speed_error_mps', track)
    speeds = finite_values('speed_mps', track)

    def rmse(values):
        if not values:
            return 99.0
        return math.sqrt(sum(v * v for v in values) / len(values))

    abs_cte = sorted(abs(x) for x in cte)
    p95 = (
        abs_cte[min(len(abs_cte) - 1, int(0.95 * len(abs_cte)))]
        if abs_cte else 99.0
    )
    max_cte = max(abs_cte) if abs_cte else 99.0
    cte_rmse = rmse(cte)
    heading_rmse_deg = math.degrees(rmse(heading))
    speed_rmse = rmse(speed_error)

    straight = [
        r for r in track
        if r.get('planner_turn_severity') == 'STRAIGHT'
        and f(r, 'planner_distance_to_turn_m', 0.0) > 12.0
    ]
    straight_speeds = finite_values('speed_mps', straight)
    straight_mean_speed = (
        sum(straight_speeds) / len(straight_speeds)
        if straight_speeds else 0.0
    )

    t_track_end = max(f(r, 'time_s', 0.0) for r in track)
    t_end = max(f(r, 'time_s', 0.0) for r in rows)
    terminal_time = max(0.0, t_end - t_track_end)

    recovery_rows = [r for r in rows if r.get('controller_state') == 'RECOVER']
    recovery_time = 0.1 * len(recovery_rows)

    # Count RECOVER episodes.
    recoveries = 0
    prev = None
    for r in rows:
        state = r.get('controller_state')
        if state == 'RECOVER' and prev != 'RECOVER':
            recoveries += 1
        prev = state

    max_wp = 0
    if 'active_waypoint_number' in rows[0]:
        for r in rows:
            try:
                max_wp = max(max_wp, int(float(r.get('active_waypoint_number', 0))))
            except Exception:
                pass

    # Primary goal: accurate path following. Speed is secondary.
    score = (
        4.5 * cte_rmse
        + 1.5 * p95
        + 0.60 * max_cte
        + 0.035 * heading_rmse_deg
        + 0.90 * speed_rmse
        + 1.20 * max(0.0, 1.75 - straight_mean_speed)
        + 0.0035 * t_track_end
        + 0.025 * terminal_time
        + 0.10 * recovery_time
        + 0.35 * recoveries
    )

    # Course completion dominates everything.
    if not success:
        score += 100.0
        if max_wp:
            score += max(0.0, 467.0 - max_wp) * 0.20

    # Severe path departure should never win just because it is fast.
    if max_cte > 6.0:
        score += 25.0 + 5.0 * (max_cte - 6.0)

    return {
        'success': success,
        'reason': 'SUCCESS' if success else 'NO_SUCCESS',
        'score': score,
        'cte_rmse_m': cte_rmse,
        'cte_p95_m': p95,
        'max_abs_cte_m': max_cte,
        'heading_rmse_deg': heading_rmse_deg,
        'speed_rmse_mps': speed_rmse,
        'straight_mean_speed_mps': straight_mean_speed,
        'track_time_s': t_track_end,
        'terminal_time_s': terminal_time,
        'recovery_time_s': recovery_time,
        'recovery_episodes': recoveries,
        'max_waypoint_number': max_wp,
        'rows': len(rows),
    }


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
    # Narrow patterns: only this tuning launch / Sydney gz process.
    subprocess.run(
        ['pkill', '-f', 'leader_stress_tuning.launch.py'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ['pkill', '-f', 'gz sim.*sydney_regatta'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(1.2)


def monitor_trial(proc, csv_path: Path, wall_timeout: float, label: str):
    """Monitor one trial and print live simulation progress to the terminal."""
    started = time.monotonic()
    last_wp = -1
    last_wp_sim = 0.0
    last_print_sim = -999.0
    last_startup_print = -999.0

    while True:
        wall_elapsed = time.monotonic() - started

        if proc.poll() is not None:
            print(f'  [{label}] process exited before SUCCESS', flush=True)
            return 'PROCESS_EXIT'

        if wall_elapsed > wall_timeout:
            print(
                f'  [{label}] wall timeout after {wall_elapsed:.1f}s',
                flush=True,
            )
            return 'WALL_TIMEOUT'

        rows = read_csv_rows(csv_path)
        if not rows:
            if wall_elapsed - last_startup_print >= 5.0:
                print(
                    f'  [{label}] starting simulation... '
                    f'waiting for logger data ({wall_elapsed:.0f}s wall)',
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
            wp = int(float(last.get('active_waypoint_number', -1)))
        except Exception:
            wp = -1

        cte = f(last, 'cross_track_error_m')
        heading_deg = math.degrees(f(last, 'heading_error_rad'))
        speed = f(last, 'speed_mps')
        target_speed = f(last, 'target_speed_mps')
        turn = last.get('planner_turn_severity', 'UNKNOWN')
        dturn = f(last, 'planner_distance_to_turn_m')

        # Print approximately once per simulated second, similar to the
        # earlier smart tuner.
        if sim_t - last_print_sim >= 1.0 or state == 'SUCCESS':
            cte_txt = f'{cte:6.3f}m' if math.isfinite(cte) else '   nan'
            hdg_txt = (
                f'{heading_deg:6.2f}deg'
                if math.isfinite(heading_deg) else '    nan'
            )
            v_txt = f'{speed:4.2f}' if math.isfinite(speed) else ' nan'
            vt_txt = (
                f'{target_speed:4.2f}'
                if math.isfinite(target_speed) else ' nan'
            )
            dt_txt = f'{dturn:5.1f}m' if math.isfinite(dturn) else '  nan'

            print(
                f'  sim={sim_t:7.2f}s '
                f'state={state:<8} '
                f'WP={wp:>3} '
                f'CTE={cte_txt} '
                f'head={hdg_txt} '
                f'v={v_txt}/{vt_txt} '
                f'turn={turn:<8} d={dt_txt}',
                flush=True,
            )
            last_print_sim = sim_t

        if state == 'SUCCESS':
            return 'SUCCESS'

        # Abort obviously unusable candidates quickly.
        recent_cte = [
            abs(f(r, 'cross_track_error_m'))
            for r in recent
            if math.isfinite(f(r, 'cross_track_error_m'))
        ]
        if len(recent_cte) >= 40 and max(recent_cte) > 8.0:
            print(
                f'  [{label}] early stop: path departure '
                f'({max(recent_cte):.2f} m)',
                flush=True,
            )
            return 'PATH_DEPARTURE'

        if wp > last_wp:
            last_wp = wp
            last_wp_sim = sim_t
        elif (
            state == 'TRACK'
            and wp > 1
            and sim_t - last_wp_sim > 50.0
        ):
            print(
                f'  [{label}] early stop: waypoint {wp} stalled '
                f'for {sim_t - last_wp_sim:.1f} simulated seconds',
                flush=True,
            )
            return 'WAYPOINT_STALL'

        if sim_t > 300.0:
            print(
                f'  [{label}] simulation timeout at {sim_t:.1f}s',
                flush=True,
            )
            return 'SIM_TIMEOUT'

        time.sleep(0.7)


def run_trial(
    workspace: Path,
    results_root: Path,
    active_dir: Path,
    trial_name: str,
    controller_values: dict,
    planner_values: dict,
    wall_timeout: float,
    progress_label: str,
):
    run_dir = results_root / trial_name
    if run_dir.exists():
        import shutil
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    controller_yaml = active_dir / f'{trial_name}_controller.yaml'
    planner_yaml = active_dir / f'{trial_name}_planner.yaml'
    write_yaml(controller_yaml, 'leader_pid_controller', controller_values)
    write_yaml(planner_yaml, 'trajectory_planner', planner_values)

    log_path = run_dir / 'launch.log'
    csv_path = run_dir / 'r1_stress.csv'

    cleanup_leftovers()

    env = os.environ.copy()
    setup = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {workspace}/install/setup.bash && "
        f"exec ros2 launch platoon_bringup leader_stress_tuning.launch.py "
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
        reason = monitor_trial(
            proc,
            csv_path,
            wall_timeout,
            progress_label,
        )
        terminate_group(proc)

    if reason not in ('SUCCESS',):
        try:
            log_lines = log_path.read_text(errors='replace').splitlines()
            if log_lines:
                print('  ---- last launch-log lines ----', flush=True)
                for line in log_lines[-12:]:
                    print(f'  {line}', flush=True)
                print('  --------------------------------', flush=True)
        except Exception:
            pass

    time.sleep(1.0)
    result = analyse(csv_path)
    result['termination_reason'] = reason
    result['trial_name'] = trial_name
    result['controller'] = controller_values
    result['planner'] = planner_values
    (run_dir / 'trial_result.json').write_text(
        json.dumps(result, indent=2)
    )
    return result


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def guidance_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    controller['heading_kp'] = trial.suggest_float('heading_kp', 350.0, 1450.0)
    controller['heading_ki'] = trial.suggest_float('heading_ki', 0.0, 42.0)
    controller['heading_kd'] = trial.suggest_float('heading_kd', 35.0, 190.0)
    controller['heading_slowdown_angle'] = trial.suggest_float(
        'heading_slowdown_angle', 0.35, 1.05
    )

    planner['minimum_turn_speed'] = trial.suggest_float(
        'minimum_turn_speed', 0.42, 0.95
    )
    planner['lateral_accel_limit'] = trial.suggest_float(
        'lateral_accel_limit', 0.16, 0.58
    )
    planner['lookahead_min'] = trial.suggest_float(
        'lookahead_min', 1.1, 3.0
    )
    planner['tight_lookahead'] = trial.suggest_float(
        'tight_lookahead', 1.2, 3.2
    )
    planner['medium_lookahead'] = trial.suggest_float(
        'medium_lookahead', 2.0, 4.6
    )
    planner['lookahead_speed_gain'] = trial.suggest_float(
        'lookahead_speed_gain', 0.5, 2.7
    )

    # Keep ordering physically sensible.
    planner['tight_lookahead'] = min(
        planner['tight_lookahead'], planner['medium_lookahead']
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

    controller['speed_kp'] = trial.suggest_float('speed_kp', 70.0, 360.0)
    controller['speed_ki'] = trial.suggest_float('speed_ki', 0.0, 45.0)
    controller['speed_kd'] = trial.suggest_float('speed_kd', 5.0, 110.0)

    planner['accel_limit'] = trial.suggest_float('accel_limit', 0.25, 0.90)
    planner['decel_limit'] = trial.suggest_float('decel_limit', 0.30, 1.00)
    return controller, planner


def brake_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    controller['brake_kp'] = trial.suggest_float('brake_kp', 100.0, 650.0)
    controller['brake_ki'] = trial.suggest_float('brake_ki', 0.0, 90.0)
    controller['brake_kd'] = trial.suggest_float('brake_kd', 0.0, 40.0)
    controller['recovery_target_speed'] = trial.suggest_float(
        'recovery_target_speed', 0.12, 0.38
    )
    controller['recovery_brake_distance'] = trial.suggest_float(
        'recovery_brake_distance', 0.55, 0.88
    )
    return controller, planner


def joint_space(trial, controller, planner):
    controller = dict(controller)
    planner = dict(planner)

    # Local refinement around the best staged solution.
    def around(name, current, frac, lo, hi):
        low = clamp(current * (1.0 - frac), lo, hi)
        high = clamp(current * (1.0 + frac), lo, hi)
        if high <= low + 1e-9:
            return current
        return trial.suggest_float(name, low, high)

    controller['heading_kp'] = around(
        'heading_kp', controller['heading_kp'], 0.22, 300.0, 1600.0
    )
    controller['heading_kd'] = around(
        'heading_kd', controller['heading_kd'], 0.25, 20.0, 220.0
    )
    controller['speed_kp'] = around(
        'speed_kp', controller['speed_kp'], 0.25, 50.0, 420.0
    )
    controller['speed_kd'] = around(
        'speed_kd', controller['speed_kd'], 0.30, 0.0, 140.0
    )
    planner['lateral_accel_limit'] = around(
        'lateral_accel_limit', planner['lateral_accel_limit'], 0.25, 0.12, 0.70
    )
    planner['tight_lookahead'] = around(
        'tight_lookahead', planner['tight_lookahead'], 0.22, 1.0, 3.5
    )
    planner['decel_limit'] = around(
        'decel_limit', planner['decel_limit'], 0.25, 0.25, 1.20
    )
    return controller, planner


def stage_counts(total):
    if total < 12:
        return [max(1, total - 3), 1, 1, 1]
    g = max(6, round(total * 0.45))
    s = max(3, round(total * 0.25))
    b = max(2, round(total * 0.15))
    j = max(2, total - g - s - b)
    # Correct rounding to exact total.
    while g + s + b + j > total and g > 6:
        g -= 1
    while g + s + b + j < total:
        j += 1
    return [g, s, b, j]


def main():
    parser = argparse.ArgumentParser(
        description='V2 multivariate/grouped staged headless autotuner for the ordered R1 stress course.'
    )
    parser.add_argument('--trials', type=int, default=36)
    parser.add_argument('--wall-timeout', type=float, default=260.0)
    parser.add_argument('--study-name', default='r1_stress_tune_02')
    parser.add_argument('--workspace', default=str(Path.home() / 'vrx_ws'))
    parser.add_argument('--seed-controller', default=None)
    parser.add_argument('--seed-planner', default=None)
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='delete an existing study directory before starting',
    )
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser()
    project = workspace / 'src/vrx_platooning'
    out = project / 'results/autotune_r1_stress' / args.study_name

    if out.exists() and any(out.iterdir()):
        if not args.overwrite:
            raise SystemExit(
                f'ERROR: study directory already exists and is not empty:\n'
                f'  {out}\n'
                f'Use a new --study-name for a clean run, or pass '
                f'--overwrite deliberately.'
            )
        import shutil
        shutil.rmtree(out)

    trials_root = out / 'trials'
    active_dir = out / 'active_configs'
    trials_root.mkdir(parents=True, exist_ok=True)
    active_dir.mkdir(parents=True, exist_ok=True)

    controller_best = dict(BASE_CONTROLLER)
    planner_best = dict(BASE_PLANNER)

    if args.seed_controller:
        seed_path = Path(args.seed_controller).expanduser()
        data = yaml.safe_load(seed_path.read_text())
        controller_best.update(
            data.get('leader_pid_controller', {}).get('ros__parameters', {})
        )

    if args.seed_planner:
        seed_path = Path(args.seed_planner).expanduser()
        data = yaml.safe_load(seed_path.read_text())
        planner_best.update(
            data.get('trajectory_planner', {}).get('ros__parameters', {})
        )

    history_path = out / 'history.csv'
    history_fields = [
        'stage', 'trial', 'score', 'success',
        'cte_rmse_m', 'cte_p95_m', 'max_abs_cte_m',
        'heading_rmse_deg', 'speed_rmse_mps',
        'straight_mean_speed_mps', 'track_time_s',
        'terminal_time_s', 'recovery_time_s', 'recovery_episodes',
        'termination_reason', 'trial_name',
    ]
    with history_path.open('w', newline='') as hf:
        csv.DictWriter(hf, fieldnames=history_fields).writeheader()

    counts = stage_counts(args.trials)
    stages = [
        ('guidance', counts[0], guidance_space),
        ('speed', counts[1], speed_space),
        ('brake', counts[2], brake_space),
        ('joint', counts[3], joint_space),
    ]

    print()
    print('============================================================')
    print(' R1 STRESS AUTOTUNER V2')
    print('============================================================')
    print(f'Trials      : {args.trials}')
    print(f'Stages      : guidance={counts[0]}, speed={counts[1]}, '
          f'brake={counts[2]}, joint={counts[3]}')
    print('Mode        : LIGHT / HEADLESS / NO RVIZ')
    print('Sampler     : Optuna TPE | multivariate=True | group=True')
    print('Course      : full ordered stress course')
    print(f'Output      : {out}')
    print('============================================================')
    print()

    global_trial = 0
    best_global_score = float('inf')
    best_global_result = None

    # First evaluate the known validated seed once.
    seed_name = f'{args.study_name}_seed'
    print(f'[seed] {seed_name}', flush=True)
    seed = run_trial(
        workspace,
        trials_root,
        active_dir,
        seed_name,
        controller_best,
        planner_best,
        args.wall_timeout,
        'seed',
    )
    best_global_score = seed['score']
    best_global_result = seed
    print(
        f"  score={seed['score']:.3f} "
        f"success={seed['success']} "
        f"cte={seed.get('cte_rmse_m', float('nan')):.3f}m "
        f"heading={seed.get('heading_rmse_deg', float('nan')):.1f}deg"
    )

    seed_row = {k: seed.get(k) for k in history_fields}
    seed_row['stage'] = 'seed'
    seed_row['trial'] = 0
    with history_path.open('a', newline='') as hf:
        csv.DictWriter(hf, fieldnames=history_fields).writerow(seed_row)

    # Save the validated seed immediately. A later candidate replaces it only
    # when it both succeeds and has a lower objective score.
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
    (out / 'best_trial_result.json').write_text(
        json.dumps(best_global_result, indent=2)
    )

    for stage_name, n_trials, space_fn in stages:
        print()
        print(f'===== STAGE {stage_name.upper()} ({n_trials} trials) =====')

        fixed_controller = dict(controller_best)
        fixed_planner = dict(planner_best)

        def objective(trial):
            nonlocal global_trial, controller_best, planner_best
            nonlocal best_global_score, best_global_result

            global_trial += 1
            controller, planner = space_fn(
                trial, fixed_controller, fixed_planner
            )

            trial_name = (
                f'{args.study_name}_{global_trial:03d}_{stage_name}'
            )
            print(
                f'[{global_trial:03d}/{args.trials}] '
                f'{stage_name}: {trial_name}',
                flush=True,
            )
            changed = {
                **{
                    k: v for k, v in controller.items()
                    if fixed_controller.get(k) != v
                },
                **{
                    f'planner.{k}': v for k, v in planner.items()
                    if fixed_planner.get(k) != v
                },
            }
            if changed:
                print(
                    '  sampled: ' +
                    ', '.join(
                        f'{k}={v:.4g}' if isinstance(v, float)
                        else f'{k}={v}'
                        for k, v in changed.items()
                    ),
                    flush=True,
                )

            result = run_trial(
                workspace,
                trials_root,
                active_dir,
                trial_name,
                controller,
                planner,
                args.wall_timeout,
                f'{global_trial:03d}/{args.trials} {stage_name}',
            )

            print(
                f"  score={result['score']:.3f} "
                f"success={result['success']} "
                f"cte={result.get('cte_rmse_m', float('nan')):.3f}m "
                f"max={result.get('max_abs_cte_m', float('nan')):.3f}m "
                f"heading={result.get('heading_rmse_deg', float('nan')):.1f}deg "
                f"straight_v={result.get('straight_mean_speed_mps', float('nan')):.2f}"
            , flush=True)
            print(
                f'  current best score={best_global_score:.3f}',
                flush=True,
            )

            row = {k: result.get(k) for k in history_fields}
            row['stage'] = stage_name
            row['trial'] = global_trial
            with history_path.open('a', newline='') as hf:
                csv.DictWriter(hf, fieldnames=history_fields).writerow(row)

            trial.set_user_attr('controller', controller)
            trial.set_user_attr('planner', planner)
            trial.set_user_attr('result', {
                k: v for k, v in result.items()
                if k not in ('controller', 'planner')
            })

            if result['score'] < best_global_score and result['success']:
                best_global_score = result['score']
                best_global_result = result
                controller_best = dict(controller)
                planner_best = dict(planner)

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
                (out / 'best_trial_result.json').write_text(
                    json.dumps(best_global_result, indent=2)
                )
                print('  >>> NEW GLOBAL BEST')

            return result['score']

        sampler = optuna.samplers.TPESampler(
            seed=42 + global_trial,
            n_startup_trials=max(3, min(7, n_trials // 3)),
            multivariate=True,
            group=True,
            warn_independent_sampling=False,
        )

        storage_path = out / 'optuna.sqlite3'
        stage_study_name = f'{args.study_name}_{stage_name}'

        study = optuna.create_study(
            study_name=stage_study_name,
            storage=f'sqlite:///{storage_path}',
            direction='minimize',
            sampler=sampler,
            load_if_exists=False,
        )
        study.optimize(
            objective,
            n_trials=n_trials,
            catch=(Exception,),
        )

        # The globally best successful configuration is carried to the next
        # stage. Failed trials can never replace it.
        print(
            f'Best after {stage_name}: score={best_global_score:.3f}'
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

    summary = {
        'study_name': args.study_name,
        'requested_trials': args.trials,
        'sampler': {
            'name': 'TPESampler',
            'multivariate': True,
            'group': True,
        },
        'best_score': best_global_score,
        'best_controller': controller_best,
        'best_planner': planner_best,
        'best_result': best_global_result,
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))

    print()
    print('============================================================')
    print(' AUTOTUNING COMPLETE')
    print('============================================================')
    print(f'Best score       : {best_global_score:.3f}')
    if best_global_result:
        print(
            f"CTE RMSE         : "
            f"{best_global_result.get('cte_rmse_m', float('nan')):.3f} m"
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
            f"Straight speed   : "
            f"{best_global_result.get('straight_mean_speed_mps', float('nan')):.3f} m/s"
        )
    print(f'Controller YAML  : {out / "best_controller.yaml"}')
    print(f'Planner YAML     : {out / "best_planner.yaml"}')
    print(f'History          : {history_path}')
    print(f'Summary          : {out / "summary.json"}')
    print('============================================================')


if __name__ == '__main__':
    main()
