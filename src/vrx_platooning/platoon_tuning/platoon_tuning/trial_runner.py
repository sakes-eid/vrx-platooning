import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from platoon_tuning.trial_analyzer import (
    TrialAnalyzer,
)


# =============================================================
# Candidate parsing
# =============================================================


def parse_candidate(values):

    candidate = {}

    for value in values:

        if '=' not in value:

            raise ValueError(
                f'Invalid candidate parameter: {value}. '
                'Expected name=value.'
            )

        name, raw_value = value.split(
            '=',
            1,
        )

        name = name.strip()

        if not name:

            raise ValueError(
                f'Invalid candidate parameter: {value}'
            )

        candidate[name] = float(
            raw_value
        )

    return candidate


# =============================================================
# Trial runner
# =============================================================


class TrialRunner:

    def __init__(
        self,
        scenario,
        run_name,
        candidate,
        results_dir,
        max_sim_time=90.0,
        divergence_error=4.0,
        wall_timeout=180.0,
    ):

        self.scenario = scenario

        self.run_name = run_name

        self.candidate = candidate

        self.max_sim_time = float(
            max_sim_time
        )

        self.divergence_error = float(
            divergence_error
        )

        self.wall_timeout = float(
            wall_timeout
        )

        self.results_dir = Path(
            results_dir
        ).expanduser()

        self.run_dir = (
            self.results_dir
            / self.run_name
        )

        self.csv_path = (
            self.run_dir
            / 'trajectory.csv'
        )

        self.result_path = (
            self.run_dir
            / 'trial_result.json'
        )

        self.processes = []

        self.log_files = []

        self.start_wall_time = None

        self.end_wall_time = None

        self.status = 'NOT_STARTED'

        self.reason = ''

        # -----------------------------------------------------
        # ROS package locations
        # -----------------------------------------------------

        self.tuning_share = Path(
            get_package_share_directory(
                'platoon_tuning'
            )
        )

        self.state_share = Path(
            get_package_share_directory(
                'platoon_state'
            )
        )

        self.planner_share = Path(
            get_package_share_directory(
                'platoon_planner'
            )
        )

        self.bringup_share = Path(
            get_package_share_directory(
                'platoon_bringup'
            )
        )

        # -----------------------------------------------------
        # Required files
        # -----------------------------------------------------

        self.world_file = (
            self.tuning_share
            / 'worlds'
            / 'tuning'
            / 'sydney_regatta.sdf'
        )

        self.light_urdf = (
            self.bringup_share
            / 'urdf'
            / 'wamv_light.urdf.xacro'
        )

        self.origin_config = (
            self.state_share
            / 'config'
            / 'origin.yaml'
        )

        self.planner_config = (
            self.planner_share
            / 'config'
            / 'planner.yaml'
        )

        self.environment = (
            os.environ.copy()
        )

        self.prepare_resource_path()

    # =========================================================
    # Environment
    # =========================================================

    def prepare_resource_path(self):

        packages = [
            'wamv_description',
            'wamv_gazebo',
            'vrx_gz',
            'platoon_bringup',
            'platoon_tuning',
        ]

        resource_paths = []

        for package in packages:

            try:

                share = Path(
                    get_package_share_directory(
                        package
                    )
                )

                # Existing working setup used:
                #
                # <package-prefix>/share
                #
                # rather than only share/<package>.
                resource_paths.append(
                    str(
                        share.parent
                    )
                )

            except Exception:

                pass

        existing = self.environment.get(
            'GZ_SIM_RESOURCE_PATH',
            ''
        )

        if existing:

            resource_paths.append(
                existing
            )

        self.environment[
            'GZ_SIM_RESOURCE_PATH'
        ] = ':'.join(
            resource_paths
        )

    # =========================================================
    # Safety checks
    # =========================================================

    def validate(self):

        if self.scenario not in (
            'straight',
            'curved',
        ):

            raise ValueError(
                'Scenario must be '
                '"straight" or "curved".'
            )

        required_files = [

            self.world_file,
            self.light_urdf,
            self.origin_config,
            self.planner_config,
        ]

        for path in required_files:

            if not path.exists():

                raise FileNotFoundError(
                    f'Required file missing: '
                    f'{path}'
                )

        # -----------------------------------------------------
        # Prevent accidentally tuning while the normal
        # experiment is already running.
        # -----------------------------------------------------

        result = subprocess.run(
            [
                'ros2',
                'node',
                'list',
            ],
            capture_output=True,
            text=True,
            env=self.environment,
        )

        active_nodes = set(
            line.strip()
            for line
            in result.stdout.splitlines()
            if line.strip()
        )

        conflicting_nodes = {

            '/leader_pid_controller',
            '/trajectory_planner',
            '/trajectory_logger',
            '/vehicle_state_node',
        }

        conflicts = (
            active_nodes
            & conflicting_nodes
        )

        if conflicts:

            raise RuntimeError(
                'Existing experiment nodes detected: '
                +
                ', '.join(
                    sorted(
                        conflicts
                    )
                )
                +
                '. Stop the normal experiment before '
                'starting autotuning.'
            )

    # =========================================================
    # Process handling
    # =========================================================

    def start_process(
        self,
        name,
        command,
    ):

        log_path = (
            self.run_dir
            / f'{name}.log'
        )

        log_file = log_path.open(
            'w',
            encoding='utf-8',
        )

        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=self.environment,
            start_new_session=True,
            text=True,
        )

        self.processes.append(
            (
                name,
                process,
            )
        )

        self.log_files.append(
            log_file
        )

        return process

    # ---------------------------------------------------------

    def stop_processes(self):

        # -----------------------------------------------------
        # First request a clean Ctrl+C-style shutdown.
        # -----------------------------------------------------

        for name, process in reversed(
            self.processes
        ):

            if process.poll() is not None:
                continue

            try:

                os.killpg(
                    os.getpgid(
                        process.pid
                    ),
                    signal.SIGINT,
                )

            except ProcessLookupError:

                pass

        deadline = (
            time.monotonic()
            + 5.0
        )

        for name, process in reversed(
            self.processes
        ):

            if process.poll() is not None:
                continue

            remaining = (
                deadline
                - time.monotonic()
            )

            if remaining <= 0.0:
                break

            try:

                process.wait(
                    timeout=remaining
                )

            except subprocess.TimeoutExpired:

                pass

        # -----------------------------------------------------
        # Escalate if necessary.
        # -----------------------------------------------------

        for name, process in reversed(
            self.processes
        ):

            if process.poll() is not None:
                continue

            try:

                os.killpg(
                    os.getpgid(
                        process.pid
                    ),
                    signal.SIGTERM,
                )

            except ProcessLookupError:

                pass

        time.sleep(
            1.0
        )

        for name, process in reversed(
            self.processes
        ):

            if process.poll() is not None:
                continue

            try:

                os.killpg(
                    os.getpgid(
                        process.pid
                    ),
                    signal.SIGKILL,
                )

            except ProcessLookupError:

                pass

        for log_file in self.log_files:

            try:
                log_file.flush()
                log_file.close()

            except Exception:
                pass

    # =========================================================
    # Gazebo startup
    # =========================================================

    def wait_for_simulation(
        self,
        gazebo_process,
        timeout=45.0,
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        while (
            time.monotonic()
            < deadline
        ):

            if (
                gazebo_process.poll()
                is not None
            ):

                raise RuntimeError(
                    'Gazebo exited during startup. '
                    'Check gazebo.log.'
                )

            result = subprocess.run(
                [
                    'ros2',
                    'topic',
                    'list',
                ],
                capture_output=True,
                text=True,
                env=self.environment,
            )

            topics = set(
                result.stdout.splitlines()
            )

            if (
                '/clock'
                in topics
                and
                '/wamv/sensors/gps/gps/fix'
                in topics
                and
                '/wamv/sensors/imu/imu/data'
                in topics
            ):

                return

            time.sleep(
                0.25
            )

        raise RuntimeError(
            'Simulation did not become ready '
            'within startup timeout.'
        )

    # =========================================================
    # Deterministic trial startup
    # =========================================================

    def wait_for_trial_nodes(
        self,
        timeout=20.0,
    ):

        expected_nodes = {
            '/vehicle_state_node',
            '/trajectory_planner',
            '/trajectory_logger',
            '/leader_pid_controller',
        }

        deadline = (
            time.monotonic()
            + timeout
        )

        while (
            time.monotonic()
            < deadline
        ):

            result = subprocess.run(
                [
                    'ros2',
                    'node',
                    'list',
                ],
                capture_output=True,
                text=True,
                env=self.environment,
            )

            nodes = set(
                result.stdout.splitlines()
            )

            missing = (
                expected_nodes
                - nodes
            )

            if not missing:

                return

            time.sleep(
                0.25
            )

        raise RuntimeError(
            'Trial ROS nodes did not become ready. '
            f'Missing: {sorted(missing)}'
        )


    def unpause_simulation(
        self,
        timeout=10.0,
    ):

        # -----------------------------------------------------
        # Discover the Gazebo world-control service rather than
        # hard-coding the world name.
        # -----------------------------------------------------

        deadline = (
            time.monotonic()
            + timeout
        )

        control_service = None

        while (
            time.monotonic()
            < deadline
        ):

            result = subprocess.run(
                [
                    'gz',
                    'service',
                    '-l',
                ],
                capture_output=True,
                text=True,
                env=self.environment,
            )

            services = [
                line.strip()
                for line
                in result.stdout.splitlines()
                if line.strip()
            ]

            candidates = [
                service
                for service
                in services
                if (
                    service.startswith(
                        '/world/'
                    )
                    and
                    service.endswith(
                        '/control'
                    )
                )
            ]

            if candidates:

                # Prefer Sydney Regatta if present.
                preferred = [
                    service
                    for service
                    in candidates
                    if (
                        '/sydney_regatta/'
                        in service
                    )
                ]

                control_service = (
                    preferred[0]
                    if preferred
                    else candidates[0]
                )

                break

            time.sleep(
                0.25
            )

        if control_service is None:

            raise RuntimeError(
                'Could not find Gazebo '
                'world control service.'
            )

        result = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                control_service,
                '--reqtype',
                'gz.msgs.WorldControl',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '5000',
                '--req',
                'pause: false',
            ],
            capture_output=True,
            text=True,
            env=self.environment,
        )

        if result.returncode != 0:

            raise RuntimeError(
                'Failed to unpause Gazebo.\n'
                f'STDOUT: {result.stdout}\n'
                f'STDERR: {result.stderr}'
            )


    # =========================================================
    # CSV monitoring
    # =========================================================

    def read_last_csv_row(self):

        if not self.csv_path.exists():

            return None

        try:

            with self.csv_path.open(
                newline='',
                encoding='utf-8',
            ) as csv_file:

                reader = csv.DictReader(
                    csv_file
                )

                last_row = None

                for row in reader:

                    last_row = row

                return last_row

        except (
            OSError,
            ValueError,
        ):

            return None

    # =========================================================
    # Main run
    # =========================================================

    def run(self):

        self.validate()

        self.run_dir.mkdir(
            parents=True,
            exist_ok=False,
        )

        self.start_wall_time = (
            time.monotonic()
        )

        print()
        print(
            '============================================'
        )
        print(
            ' AUTOTUNER SINGLE TRIAL'
        )
        print(
            '============================================'
        )
        print(
            f'Run       : {self.run_name}'
        )
        print(
            f'Scenario  : {self.scenario}'
        )
        print(
            'Mode      : LIGHT / HEADLESS / ACCELERATED'
        )
        print()

        if self.candidate:

            print(
                'Candidate parameters:'
            )

            for name, value in sorted(
                self.candidate.items()
            ):

                print(
                    f'  {name:<24} {value}'
                )

            print()

        try:

            # =================================================
            # Gazebo + VRX
            # =================================================

            gazebo = self.start_process(
                'gazebo',
                [
                    'ros2',
                    'launch',
                    'vrx_gz',
                    'competition.launch.py',

                    (
                        'world:='
                        + str(
                            self.world_file
                        )
                    ),

                    'sim_mode:=full',

                    'headless:=True',

                    'paused:=True',

                    (
                        'urdf:='
                        + str(
                            self.light_urdf
                        )
                    ),
                ],
            )

            print(
                'Starting headless Gazebo...'
            )

            self.wait_for_simulation(
                gazebo
            )

            print(
                'Simulation ready.'
            )

            # =================================================
            # Vehicle state
            # =================================================

            self.start_process(
                'vehicle_state',
                [
                    'ros2',
                    'run',
                    'platoon_state',
                    'vehicle_state',

                    '--ros-args',

                    '--params-file',
                    str(
                        self.origin_config
                    ),

                    '-p',
                    'use_sim_time:=true',
                ],
            )

            # =================================================
            # Planner
            # =================================================

            self.start_process(
                'planner',
                [
                    'ros2',
                    'run',
                    'platoon_planner',
                    'trajectory_planner',

                    '--ros-args',

                    '--params-file',
                    str(
                        self.planner_config
                    ),

                    '-p',
                    'use_sim_time:=true',

                    '-p',
                    (
                        'trajectory_type:='
                        + self.scenario
                    ),
                ],
            )

            # =================================================
            # Logger
            # =================================================

            lookahead = (
                self.candidate.get(
                    'lookahead_distance',
                    3.0,
                )
            )

            target_speed = (
                self.candidate.get(
                    'target_speed',
                    1.0,
                )
            )

            self.start_process(
                'logger',
                [
                    'ros2',
                    'run',
                    'platoon_monitor',
                    'trajectory_logger',

                    '--ros-args',

                    '-p',
                    'use_sim_time:=true',

                    '-p',
                    (
                        'output_file:='
                        + str(
                            self.csv_path
                        )
                    ),

                    '-p',
                    (
                        'lookahead_distance:='
                        + str(
                            lookahead
                        )
                    ),

                    '-p',
                    (
                        'target_speed:='
                        + str(
                            target_speed
                        )
                    ),

                    '-p',
                    (
                        'controller_state_topic:='
                        '/experiment/controller_state'
                    ),

                    '-p',
                    (
                        'success_topic:='
                        '/experiment/success'
                    ),
                ],
            )

            # =================================================
            # Controller
            # =================================================

            controller_command = [
                'ros2',
                'run',
                'platoon_control',
                'leader_pid_controller',

                '--ros-args',

                '-p',
                'use_sim_time:=true',
            ]

            for name, value in sorted(
                self.candidate.items()
            ):

                controller_command.extend(
                    [
                        '-p',
                        f'{name}:={value}',
                    ]
                )

            self.start_process(
                'controller',
                controller_command,
            )

            # =================================================
            # Deterministic startup
            #
            # Gazebo is still paused here. Make sure every ROS
            # node exists before allowing simulated time to move.
            # =================================================

            print(
                'Waiting for trial nodes...'
            )

            self.wait_for_trial_nodes()

            # Give DDS publishers/subscribers a short wall-time
            # window to discover each other while simulation
            # remains frozen.
            time.sleep(
                1.0
            )

            print(
                'All trial nodes ready.'
            )

            print(
                'Unpausing simulation...'
            )

            self.unpause_simulation()

            print(
                'Trial running...'
            )

            # =================================================
            # Monitor experiment
            # =================================================

            wall_deadline = (
                self.start_wall_time
                + self.wall_timeout
            )

            last_reported_sim_second = -1

            while True:

                if (
                    time.monotonic()
                    >= wall_deadline
                ):

                    self.status = (
                        'FAILED'
                    )

                    self.reason = (
                        'WALL_TIMEOUT'
                    )

                    break

                row = (
                    self.read_last_csv_row()
                )

                if row is None:

                    time.sleep(
                        0.05
                    )

                    continue

                try:

                    sim_time = float(
                        row[
                            'time_s'
                        ]
                    )

                    position_error = float(
                        row[
                            'position_error_m'
                        ]
                    )

                    state = row.get(
                        'controller_state',
                        'UNKNOWN'
                    )

                except (
                    KeyError,
                    TypeError,
                    ValueError,
                ):

                    time.sleep(
                        0.05
                    )

                    continue

                whole_second = int(
                    sim_time
                )

                if (
                    whole_second
                    > last_reported_sim_second
                ):

                    last_reported_sim_second = (
                        whole_second
                    )

                    print(
                        f'  sim={sim_time:6.2f}s  '
                        f'state={state:<8}  '
                        f'error={position_error:.3f}m'
                    )

                # ---------------------------------------------
                # Success
                # ---------------------------------------------

                if state == 'SUCCESS':

                    self.status = (
                        'SUCCESS'
                    )

                    self.reason = (
                        'SUCCESS'
                    )

                    break

                # ---------------------------------------------
                # Divergence
                # ---------------------------------------------

                if (
                    state == 'TRACK'
                    and
                    position_error
                    > self.divergence_error
                ):

                    self.status = (
                        'FAILED'
                    )

                    self.reason = (
                        'DIVERGENCE'
                    )

                    break

                # ---------------------------------------------
                # Simulated-time limit
                # ---------------------------------------------

                if (
                    sim_time
                    >= self.max_sim_time
                ):

                    self.status = (
                        'FAILED'
                    )

                    self.reason = (
                        'SIM_TIME_LIMIT'
                    )

                    break

                time.sleep(
                    0.05
                )

        except Exception as error:

            self.status = (
                'FAILED'
            )

            self.reason = (
                f'EXCEPTION: {error}'
            )

        finally:

            self.end_wall_time = (
                time.monotonic()
            )

            print()
            print(
                'Stopping trial processes...'
            )

            self.stop_processes()

        # =====================================================
        # Analyze output
        # =====================================================

        wall_duration = (
            self.end_wall_time
            - self.start_wall_time
        )

        analysis = None

        if self.csv_path.exists():

            try:

                analysis = (
                    TrialAnalyzer(
                        target_speed=(
                            self.candidate.get(
                                'target_speed',
                                1.0,
                            )
                        )
                    ).analyze(
                        self.csv_path
                    )
                )

            except Exception as error:

                self.reason += (
                    f' | ANALYSIS_ERROR: {error}'
                )

        # =====================================================
        # Effective speed estimate
        # =====================================================

        effective_rtf = None

        if (
            analysis is not None
            and
            wall_duration > 0.0
        ):

            effective_rtf = (
                analysis[
                    'total_time_s'
                ]
                /
                wall_duration
            )

        result = {

            'run_name':
                self.run_name,

            'scenario':
                self.scenario,

            'status':
                self.status,

            'reason':
                self.reason,

            'candidate':
                self.candidate,

            'wall_duration_s':
                wall_duration,

            'effective_end_to_end_rtf':
                effective_rtf,

            'analysis':
                analysis,
        }

        self.result_path.write_text(
            json.dumps(
                result,
                indent=2,
            )
        )

        # =====================================================
        # Report
        # =====================================================

        print()
        print(
            '============================================'
        )
        print(
            ' TRIAL COMPLETE'
        )
        print(
            '============================================'
        )
        print(
            f'Status      : {self.status}'
        )
        print(
            f'Reason      : {self.reason}'
        )
        print(
            f'Wall time   : {wall_duration:.2f} s'
        )

        if (
            effective_rtf
            is not None
        ):

            print(
                f'End-to-end  : '
                f'{effective_rtf:.2f} x realtime'
            )

        if analysis is not None:

            tracking = analysis[
                'tracking'
            ]

            print(
                f'Sim time    : '
                f'{analysis["total_time_s"]:.2f} s'
            )

            print(
                f'RMSE        : '
                f'{tracking["position_rmse_m"]:.3f} m'
            )

            print(
                f'Max error   : '
                f'{tracking["maximum_position_error_m"]:.3f} m'
            )

            print(
                f'Recoveries  : '
                f'{analysis["recovery"]["count"]}'
            )

        print(
            f'Results     : {self.run_dir}'
        )

        print(
            '============================================'
        )
        print()

        return result


# =============================================================
# CLI
# =============================================================


def main():

    parser = argparse.ArgumentParser(
        description=(
            'Run one isolated headless '
            'autotuning experiment.'
        )
    )

    parser.add_argument(
        '--scenario',
        choices=[
            'straight',
            'curved',
        ],
        required=True,
    )

    parser.add_argument(
        '--run-name',
        required=True,
    )

    parser.add_argument(
        '--candidate',
        action='append',
        default=[],
        help=(
            'Controller parameter override '
            'in name=value form. '
            'Can be repeated.'
        ),
    )

    parser.add_argument(
        '--results-dir',
        default=(
            '/home/sajed/vrx_ws/src/'
            'vrx_platooning/results/autotune'
        ),
    )

    parser.add_argument(
        '--max-sim-time',
        type=float,
        default=90.0,
    )

    parser.add_argument(
        '--divergence-error',
        type=float,
        default=4.0,
    )

    parser.add_argument(
        '--wall-timeout',
        type=float,
        default=180.0,
    )

    args = parser.parse_args()

    candidate = parse_candidate(
        args.candidate
    )

    runner = TrialRunner(
        scenario=(
            args.scenario
        ),

        run_name=(
            args.run_name
        ),

        candidate=(
            candidate
        ),

        results_dir=(
            args.results_dir
        ),

        max_sim_time=(
            args.max_sim_time
        ),

        divergence_error=(
            args.divergence_error
        ),

        wall_timeout=(
            args.wall_timeout
        ),
    )

    result = runner.run()

    if result[
        'status'
    ] != 'SUCCESS':

        raise SystemExit(
            1
        )


if __name__ == '__main__':

    main()
