import argparse
import csv
import json
import math
import random
import time

from copy import deepcopy
from datetime import datetime
from pathlib import Path

import yaml

from platoon_tuning.objective import (
    FAILURE_SCORE,
    all_scenarios_pass,
    combined_score,
)

from platoon_tuning.trial_runner import (
    TrialRunner,
)


# =============================================================
# Config
# =============================================================


def load_config(
    config_path,
):

    path = Path(
        config_path
    ).expanduser()

    with path.open(
        encoding='utf-8',
    ) as file:

        data = yaml.safe_load(
            file
        )

    if (
        not data
        or
        'autotune'
        not in data
    ):

        raise ValueError(
            'Missing top-level autotune section.'
        )

    return data[
        'autotune'
    ]


# =============================================================
# Candidate utilities
# =============================================================


def candidate_key(
    candidate,
):

    return tuple(
        (
            key,
            round(
                float(value),
                8,
            ),
        )

        for key, value
        in sorted(
            candidate.items()
        )
    )


def random_stage_candidate(
    base,
    parameters,
    rng,
):

    candidate = deepcopy(
        base
    )

    for name, bounds in parameters.items():

        minimum = float(
            bounds[
                'min'
            ]
        )

        maximum = float(
            bounds[
                'max'
            ]
        )

        candidate[
            name
        ] = rng.uniform(
            minimum,
            maximum,
        )

    return candidate


def refinement_candidate(
    base,
    relative_range,
    rng,
):

    candidate = deepcopy(
        base
    )

    for name, value in base.items():

        value = float(
            value
        )

        if name == 'lookahead_distance':

            lower = max(
                0.5,
                value
                * (
                    1.0
                    - relative_range
                ),
            )

        else:

            lower = max(
                0.0,
                value
                * (
                    1.0
                    - relative_range
                ),
            )

        upper = value * (
            1.0
            + relative_range
        )

        # If the best value is zero, still allow the optimizer
        # to test a small nonzero value.
        if upper == 0.0:

            if name.endswith(
                '_ki'
            ):

                upper = 10.0

            elif name.endswith(
                '_kd'
            ):

                upper = 30.0

            else:

                upper = 1.0

        candidate[
            name
        ] = rng.uniform(
            lower,
            upper,
        )

    return candidate


# =============================================================
# Output
# =============================================================


def save_best_yaml(
    path,
    candidate,
):

    data = {

        'leader_pid_controller': {

            'ros__parameters':
                {
                    key:
                        float(value)

                    for key, value
                    in sorted(
                        candidate.items()
                    )
                }
        }
    }

    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
        )
    )


# =============================================================
# Autotuner
# =============================================================


class AutoTuner:

    def __init__(
        self,
        config,
        session_name,
        results_root,
        selected_stages=None,
    ):

        self.config = config

        self.session_name = (
            session_name
        )

        self.results_root = Path(
            results_root
        ).expanduser()

        self.session_dir = (
            self.results_root
            / self.session_name
        )

        self.session_dir.mkdir(
            parents=True,
            exist_ok=False,
        )

        self.rng = random.Random(
            int(
                config.get(
                    'random_seed',
                    42,
                )
            )
        )

        self.current_best = deepcopy(
            config[
                'baseline'
            ]
        )

        self.current_best_score = (
            FAILURE_SCORE
        )

        self.cache = {}

        self.history = []

        configured_stages = (
            config[
                'leader'
            ][
                'stages'
            ]
        )

        if selected_stages:

            self.stages = [
                stage
                for stage
                in configured_stages
                if stage
                in selected_stages
            ]

        else:

            self.stages = list(
                configured_stages
            )

        self.history_file = (
            self.session_dir
            / 'history.csv'
        )

        self.best_yaml = (
            self.session_dir
            / 'best_pid.yaml'
        )

        self.summary_json = (
            self.session_dir
            / 'summary.json'
        )

    # =========================================================
    # Trial execution
    # =========================================================

    def run_scenario(
        self,
        stage,
        trial_number,
        scenario,
        candidate,
    ):

        run_name = (
            f'{stage}_'
            f'{trial_number:03d}_'
            f'{scenario}'
        )

        # -----------------------------------------------------
        # Give DDS / old process discovery a moment to clear
        # between completely separate Gazebo runs.
        # -----------------------------------------------------

        time.sleep(
            1.0
        )

        for attempt in range(
            1,
            4,
        ):

            try:

                runner = TrialRunner(

                    scenario=scenario,

                    run_name=run_name,

                    candidate=candidate,

                    results_dir=(
                        self.session_dir
                    ),

                    max_sim_time=float(
                        self.config[
                            'maximum_sim_time'
                        ]
                    ),

                    divergence_error=float(
                        self.config[
                            'divergence_error'
                        ]
                    ),

                    wall_timeout=float(
                        self.config.get(
                            'wall_timeout',
                            180.0,
                        )
                    ),
                )

                result = runner.run()

                return result

            except RuntimeError as error:

                if (
                    'Existing experiment nodes detected'
                    not in str(
                        error
                    )
                    or
                    attempt == 3
                ):

                    raise

                print(
                    'Waiting for ROS graph cleanup...'
                )

                time.sleep(
                    3.0
                )

        return None

    # =========================================================
    # Candidate evaluation
    # =========================================================

    def evaluate_candidate(
        self,
        stage,
        trial_number,
        candidate,
    ):

        key = candidate_key(
            candidate
        )

        if key in self.cache:

            cached = self.cache[
                key
            ]

            print(
                'Candidate already evaluated. '
                'Reusing cached score.'
            )

            return cached

        scenario_results = {}

        hard_failure = False

        for scenario in self.config[
            'scenarios'
        ]:

            print()
            print(
                '--------------------------------------------'
            )
            print(
                f'{stage.upper()} '
                f'TRIAL {trial_number} '
                f'- {scenario.upper()}'
            )
            print(
                '--------------------------------------------'
            )

            try:

                trial_result = (
                    self.run_scenario(
                        stage=stage,
                        trial_number=(
                            trial_number
                        ),
                        scenario=scenario,
                        candidate=candidate,
                    )
                )

            except Exception as error:

                print(
                    f'Trial launch failed: {error}'
                )

                hard_failure = True
                break

            if (
                trial_result is None
                or
                trial_result.get(
                    'status'
                )
                != 'SUCCESS'
                or
                trial_result.get(
                    'analysis'
                )
                is None
            ):

                hard_failure = True
                break

            scenario_results[
                scenario
            ] = trial_result[
                'analysis'
            ]

        if hard_failure:

            score = FAILURE_SCORE
            passed = False

        else:

            score = combined_score(
                scenario_results,
                self.config,
            )

            passed = all_scenarios_pass(
                scenario_results,
                self.config,
            )

        evaluation = {

            'candidate':
                deepcopy(
                    candidate
                ),

            'scenario_results':
                scenario_results,

            'score':
                float(
                    score
                ),

            'passed':
                bool(
                    passed
                ),
        }

        self.cache[
            key
        ] = evaluation

        return evaluation

    # =========================================================
    # History
    # =========================================================

    def record_history(
        self,
        stage,
        trial_number,
        evaluation,
        became_best,
    ):

        candidate = evaluation[
            'candidate'
        ]

        scenario_results = evaluation[
            'scenario_results'
        ]

        row = {

            'stage':
                stage,

            'trial':
                trial_number,

            'score':
                evaluation[
                    'score'
                ],

            'passed_acceptance':
                evaluation[
                    'passed'
                ],

            'became_best':
                became_best,
        }

        for name, value in sorted(
            candidate.items()
        ):

            row[
                name
            ] = value

        for scenario in self.config[
            'scenarios'
        ]:

            result = scenario_results.get(
                scenario
            )

            prefix = (
                scenario
                + '_'
            )

            if result is None:

                row[
                    prefix
                    + 'success'
                ] = False

                continue

            tracking = result.get(
                'post_capture_tracking'
            )

            capture = result.get(
                'capture',
                {},
            )

            row[
                prefix
                + 'success'
            ] = result.get(
                'success',
                False,
            )

            row[
                prefix
                + 'capture_time_s'
            ] = capture.get(
                'capture_time_s'
            )

            if tracking:

                row[
                    prefix
                    + 'rmse_m'
                ] = tracking.get(
                    'position_rmse_m'
                )

                row[
                    prefix
                    + 'max_error_m'
                ] = tracking.get(
                    'maximum_position_error_m'
                )

                row[
                    prefix
                    + 'heading_rmse_rad'
                ] = tracking.get(
                    'heading_rmse_rad'
                )

                row[
                    prefix
                    + 'speed_rmse_mps'
                ] = tracking.get(
                    'speed_rmse_mps'
                )

            row[
                prefix
                + 'recoveries'
            ] = result.get(
                'recovery',
                {},
            ).get(
                'count',
                0,
            )

        self.history.append(
            row
        )

        all_fields = []

        for history_row in self.history:

            for field in history_row:

                if field not in all_fields:

                    all_fields.append(
                        field
                    )

        with self.history_file.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=all_fields,
            )

            writer.writeheader()

            for history_row in self.history:

                writer.writerow(
                    history_row
                )

    # =========================================================
    # Candidate handling
    # =========================================================

    def consider(
        self,
        stage,
        trial_number,
        candidate,
    ):

        evaluation = (
            self.evaluate_candidate(
                stage,
                trial_number,
                candidate,
            )
        )

        score = evaluation[
            'score'
        ]

        became_best = (
            score
            < self.current_best_score
        )

        if became_best:

            old_score = (
                self.current_best_score
            )

            self.current_best = (
                deepcopy(
                    candidate
                )
            )

            self.current_best_score = (
                score
            )

            save_best_yaml(
                self.best_yaml,
                self.current_best,
            )

            print()
            print(
                '############################################'
            )
            print(
                ' NEW BEST CONTROLLER'
            )
            print(
                '############################################'
            )
            print(
                f'Old score : {old_score:.6f}'
            )
            print(
                f'New score : {score:.6f}'
            )

            for name, value in sorted(
                self.current_best.items()
            ):

                print(
                    f'{name:<24} '
                    f'{value:.6f}'
                )

            print(
                '############################################'
            )

        else:

            print()
            print(
                f'Candidate score : {score:.6f}'
            )
            print(
                f'Best score      : '
                f'{self.current_best_score:.6f}'
            )

        if evaluation[
            'passed'
        ]:

            print(
                'Candidate satisfies all '
                'acceptance criteria.'
            )

        self.record_history(
            stage,
            trial_number,
            evaluation,
            became_best,
        )

        self.write_summary()

        return evaluation

    # =========================================================
    # Stage execution
    # =========================================================

    def run_stage(
        self,
        stage,
    ):

        stage_config = (
            self.config[
                'leader'
            ][
                stage
            ]
        )

        trials = int(
            stage_config[
                'trials'
            ]
        )

        print()
        print()
        print(
            '============================================'
        )
        print(
            f' STARTING STAGE: {stage.upper()}'
        )
        print(
            f' Trials: {trials}'
        )
        print(
            '============================================'
        )

        # -----------------------------------------------------
        # Trial 0 / first candidate:
        # current best controller.
        #
        # Cache prevents needless reruns between stages.
        # -----------------------------------------------------

        self.consider(
            stage,
            0,
            deepcopy(
                self.current_best
            ),
        )

        for trial_number in range(
            1,
            trials + 1,
        ):

            if stage == 'refinement':

                relative_range = float(
                    stage_config[
                        'relative_range'
                    ]
                )

                candidate = (
                    refinement_candidate(
                        self.current_best,
                        relative_range,
                        self.rng,
                    )
                )

            else:

                parameters = (
                    stage_config[
                        'parameters'
                    ]
                )

                candidate = (
                    random_stage_candidate(
                        self.current_best,
                        parameters,
                        self.rng,
                    )
                )

            self.consider(
                stage,
                trial_number,
                candidate,
            )

    # =========================================================
    # Summary
    # =========================================================

    def write_summary(
        self,
    ):

        summary = {

            'session_name':
                self.session_name,

            'best_score':
                self.current_best_score,

            'best_candidate':
                self.current_best,

            'evaluated_candidates':
                len(
                    self.cache
                ),

            'history_rows':
                len(
                    self.history
                ),
        }

        self.summary_json.write_text(
            json.dumps(
                summary,
                indent=2,
            )
        )

    # =========================================================
    # Run
    # =========================================================

    def run(self):

        print()
        print(
            '============================================'
        )
        print(
            ' FULL AUTONOMOUS LEADER PID TUNING'
        )
        print(
            '============================================'
        )
        print(
            f'Session : {self.session_name}'
        )
        print(
            'Stages  : '
            +
            ', '.join(
                self.stages
            )
        )
        print(
            'Scenarios: '
            +
            ', '.join(
                self.config[
                    'scenarios'
                ]
            )
        )
        print(
            f'Results : {self.session_dir}'
        )
        print(
            '============================================'
        )

        for stage in self.stages:

            self.run_stage(
                stage
            )

        self.write_summary()

        print()
        print(
            '============================================'
        )
        print(
            ' AUTOTUNING COMPLETE'
        )
        print(
            '============================================'
        )
        print(
            f'Best score : '
            f'{self.current_best_score:.6f}'
        )
        print(
            f'Best PID   : '
            f'{self.best_yaml}'
        )
        print(
            f'History    : '
            f'{self.history_file}'
        )
        print(
            '============================================'
        )


# =============================================================
# CLI
# =============================================================


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--config',
        default=(
            '/home/sajed/vrx_ws/src/'
            'vrx_platooning/platoon_tuning/'
            'config/autotune.yaml'
        ),
    )

    parser.add_argument(
        '--session-name',
        default=None,
    )

    parser.add_argument(
        '--results-root',
        default=(
            '/home/sajed/vrx_ws/src/'
            'vrx_platooning/results/'
            'autotune_sessions'
        ),
    )

    parser.add_argument(
        '--stages',
        nargs='+',
        choices=[
            'heading',
            'speed',
            'brake',
            'refinement',
        ],
        default=None,
    )

    args = parser.parse_args()

    config = load_config(
        args.config
    )

    session_name = (
        args.session_name
    )

    if session_name is None:

        session_name = (
            'leader_'
            +
            datetime.now().strftime(
                '%Y%m%d_%H%M%S'
            )
        )

    tuner = AutoTuner(

        config=config,

        session_name=session_name,

        results_root=(
            args.results_root
        ),

        selected_stages=(
            args.stages
        ),
    )

    tuner.run()


if __name__ == '__main__':
    main()
