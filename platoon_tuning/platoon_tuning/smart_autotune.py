import argparse
import csv
import json
import math
import shutil

from copy import deepcopy
from pathlib import Path

import optuna
import yaml

from optuna.distributions import FloatDistribution
from optuna.trial import TrialState

from platoon_tuning.autotune import (
    AutoTuner,
    load_config,
    save_best_yaml,
)

from platoon_tuning.objective import (
    FAILURE_SCORE,
    all_scenarios_pass,
    combined_score,
    score_tracking_trial,
)


PARAMS = [
    "heading_kp",
    "heading_ki",
    "heading_kd",
    "lookahead_distance",

    "speed_kp",
    "speed_ki",
    "speed_kd",

    "brake_kp",
    "brake_ki",
    "brake_kd",
]


def load_best(path):

    data = yaml.safe_load(
        Path(path).read_text()
    )

    params = (
        data[
            "leader_pid_controller"
        ][
            "ros__parameters"
        ]
    )

    return {
        key: float(value)
        for key, value
        in params.items()
    }


def seed_score(
    summary,
    history,
):

    path = Path(
        summary
    )

    if path.exists():

        data = json.loads(
            path.read_text()
        )

        if "best_score" in data:

            return float(
                data["best_score"]
            )

    values = []

    with Path(history).open(
        newline="",
        encoding="utf-8",
    ) as file:

        for row in csv.DictReader(
            file
        ):

            try:

                values.append(
                    float(
                        row["score"]
                    )
                )

            except (
                KeyError,
                TypeError,
                ValueError,
            ):

                pass

    if not values:

        raise RuntimeError(
            "No valid seed score."
        )

    return min(
        values
    )


def space_for(
    config,
    stage,
    base,
):

    if stage != "refinement":

        return deepcopy(
            config[
                "leader"
            ][
                stage
            ][
                "parameters"
            ]
        )

    relative_range = float(
        config[
            "leader"
        ][
            "refinement"
        ][
            "relative_range"
        ]
    )

    space = {}

    for name in PARAMS:

        value = float(
            base[name]
        )

        if name == "lookahead_distance":

            lower = max(
                0.5,
                value
                * (
                    1.0
                    - relative_range
                ),
            )

            upper = (
                value
                * (
                    1.0
                    + relative_range
                )
            )

        elif value == 0.0:

            lower = 0.0

            upper = (
                10.0
                if name.endswith(
                    "_ki"
                )
                else 30.0
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

            upper = (
                value
                * (
                    1.0
                    + relative_range
                )
            )

        space[
            name
        ] = {
            "min":
                lower,

            "max":
                max(
                    upper,
                    lower + 1e-6,
                ),
        }

    return space


def distributions(
    space,
):

    return {

        name:
            FloatDistribution(
                float(
                    bounds[
                        "min"
                    ]
                ),
                float(
                    bounds[
                        "max"
                    ]
                ),
            )

        for name, bounds
        in space.items()
    }


def import_heading(
    study,
    history,
    space,
):

    distributions_map = (
        distributions(
            space
        )
    )

    count = 0

    with Path(history).open(
        newline="",
        encoding="utf-8",
    ) as file:

        for row in csv.DictReader(
            file
        ):

            if (
                row.get(
                    "stage"
                )
                != "heading"
            ):

                continue

            try:

                value = float(
                    row[
                        "score"
                    ]
                )

                params = {

                    name:
                        float(
                            row[
                                name
                            ]
                        )

                    for name
                    in space
                }

            except (
                KeyError,
                TypeError,
                ValueError,
            ):

                continue

            if not math.isfinite(
                value
            ):

                continue

            outside = any(

                params[name]
                <
                float(
                    space[
                        name
                    ][
                        "min"
                    ]
                )

                or

                params[name]
                >
                float(
                    space[
                        name
                    ][
                        "max"
                    ]
                )

                for name
                in params
            )

            if outside:

                continue

            completed = (
                optuna.trial.create_trial(

                    params=params,

                    distributions=(
                        distributions_map
                    ),

                    value=value,

                    state=(
                        TrialState.COMPLETE
                    ),
                )
            )

            study.add_trial(
                completed
            )

            count += 1

    return count


def add_seed(
    study,
    space,
    best,
    score,
):

    completed = (
        optuna.trial.create_trial(

            params={
                name:
                    float(
                        best[
                            name
                        ]
                    )

                for name
                in space
            },

            distributions=(
                distributions(
                    space
                )
            ),

            value=float(
                score
            ),

            state=(
                TrialState.COMPLETE
            ),
        )
    )

    study.add_trial(
        completed
    )


def get_analysis(
    result,
):

    if (

        result

        and

        result.get(
            "status"
        )
        == "SUCCESS"

        and

        result.get(
            "analysis"
        )

    ):

        return result[
            "analysis"
        ]

    return None


def write_summary(
    tuner,
    seed,
    seed_history,
):

    data = {

        "session_name":
            tuner.session_name,

        "method":
            (
                "Optuna multivariate TPE "
                "+ prior trials "
                "+ curved-first pruning"
            ),

        "seed_score":
            seed,

        "seed_history":
            str(
                seed_history
            ),

        "best_score":
            tuner.current_best_score,

        "best_candidate":
            tuner.current_best,

        "new_history_rows":
            len(
                tuner.history
            ),
    }

    tuner.summary_json.write_text(
        json.dumps(
            data,
            indent=2,
        )
    )


def main():

    base = Path(
        "/home/sajed/vrx_ws/src/"
        "vrx_platooning"
    )

    old = (
        base
        /
        "results/autotune_sessions/"
        "leader_autotune_full_01"
    )

    parser = (
        argparse.ArgumentParser()
    )

    parser.add_argument(
        "--config",
        default=str(
            base
            /
            "platoon_tuning/config/"
            "autotune.yaml"
        ),
    )

    parser.add_argument(
        "--session-name",
        default="leader_smart_01",
    )

    parser.add_argument(
        "--results-root",
        default=str(
            base
            /
            "results/autotune_sessions"
        ),
    )

    parser.add_argument(
        "--seed-history",
        default=str(
            old
            /
            "history.csv"
        ),
    )

    parser.add_argument(
        "--seed-best",
        default=str(
            old
            /
            "best_pid.yaml"
        ),
    )

    parser.add_argument(
        "--seed-summary",
        default=str(
            old
            /
            "summary.json"
        ),
    )

    parser.add_argument(
        "--stages",
        nargs="+",
        choices=[
            "heading",
            "speed",
            "brake",
            "refinement",
        ],
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
    )

    args = (
        parser.parse_args()
    )

    config = load_config(
        args.config
    )

    best = load_best(
        args.seed_best
    )

    start_score = seed_score(
        args.seed_summary,
        args.seed_history,
    )

    budgets = config.get(

        "smart_trials",

        {
            "heading": 12,
            "speed": 15,
            "brake": 10,
            "refinement": 20,
        },
    )

    if args.dry_run:

        print(
            f"Seed score: "
            f"{start_score:.6f}"
        )

        print(
            f"Budgets: "
            f"{budgets}"
        )

        for name in PARAMS:

            print(
                f"{name:<24} "
                f"{best[name]:.6f}"
            )

        return

    tuner = AutoTuner(
        config,
        args.session_name,
        args.results_root,
        args.stages,
    )

    tuner.current_best = (
        deepcopy(
            best
        )
    )

    tuner.current_best_score = (
        start_score
    )

    save_best_yaml(
        tuner.best_yaml,
        tuner.current_best,
    )

    shutil.copy2(
        args.seed_history,
        tuner.session_dir
        /
        "seed_history.csv",
    )

    shutil.copy2(
        args.seed_best,
        tuner.session_dir
        /
        "seed_best_pid.yaml",
    )

    shutil.copy2(
        args.seed_summary,
        tuner.session_dir
        /
        "seed_summary.json",
    )

    optuna.logging.set_verbosity(
        optuna.logging.WARNING
    )

    print(
        "=" * 60
    )

    print(
        "SMART PID OPTIMIZER"
    )

    print(
        f"Seed score : "
        f"{tuner.current_best_score:.6f}"
    )

    print(
        "Method     : "
        "multivariate TPE "
        "+ prior data "
        "+ safe pruning"
    )

    print(
        "=" * 60
    )

    for stage in tuner.stages:

        stage_base = deepcopy(
            tuner.current_best
        )

        space = space_for(
            config,
            stage,
            stage_base,
        )

        sampler = (
            optuna.samplers.TPESampler(

                seed=int(
                    config.get(
                        "random_seed",
                        42,
                    )
                ),

                n_startup_trials=5,

                multivariate=True,
            )
        )

        study = (
            optuna.create_study(

                direction="minimize",

                sampler=sampler,
            )
        )

        if stage == "heading":

            imported = import_heading(
                study,
                args.seed_history,
                space,
            )

            print(
                f"\nImported "
                f"{imported} "
                f"previous heading trials."
            )

            if imported == 0:

                add_seed(
                    study,
                    space,
                    tuner.current_best,
                    tuner.current_best_score,
                )

        else:

            add_seed(
                study,
                space,
                tuner.current_best,
                tuner.current_best_score,
            )

        print(
            f"\n--- "
            f"{stage.upper()} "
            f"| "
            f"{int(budgets[stage])} "
            f"guided trials ---"
        )

        def objective(
            trial,
        ):

            candidate = deepcopy(
                stage_base
            )

            for name, bounds in (
                space.items()
            ):

                candidate[
                    name
                ] = (
                    trial.suggest_float(

                        name,

                        float(
                            bounds[
                                "min"
                            ]
                        ),

                        float(
                            bounds[
                                "max"
                            ]
                        ),
                    )
                )

            print(
                f"\n"
                f"{stage.upper()} "
                f"trial {trial.number} "
                f"| best "
                f"{tuner.current_best_score:.6f}"
            )

            curved_result = (
                tuner.run_scenario(

                    stage,
                    trial.number,
                    "curved",
                    candidate,
                )
            )

            curved = get_analysis(
                curved_result
            )

            if curved is None:

                evaluation = {

                    "candidate":
                        candidate,

                    "scenario_results":
                        {},

                    "score":
                        FAILURE_SCORE,

                    "passed":
                        False,
                }

                tuner.record_history(
                    stage,
                    trial.number,
                    evaluation,
                    False,
                )

                return FAILURE_SCORE

            curved_score = (
                score_tracking_trial(
                    curved,
                    config,
                )
            )

            trial.report(
                curved_score,
                0,
            )

            # ---------------------------------------------
            # Mathematically safe pruning:
            #
            # final_score = max(curved, straight)
            #
            # Therefore if curved already cannot beat the
            # incumbent, straight can never rescue it.
            # ---------------------------------------------

            if (
                curved_score
                >=
                tuner.current_best_score
            ):

                evaluation = {

                    "candidate":
                        candidate,

                    "scenario_results":
                        {
                            "curved":
                                curved
                        },

                    "score":
                        curved_score,

                    "passed":
                        False,
                }

                tuner.record_history(
                    stage,
                    trial.number,
                    evaluation,
                    False,
                )

                print(
                    "PRUNED after curved: "
                    f"{curved_score:.6f} "
                    ">= "
                    f"{tuner.current_best_score:.6f}"
                )

                raise (
                    optuna.TrialPruned()
                )

            straight_result = (
                tuner.run_scenario(

                    stage,
                    trial.number,
                    "straight",
                    candidate,
                )
            )

            straight = get_analysis(
                straight_result
            )

            if straight is None:

                evaluation = {

                    "candidate":
                        candidate,

                    "scenario_results":
                        {
                            "curved":
                                curved
                        },

                    "score":
                        FAILURE_SCORE,

                    "passed":
                        False,
                }

                tuner.record_history(
                    stage,
                    trial.number,
                    evaluation,
                    False,
                )

                return FAILURE_SCORE

            results = {

                "curved":
                    curved,

                "straight":
                    straight,
            }

            score = combined_score(
                results,
                config,
            )

            passed = (
                all_scenarios_pass(
                    results,
                    config,
                )
            )

            improved = (
                score
                <
                tuner.current_best_score
            )

            if improved:

                old_score = (
                    tuner.current_best_score
                )

                tuner.current_best_score = (
                    float(
                        score
                    )
                )

                tuner.current_best = (
                    deepcopy(
                        candidate
                    )
                )

                save_best_yaml(
                    tuner.best_yaml,
                    tuner.current_best,
                )

                print(
                    "NEW BEST: "
                    f"{old_score:.6f} "
                    "-> "
                    f"{score:.6f}"
                )

            else:

                print(
                    f"Score "
                    f"{score:.6f} "
                    "| best "
                    f"{tuner.current_best_score:.6f}"
                )

            evaluation = {

                "candidate":
                    candidate,

                "scenario_results":
                    results,

                "score":
                    score,

                "passed":
                    passed,
            }

            tuner.record_history(
                stage,
                trial.number,
                evaluation,
                improved,
            )

            write_summary(
                tuner,
                start_score,
                args.seed_history,
            )

            return score

        study.optimize(

            objective,

            n_trials=int(
                budgets[
                    stage
                ]
            ),
        )

        print(
            f"{stage.upper()} complete "
            f"| best "
            f"{tuner.current_best_score:.6f}"
        )

    write_summary(
        tuner,
        start_score,
        args.seed_history,
    )

    print(
        "\n"
        + "=" * 60
    )

    print(
        "SMART OPTIMIZATION COMPLETE"
    )

    print(
        f"Best score : "
        f"{tuner.current_best_score:.6f}"
    )

    print(
        f"Best PID   : "
        f"{tuner.best_yaml}"
    )

    print(
        f"History    : "
        f"{tuner.history_file}"
    )

    print(
        "=" * 60
    )


if __name__ == "__main__":

    main()
