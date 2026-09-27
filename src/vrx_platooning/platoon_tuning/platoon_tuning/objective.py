import math


FAILURE_SCORE = 1_000_000.0


def normalized(
    value,
    reference,
):

    if reference <= 0.0:
        return value

    return value / reference


def score_tracking_trial(
    result,
    config,
):

    # =========================================================
    # Experiment-level failures
    # =========================================================

    if result is None:
        return FAILURE_SCORE

    if (
        config.get(
            'require_success',
            True,
        )
        and
        not result.get(
            'success',
            False,
        )
    ):

        return FAILURE_SCORE

    # =========================================================
    # Capture requirement
    # =========================================================

    capture = result.get(
        'capture',
        {},
    )

    if not capture.get(
        'success',
        False,
    ):

        return FAILURE_SCORE

    capture_time = capture.get(
        'capture_time_s'
    )

    if capture_time is None:
        return FAILURE_SCORE

    acceptance = config[
        'acceptance'
    ]

    maximum_capture_time = float(
        acceptance[
            'maximum_capture_time'
        ]
    )

    if (
        capture_time
        > maximum_capture_time
    ):

        return FAILURE_SCORE

    # =========================================================
    # Use POST-CAPTURE metrics
    # =========================================================

    tracking = result.get(
        'post_capture_tracking'
    )

    if tracking is None:
        return FAILURE_SCORE

    weights = config[
        'weights'
    ]

    target_rmse = float(
        acceptance[
            'tracking_rmse'
        ]
    )

    target_max_error = float(
        acceptance[
            'maximum_tracking_error'
        ]
    )

    position_rmse_term = normalized(
        tracking[
            'position_rmse_m'
        ],
        target_rmse,
    )

    maximum_error_term = normalized(
        tracking[
            'maximum_position_error_m'
        ],
        target_max_error,
    )

    heading_rmse_term = normalized(
        tracking[
            'heading_rmse_rad'
        ],
        0.15,
    )

    speed_rmse_term = normalized(
        tracking[
            'speed_rmse_mps'
        ],
        0.15,
    )

    score = (

        float(
            weights[
                'position_rmse'
            ]
        )
        * position_rmse_term

        +

        float(
            weights[
                'maximum_position_error'
            ]
        )
        * maximum_error_term

        +

        float(
            weights[
                'heading_rmse'
            ]
        )
        * heading_rmse_term

        +

        float(
            weights[
                'speed_rmse'
            ]
        )
        * speed_rmse_term
    )

    # =========================================================
    # Capture penalty
    # =========================================================

    score += (

        float(
            weights.get(
                'capture_time_penalty',
                0.0,
            )
        )

        *

        capture_time
    )

    # =========================================================
    # Recovery penalties
    # =========================================================

    recovery = result.get(
        'recovery',
        {},
    )

    score += (

        float(
            weights.get(
                'recovery_count_penalty',
                0.0,
            )
        )

        *

        float(
            recovery.get(
                'count',
                0,
            )
        )
    )

    score += (

        float(
            weights.get(
                'recovery_time_penalty',
                0.0,
            )
        )

        *

        float(
            recovery.get(
                'total_time_s',
                0.0,
            )
        )
    )

    return score


def passes_acceptance(
    result,
    config,
):

    if result is None:
        return False

    if (
        config.get(
            'require_success',
            True,
        )
        and
        not result.get(
            'success',
            False,
        )
    ):

        return False

    capture = result.get(
        'capture',
        {},
    )

    if not capture.get(
        'success',
        False,
    ):

        return False

    capture_time = capture.get(
        'capture_time_s'
    )

    if capture_time is None:
        return False

    acceptance = config[
        'acceptance'
    ]

    if (
        capture_time
        >
        float(
            acceptance[
                'maximum_capture_time'
            ]
        )
    ):

        return False

    tracking = result.get(
        'post_capture_tracking'
    )

    if tracking is None:
        return False

    if (
        tracking[
            'maximum_position_error_m'
        ]
        >
        float(
            acceptance[
                'maximum_tracking_error'
            ]
        )
    ):

        return False

    if (
        tracking[
            'position_rmse_m'
        ]
        >
        float(
            acceptance[
                'tracking_rmse'
            ]
        )
    ):

        return False

    return True


def combined_score(
    scenario_results,
    config,
):

    if not scenario_results:
        return FAILURE_SCORE

    scores = {

        name:
            score_tracking_trial(
                result,
                config,
            )

        for name, result
        in scenario_results.items()
    }

    # Worst scenario determines candidate score.
    return max(
        scores.values()
    )


def all_scenarios_pass(
    scenario_results,
    config,
):

    expected = set(
        config[
            'scenarios'
        ]
    )

    received = set(
        scenario_results.keys()
    )

    if expected != received:
        return False

    return all(

        passes_acceptance(
            result,
            config,
        )

        for result
        in scenario_results.values()
    )
