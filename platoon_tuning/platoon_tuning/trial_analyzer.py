import argparse
import csv
import json
import math
from pathlib import Path


# =============================================================
# Utilities
# =============================================================


def mean(values):

    if not values:
        return 0.0

    return sum(values) / len(values)


def rmse(values):

    if not values:
        return 0.0

    return math.sqrt(
        sum(
            value * value
            for value in values
        )
        / len(values)
    )


# =============================================================
# Trial analyzer
# =============================================================


class TrialAnalyzer:

    def __init__(
        self,
        target_speed=1.0,
        capture_error=1.0,
        capture_hold_time=0.5,
    ):

        self.target_speed = float(
            target_speed
        )

        self.capture_error = float(
            capture_error
        )

        self.capture_hold_time = float(
            capture_hold_time
        )

    # =========================================================
    # CSV loading
    # =========================================================

    def load_csv(
        self,
        csv_path,
    ):

        csv_path = Path(
            csv_path
        ).expanduser()

        if not csv_path.exists():

            raise FileNotFoundError(
                f'CSV file does not exist: '
                f'{csv_path}'
            )

        with csv_path.open(
            newline='',
            encoding='utf-8',
        ) as csv_file:

            rows = list(
                csv.DictReader(
                    csv_file
                )
            )

        if not rows:

            raise ValueError(
                f'CSV contains no data: '
                f'{csv_path}'
            )

        return rows

    # =========================================================
    # Numeric helpers
    # =========================================================

    @staticmethod
    def get_float(
        row,
        field,
    ):

        return float(
            row[
                field
            ]
        )

    # =========================================================
    # Tracking metrics
    # =========================================================

    def calculate_tracking_metrics(
        self,
        rows,
    ):

        if not rows:
            return None

        position_errors = [
            abs(
                self.get_float(
                    row,
                    'position_error_m',
                )
            )
            for row in rows
        ]

        heading_errors = [
            self.get_float(
                row,
                'heading_error_rad',
            )
            for row in rows
        ]

        speeds = [
            self.get_float(
                row,
                'speed_mps',
            )
            for row in rows
        ]

        speed_errors = [
            self.target_speed
            - speed
            for speed in speeds
        ]

        return {

            'samples':
                len(rows),

            'mean_position_error_m':
                mean(
                    position_errors
                ),

            'maximum_position_error_m':
                max(
                    position_errors
                ),

            'position_rmse_m':
                rmse(
                    position_errors
                ),

            'mean_abs_heading_error_rad':
                mean(
                    [
                        abs(error)
                        for error
                        in heading_errors
                    ]
                ),

            'heading_rmse_rad':
                rmse(
                    heading_errors
                ),

            'mean_speed_mps':
                mean(
                    speeds
                ),

            'speed_rmse_mps':
                rmse(
                    speed_errors
                ),
        }

    # =========================================================
    # Capture detection
    # =========================================================

    def find_capture(
        self,
        track_rows,
    ):

        if not track_rows:

            return None

        # -----------------------------------------------------
        # Capture means:
        #
        # The robot enters the capture-error band and then
        # remains continuously inside it for at least the
        # configured hold time.
        #
        # We return the START of that successful interval,
        # rather than the time at which the hold requirement
        # is finally confirmed.
        # -----------------------------------------------------

        candidate_start_index = None

        for index, row in enumerate(
            track_rows
        ):

            time_s = self.get_float(
                row,
                'time_s',
            )

            error = abs(
                self.get_float(
                    row,
                    'position_error_m',
                )
            )

            if (
                error
                <= self.capture_error
            ):

                if (
                    candidate_start_index
                    is None
                ):

                    candidate_start_index = (
                        index
                    )

                start_time = self.get_float(
                    track_rows[
                        candidate_start_index
                    ],
                    'time_s',
                )

                held_time = (
                    time_s
                    - start_time
                )

                if (
                    held_time
                    >= self.capture_hold_time
                ):

                    return {

                        'start_index':
                            candidate_start_index,

                        'start_time_s':
                            start_time,

                        'confirmation_time_s':
                            time_s,

                        'held_time_s':
                            held_time,
                    }

            else:

                # ---------------------------------------------
                # Robot left the capture band before satisfying
                # the required hold time.
                # ---------------------------------------------

                candidate_start_index = (
                    None
                )

        return None

    # =========================================================
    # State analysis
    # =========================================================

    def analyze_states(
        self,
        rows,
        has_state,
    ):

        transitions = []

        state_durations = {}

        state_entry_counts = {}

        if not has_state:

            return {
                'transitions':
                    transitions,

                'durations':
                    state_durations,

                'entry_counts':
                    state_entry_counts,
            }

        previous_state = None

        # -----------------------------------------------------
        # Record state transitions / entries
        # -----------------------------------------------------

        for row in rows:

            state = row.get(
                'controller_state',
                'UNKNOWN',
            )

            time_s = self.get_float(
                row,
                'time_s',
            )

            if (
                state
                != previous_state
            ):

                transitions.append(
                    {
                        'time_s':
                            time_s,

                        'state':
                            state,
                    }
                )

                state_entry_counts[
                    state
                ] = (
                    state_entry_counts.get(
                        state,
                        0,
                    )
                    + 1
                )

            previous_state = state

        # -----------------------------------------------------
        # Integrate time spent in each state.
        #
        # Each interval belongs to the state of the earlier
        # sample.
        # -----------------------------------------------------

        for index in range(
            len(rows) - 1
        ):

            current_row = rows[
                index
            ]

            next_row = rows[
                index + 1
            ]

            current_time = (
                self.get_float(
                    current_row,
                    'time_s',
                )
            )

            next_time = (
                self.get_float(
                    next_row,
                    'time_s',
                )
            )

            delta_time = (
                next_time
                - current_time
            )

            if delta_time < 0.0:

                continue

            state = current_row.get(
                'controller_state',
                'UNKNOWN',
            )

            state_durations[
                state
            ] = (
                state_durations.get(
                    state,
                    0.0,
                )
                + delta_time
            )

        return {

            'transitions':
                transitions,

            'durations':
                state_durations,

            'entry_counts':
                state_entry_counts,
        }

    # =========================================================
    # Main analysis
    # =========================================================

    def analyze(
        self,
        csv_path,
    ):

        rows = self.load_csv(
            csv_path
        )

        has_state = (
            'controller_state'
            in rows[0]
        )

        # =====================================================
        # TRACK rows
        # =====================================================

        if has_state:

            track_rows = [
                row
                for row in rows
                if row.get(
                    'controller_state',
                    '',
                )
                == 'TRACK'
            ]

        else:

            track_rows = rows

        if not track_rows:

            raise ValueError(
                'No TRACK rows found in CSV.'
            )

        # =====================================================
        # Full TRACK metrics
        #
        # These remain available for the project report and
        # backwards compatibility.
        # =====================================================

        full_tracking = (
            self.calculate_tracking_metrics(
                track_rows
            )
        )

        # =====================================================
        # Capture
        # =====================================================

        capture_result = (
            self.find_capture(
                track_rows
            )
        )

        first_track_time = (
            self.get_float(
                track_rows[0],
                'time_s',
            )
        )

        initial_position_error = abs(
            self.get_float(
                track_rows[0],
                'position_error_m',
            )
        )

        if capture_result is not None:

            capture_start_index = (
                capture_result[
                    'start_index'
                ]
            )

            capture_time = (
                capture_result[
                    'start_time_s'
                ]
                - first_track_time
            )

            capture_confirmation_time = (
                capture_result[
                    'confirmation_time_s'
                ]
                - first_track_time
            )

            post_capture_rows = (
                track_rows[
                    capture_start_index:
                ]
            )

            post_capture_tracking = (
                self.calculate_tracking_metrics(
                    post_capture_rows
                )
            )

            capture_success = True

        else:

            capture_time = None

            capture_confirmation_time = (
                None
            )

            post_capture_rows = []

            post_capture_tracking = None

            capture_success = False

        # =====================================================
        # Success
        # =====================================================

        final_state = (
            rows[-1].get(
                'controller_state',
                '',
            )
            if has_state
            else 'UNKNOWN'
        )

        success = (
            final_state
            == 'SUCCESS'
        )

        # =====================================================
        # Timing
        # =====================================================

        first_time = self.get_float(
            rows[0],
            'time_s',
        )

        final_time = self.get_float(
            rows[-1],
            'time_s',
        )

        total_time = (
            final_time
            - first_time
        )

        # =====================================================
        # State timing / transitions
        # =====================================================

        state_analysis = (
            self.analyze_states(
                rows,
                has_state,
            )
        )

        transitions = (
            state_analysis[
                'transitions'
            ]
        )

        state_durations = (
            state_analysis[
                'durations'
            ]
        )

        state_entry_counts = (
            state_analysis[
                'entry_counts'
            ]
        )

        recovery_count = (
            state_entry_counts.get(
                'RECOVER',
                0,
            )
        )

        recovery_time = (
            state_durations.get(
                'RECOVER',
                0.0,
            )
        )

        brake_duration = (
            state_durations.get(
                'BRAKE',
                0.0,
            )
        )

        track_duration = (
            state_durations.get(
                'TRACK',
                0.0,
            )
        )

        hold_duration = (
            state_durations.get(
                'HOLD',
                0.0,
            )
        )

        # =====================================================
        # Brake-specific data
        # =====================================================

        if has_state:

            brake_rows = [
                row
                for row in rows
                if row.get(
                    'controller_state',
                    '',
                )
                == 'BRAKE'
            ]

        else:

            brake_rows = []

        if brake_rows:

            brake_min_speed = min(
                self.get_float(
                    row,
                    'speed_mps',
                )
                for row
                in brake_rows
            )

        else:

            brake_min_speed = 0.0

        # =====================================================
        # Result
        # =====================================================

        result = {

            'csv_file':
                str(
                    Path(
                        csv_path
                    ).expanduser()
                ),

            'success':
                success,

            'final_state':
                final_state,

            'samples_total':
                len(rows),

            'samples_track':
                len(track_rows),

            'total_time_s':
                total_time,

            'track_time_s':
                track_duration,

            # -------------------------------------------------
            # FULL tracking metrics.
            #
            # Keep this key unchanged so existing code does not
            # break.
            # -------------------------------------------------

            'tracking':
                full_tracking,

            # -------------------------------------------------
            # Capture information
            # -------------------------------------------------

            'capture': {

                'success':
                    capture_success,

                'threshold_m':
                    self.capture_error,

                'required_hold_time_s':
                    self.capture_hold_time,

                'initial_position_error_m':
                    initial_position_error,

                'capture_time_s':
                    capture_time,

                'confirmation_time_s':
                    capture_confirmation_time,
            },

            # -------------------------------------------------
            # Metrics used later by the optimizer.
            #
            # These begin at the START of the first successful
            # capture interval.
            # -------------------------------------------------

            'post_capture_tracking':
                post_capture_tracking,

            'recovery': {

                'count':
                    recovery_count,

                'total_time_s':
                    recovery_time,
            },

            'brake': {

                'duration_s':
                    brake_duration,

                'minimum_speed_mps':
                    brake_min_speed,

                'episodes':
                    state_entry_counts.get(
                        'BRAKE',
                        0,
                    ),
            },

            'hold': {

                'duration_s':
                    hold_duration,
            },

            'state_durations_s':
                state_durations,

            'transitions':
                transitions,
        }

        return result


# =============================================================
# Command-line entry point
# =============================================================


def main():

    parser = argparse.ArgumentParser(
        description=(
            'Analyze one platooning '
            'trajectory CSV.'
        )
    )

    parser.add_argument(
        'csv_file',
        help='Trajectory CSV file',
    )

    parser.add_argument(
        '--target-speed',
        type=float,
        default=1.0,
        help=(
            'Target tracking speed '
            'in m/s.'
        ),
    )

    parser.add_argument(
        '--capture-error',
        type=float,
        default=1.0,
        help=(
            'Position error threshold '
            'used to detect trajectory capture.'
        ),
    )

    parser.add_argument(
        '--capture-hold-time',
        type=float,
        default=0.5,
        help=(
            'Time the robot must continuously '
            'remain inside the capture threshold.'
        ),
    )

    parser.add_argument(
        '--json',
        action='store_true',
        help='Print full JSON result.',
    )

    args = parser.parse_args()

    analyzer = TrialAnalyzer(

        target_speed=(
            args.target_speed
        ),

        capture_error=(
            args.capture_error
        ),

        capture_hold_time=(
            args.capture_hold_time
        ),
    )

    result = analyzer.analyze(
        args.csv_file
    )

    if args.json:

        print(
            json.dumps(
                result,
                indent=2,
            )
        )

        return

    tracking = result[
        'tracking'
    ]

    capture = result[
        'capture'
    ]

    post_capture = result[
        'post_capture_tracking'
    ]

    print()

    print(
        '============================================'
    )

    print(
        ' AUTOTUNER TRIAL ANALYSIS'
    )

    print(
        '============================================'
    )

    print(
        f"Success                 : "
        f"{result['success']}"
    )

    print(
        f"Final state             : "
        f"{result['final_state']}"
    )

    print(
        f"Total time              : "
        f"{result['total_time_s']:.3f} s"
    )

    print(
        f"TRACK duration          : "
        f"{result['track_time_s']:.3f} s"
    )

    print()

    print(
        '--- FULL TRACK ---'
    )

    print(
        f"Mean position error     : "
        f"{tracking['mean_position_error_m']:.3f} m"
    )

    print(
        f"Maximum position error  : "
        f"{tracking['maximum_position_error_m']:.3f} m"
    )

    print(
        f"Position RMSE           : "
        f"{tracking['position_rmse_m']:.3f} m"
    )

    print(
        f"Heading RMSE            : "
        f"{tracking['heading_rmse_rad']:.3f} rad"
    )

    print(
        f"Mean speed              : "
        f"{tracking['mean_speed_mps']:.3f} m/s"
    )

    print(
        f"Speed RMSE              : "
        f"{tracking['speed_rmse_mps']:.3f} m/s"
    )

    print()

    print(
        '--- CAPTURE ---'
    )

    print(
        f"Capture success         : "
        f"{capture['success']}"
    )

    print(
        f"Initial position error  : "
        f"{capture['initial_position_error_m']:.3f} m"
    )

    print(
        f"Capture threshold       : "
        f"{capture['threshold_m']:.3f} m"
    )

    print(
        f"Required hold time      : "
        f"{capture['required_hold_time_s']:.3f} s"
    )

    if capture[
        'success'
    ]:

        print(
            f"Capture time            : "
            f"{capture['capture_time_s']:.3f} s"
        )

        print(
            f"Capture confirmed       : "
            f"{capture['confirmation_time_s']:.3f} s"
        )

    else:

        print(
            'Capture time            : '
            'NOT CAPTURED'
        )

    print()

    print(
        '--- POST-CAPTURE TRACK ---'
    )

    if post_capture is not None:

        print(
            f"Samples                  : "
            f"{post_capture['samples']}"
        )

        print(
            f"Mean position error      : "
            f"{post_capture['mean_position_error_m']:.3f} m"
        )

        print(
            f"Maximum position error   : "
            f"{post_capture['maximum_position_error_m']:.3f} m"
        )

        print(
            f"Position RMSE            : "
            f"{post_capture['position_rmse_m']:.3f} m"
        )

        print(
            f"Heading RMSE             : "
            f"{post_capture['heading_rmse_rad']:.3f} rad"
        )

        print(
            f"Speed RMSE               : "
            f"{post_capture['speed_rmse_mps']:.3f} m/s"
        )

    else:

        print(
            'No post-capture metrics available.'
        )

    print()

    print(
        '--- TERMINAL CONTROL ---'
    )

    print(
        f"Recovery count          : "
        f"{result['recovery']['count']}"
    )

    print(
        f"Recovery time           : "
        f"{result['recovery']['total_time_s']:.3f} s"
    )

    print(
        f"Brake episodes          : "
        f"{result['brake']['episodes']}"
    )

    print(
        f"Brake duration          : "
        f"{result['brake']['duration_s']:.3f} s"
    )

    print(
        'State sequence          : '
        +
        ' -> '.join(
            transition[
                'state'
            ]
            for transition
            in result[
                'transitions'
            ]
        )
    )

    print(
        '============================================'
    )

    print()


if __name__ == '__main__':

    main()
