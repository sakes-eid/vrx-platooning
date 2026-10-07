#!/usr/bin/env python3

"""
Generic interactive point selection on the Sydney navigation map.

Used by:
    Straight:
        start -> end

    Curve:
        start -> apogee -> end

Controls:
    Left drag   = pan
    Mouse wheel = zoom
    Right click = select point

Only ordinary safe-water cells may be selected.
"""

import matplotlib.pyplot as plt
import numpy as np

from matplotlib.colors import (
    ListedColormap,
)

from platoon_planner.sydney_environment import (
    ned_to_grid,
)


def display_map(
    raw_grid,
    safe_grid,
):
    """
    Display encoding:

        0 = safe water
        1 = WAM-V safety buffer
        2 = collision-relevant terrain
    """

    display = np.zeros_like(
        raw_grid
    )

    display[
        (safe_grid == 1)
        & (raw_grid == 0)
    ] = 1

    display[
        raw_grid == 1
    ] = 2

    return display


def select_mission_points(
    raw_grid,
    safe_grid,
    metadata,
    labels,
    title,
):
    """
    Select an ordered sequence of safe world_ned points.

    labels:
        Example:
            ("Start", "End")

        or:
            ("Start", "Apogee", "End")

    Returns:
        list of (North, East)
    """

    if not labels:

        raise ValueError(
            "At least one point label is required."
        )

    display = display_map(
        raw_grid,
        safe_grid,
    )

    cmap = ListedColormap(
        [
            "white",
            "orange",
            "black",
        ]
    )

    fig, ax = plt.subplots(
        figsize=(
            15,
            9,
        )
    )

    ax.imshow(
        display,
        origin="lower",
        extent=[
            metadata[
                "east_min"
            ],
            metadata[
                "east_max"
            ],
            metadata[
                "north_min"
            ],
            metadata[
                "north_max"
            ],
        ],
        interpolation="nearest",
        aspect="equal",
        cmap=cmap,
        vmin=0,
        vmax=2,
    )

    ax.set_xlabel(
        "East [m]"
    )

    ax.set_ylabel(
        "North [m]"
    )

    ax.set_title(
        title
        + "\n"
        + "Right-click points in order | "
        + "Left drag = pan | "
        + "Wheel = zoom"
    )

    selected = []

    drag_state = None

    # ---------------------------------------------------------
    # Mouse press
    # ---------------------------------------------------------

    def on_press(
        event,
    ):

        nonlocal drag_state

        if event.inaxes != ax:
            return

        # Left button:
        # begin pan.
        if event.button == 1:

            drag_state = (
                event.x,
                event.y,
                ax.get_xlim(),
                ax.get_ylim(),
            )

            return

        # Right button:
        # select point.
        if event.button != 3:
            return

        if (
            event.xdata is None
            or
            event.ydata is None
        ):
            return

        if len(
            selected
        ) >= len(
            labels
        ):
            return

        # Matplotlib axes:
        #
        # x = East
        # y = North
        north = float(
            event.ydata
        )

        east = float(
            event.xdata
        )

        row, column = (
            ned_to_grid(
                north,
                east,
                metadata,
            )
        )

        if not (
            0
            <= row
            < safe_grid.shape[0]
            and
            0
            <= column
            < safe_grid.shape[1]
        ):

            print(
                "Point ignored: outside navigation map."
            )

            return

        if safe_grid[
            row,
            column,
        ] != 0:

            print(
                "Point ignored: select ordinary safe water."
            )

            return

        label = labels[
            len(
                selected
            )
        ]

        point = (
            north,
            east,
        )

        selected.append(
            point
        )

        print(
            f"{label}: "
            f"North={north:.2f}, "
            f"East={east:.2f}"
        )

        ax.scatter(
            east,
            north,
            s=90,
            marker="x",
            label=label,
        )

        # Connect selected points visually.
        if len(
            selected
        ) >= 2:

            ax.plot(
                [
                    point[
                        1
                    ]
                    for point
                    in selected
                ],
                [
                    point[
                        0
                    ]
                    for point
                    in selected
                ],
                linestyle="--",
                linewidth=2.0,
            )

        ax.legend()

        fig.canvas.draw_idle()

        if len(
            selected
        ) == len(
            labels
        ):

            plt.pause(
                0.5
            )

            plt.close(
                fig
            )

    # ---------------------------------------------------------
    # Pan
    # ---------------------------------------------------------

    def on_motion(
        event,
    ):

        nonlocal drag_state

        if (
            drag_state is None
            or
            event.inaxes != ax
        ):
            return

        if (
            event.x is None
            or
            event.y is None
        ):
            return

        (
            start_x,
            start_y,
            x_limits,
            y_limits,
        ) = drag_state

        x0, x1 = (
            x_limits
        )

        y0, y1 = (
            y_limits
        )

        dx_pixels = (
            event.x
            - start_x
        )

        dy_pixels = (
            event.y
            - start_y
        )

        east_per_pixel = (
            (x1 - x0)
            / ax.bbox.width
        )

        north_per_pixel = (
            (y1 - y0)
            / ax.bbox.height
        )

        ax.set_xlim(
            x0
            - dx_pixels
            * east_per_pixel,

            x1
            - dx_pixels
            * east_per_pixel,
        )

        ax.set_ylim(
            y0
            - dy_pixels
            * north_per_pixel,

            y1
            - dy_pixels
            * north_per_pixel,
        )

        fig.canvas.draw_idle()

    def on_release(
        event,
    ):

        nonlocal drag_state

        if event.button == 1:

            drag_state = None

    # ---------------------------------------------------------
    # Zoom
    # ---------------------------------------------------------

    def on_scroll(
        event,
    ):

        if event.inaxes != ax:
            return

        if (
            event.xdata is None
            or
            event.ydata is None
        ):
            return

        x0, x1 = (
            ax.get_xlim()
        )

        y0, y1 = (
            ax.get_ylim()
        )

        factor = (
            0.85
            if event.button == "up"
            else 1.15
        )

        width = (
            x1 - x0
        )

        height = (
            y1 - y0
        )

        x_fraction = (
            (event.xdata - x0)
            / width
        )

        y_fraction = (
            (event.ydata - y0)
            / height
        )

        new_width = (
            width
            * factor
        )

        new_height = (
            height
            * factor
        )

        ax.set_xlim(
            event.xdata
            - x_fraction
            * new_width,

            event.xdata
            + (
                1.0
                - x_fraction
            )
            * new_width,
        )

        ax.set_ylim(
            event.ydata
            - y_fraction
            * new_height,

            event.ydata
            + (
                1.0
                - y_fraction
            )
            * new_height,
        )

        fig.canvas.draw_idle()

    # ---------------------------------------------------------
    # Events
    # ---------------------------------------------------------

    fig.canvas.mpl_connect(
        "button_press_event",
        on_press,
    )

    fig.canvas.mpl_connect(
        "motion_notify_event",
        on_motion,
    )

    fig.canvas.mpl_connect(
        "button_release_event",
        on_release,
    )

    fig.canvas.mpl_connect(
        "scroll_event",
        on_scroll,
    )

    plt.show()

    if len(
        selected
    ) != len(
        labels
    ):

        raise RuntimeError(
            "Mission point selection cancelled."
        )

    return selected
