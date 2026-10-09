#!/usr/bin/env python3

"""
Graphical startup configuration for the VRX platooning system.

The GUI only creates and validates StartupConfig.
It does not launch Gazebo, ROS nodes, or tuners.
"""

import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

from platoon_bringup.startup_config import StartupConfig

from platoon_planner.mission_storage import (
    list_saved_missions,
)


class StartupGUI:

    def __init__(self):

        self.result = None

        self.root = tk.Tk()
        self.root.title("VRX Platooning — Run Configuration")
        self.root.geometry("760x920")
        self.root.minsize(700, 760)

        # -----------------------------------------------------
        # Variables
        # -----------------------------------------------------

        self.robot_count = tk.IntVar(value=1)

        self.mission_source = tk.StringVar(
            value="new"
        )

        self.mission_type = tk.StringVar(
            value="stress"
        )

        self.saved_mission = tk.StringVar(
            value=""
        )

        self.save_after_accept = tk.BooleanVar(
            value=False
        )

        self.save_name = tk.StringVar(
            value=""
        )

        self.run_mode = tk.StringVar(
            value="test"
        )

        self.headless = tk.BooleanVar(
            value=False
        )

        self.show_rviz = tk.BooleanVar(
            value=True
        )

        self.show_map = tk.BooleanVar(
            value=True
        )

        self.max_speed = tk.StringVar(
            value="2.0"
        )

        self.follower_spacing = tk.StringVar(
            value="5.0"
        )

        self.logging_enabled = tk.BooleanVar(
            value=True
        )

        default_results = str(
            Path.home()
            / "vrx_ws"
            / "src"
            / "vrx_platooning"
            / "results"
        )

        self.results_dir = tk.StringVar(
            value=default_results
        )

        self.run_name = tk.StringVar()

        self.tuning_target = tk.StringVar(
            value="r1"
        )

        self.tuning_trials = tk.StringVar(
            value=""
        )

        self.tuning_timeout = tk.StringVar(
            value=""
        )

        self.saved_items = {}

        self._build()
        self._load_saved_missions()
        self._refresh()
        self._update_run_name()

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self._cancel,
        )

    # =========================================================
    # Layout
    # =========================================================

    def _build(self):

        outer = ttk.Frame(
            self.root,
            padding=14,
        )

        outer.pack(
            fill="both",
            expand=True,
        )

        title = ttk.Label(
            outer,
            text="VRX Platooning Run Configuration",
            font=("Arial", 16, "bold"),
        )

        title.pack(
            anchor="w",
            pady=(0, 12),
        )

        # -----------------------------------------------------
        # Platoon
        # -----------------------------------------------------

        platoon = ttk.LabelFrame(
            outer,
            text="Platoon",
            padding=10,
        )

        platoon.pack(
            fill="x",
            pady=5,
        )

        ttk.Label(
            platoon,
            text="Robots:",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        for index, count in enumerate(
            (1, 2, 3),
            start=1,
        ):

            ttk.Radiobutton(
                platoon,
                text=str(count),
                variable=self.robot_count,
                value=count,
                command=self._configuration_changed,
            ).grid(
                row=0,
                column=index,
                padx=8,
            )

        ttk.Label(
            platoon,
            text="Follower spacing [m]:",
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        self.spacing_entry = ttk.Entry(
            platoon,
            textvariable=self.follower_spacing,
            width=12,
        )

        self.spacing_entry.grid(
            row=1,
            column=1,
            columnspan=2,
            sticky="w",
            pady=(8, 0),
        )

        # -----------------------------------------------------
        # Mission
        # -----------------------------------------------------

        mission = ttk.LabelFrame(
            outer,
            text="Mission",
            padding=10,
        )

        mission.pack(
            fill="x",
            pady=5,
        )

        ttk.Label(
            mission,
            text="Source:",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.new_radio = ttk.Radiobutton(
            mission,
            text="New",
            variable=self.mission_source,
            value="new",
            command=self._configuration_changed,
        )

        self.new_radio.grid(
            row=0,
            column=1,
            sticky="w",
            padx=8,
        )

        self.saved_radio = ttk.Radiobutton(
            mission,
            text="Saved",
            variable=self.mission_source,
            value="saved",
            command=self._configuration_changed,
        )

        self.saved_radio.grid(
            row=0,
            column=2,
            sticky="w",
            padx=8,
        )

        ttk.Label(
            mission,
            text="Mission type:",
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        self.mission_combo = ttk.Combobox(
            mission,
            textvariable=self.mission_type,
            state="readonly",
            values=(
                "straight",
                "curve",
                "coverage",
                "stress",
            ),
            width=20,
        )

        self.mission_combo.grid(
            row=1,
            column=1,
            columnspan=2,
            sticky="w",
            padx=8,
            pady=(8, 0),
        )

        self.mission_combo.bind(
            "<<ComboboxSelected>>",
            lambda _event:
                self._configuration_changed(),
        )

        ttk.Label(
            mission,
            text="Saved mission:",
        ).grid(
            row=2,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        self.saved_combo = ttk.Combobox(
            mission,
            textvariable=self.saved_mission,
            state="disabled",
            width=42,
        )

        self.saved_combo.grid(
            row=2,
            column=1,
            columnspan=3,
            sticky="ew",
            padx=8,
            pady=(8, 0),
        )

        self.save_check = ttk.Checkbutton(
            mission,
            text="Save mission after validation and acceptance",
            variable=self.save_after_accept,
            command=self._refresh,
        )

        self.save_check.grid(
            row=3,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(9, 0),
        )

        ttk.Label(
            mission,
            text="Save as:",
        ).grid(
            row=4,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        self.save_name_entry = ttk.Entry(
            mission,
            textvariable=self.save_name,
            width=28,
        )

        self.save_name_entry.grid(
            row=4,
            column=1,
            columnspan=2,
            sticky="w",
            padx=8,
            pady=(8, 0),
        )

        mission.columnconfigure(
            3,
            weight=1,
        )

        # -----------------------------------------------------
        # Run
        # -----------------------------------------------------

        run = ttk.LabelFrame(
            outer,
            text="Run",
            padding=10,
        )

        run.pack(
            fill="x",
            pady=5,
        )

        ttk.Label(
            run,
            text="Mode:",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        ttk.Radiobutton(
            run,
            text="Test",
            variable=self.run_mode,
            value="test",
            command=self._configuration_changed,
        ).grid(
            row=0,
            column=1,
            sticky="w",
            padx=8,
        )

        ttk.Radiobutton(
            run,
            text="Tuning",
            variable=self.run_mode,
            value="tuning",
            command=self._configuration_changed,
        ).grid(
            row=0,
            column=2,
            sticky="w",
            padx=8,
        )

        ttk.Label(
            run,
            text="Maximum speed [m/s]:",
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        ttk.Entry(
            run,
            textvariable=self.max_speed,
            width=12,
        ).grid(
            row=1,
            column=1,
            sticky="w",
            padx=8,
            pady=(8, 0),
        )

        ttk.Label(
            run,
            text="Run name:",
        ).grid(
            row=2,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        ttk.Entry(
            run,
            textvariable=self.run_name,
            width=42,
        ).grid(
            row=2,
            column=1,
            columnspan=3,
            sticky="ew",
            padx=8,
            pady=(8, 0),
        )

        run.columnconfigure(
            3,
            weight=1,
        )

        # -----------------------------------------------------
        # Display
        # -----------------------------------------------------

        display = ttk.LabelFrame(
            outer,
            text="Simulation / Display",
            padding=10,
        )

        display.pack(
            fill="x",
            pady=5,
        )

        ttk.Checkbutton(
            display,
            text="Gazebo headless",
            variable=self.headless,
        ).grid(
            row=0,
            column=0,
            sticky="w",
            padx=(0, 18),
        )

        ttk.Checkbutton(
            display,
            text="Launch RViz",
            variable=self.show_rviz,
            command=self._refresh,
        ).grid(
            row=0,
            column=1,
            sticky="w",
            padx=(0, 18),
        )

        self.map_check = ttk.Checkbutton(
            display,
            text="Show Sydney map",
            variable=self.show_map,
        )

        self.map_check.grid(
            row=0,
            column=2,
            sticky="w",
        )

        # -----------------------------------------------------
        # Tuning
        # -----------------------------------------------------

        tuning = ttk.LabelFrame(
            outer,
            text="Tuning",
            padding=10,
        )

        tuning.pack(
            fill="x",
            pady=5,
        )

        ttk.Label(
            tuning,
            text="Target:",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.tuning_combo = ttk.Combobox(
            tuning,
            textvariable=self.tuning_target,
            state="disabled",
            width=18,
        )

        self.tuning_combo.grid(
            row=0,
            column=1,
            sticky="w",
            padx=8,
        )

        ttk.Label(
            tuning,
            text="Optimization trials:",
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        self.trials_entry = ttk.Entry(
            tuning,
            textvariable=self.tuning_trials,
            width=12,
        )

        self.trials_entry.grid(
            row=1,
            column=1,
            sticky="w",
            padx=8,
            pady=(8, 0),
        )

        ttk.Label(
            tuning,
            text="Per-trial timeout [s]:",
        ).grid(
            row=1,
            column=2,
            sticky="w",
            pady=(8, 0),
        )

        self.timeout_entry = ttk.Entry(
            tuning,
            textvariable=self.tuning_timeout,
            width=12,
        )

        self.timeout_entry.grid(
            row=1,
            column=3,
            sticky="w",
            padx=8,
            pady=(8, 0),
        )

        self.tuning_note = ttk.Label(
            tuning,
            text=(
                "Tuning uses the selected saved mission. "
                "One seed validation run is added before "
                "the optimization trials."
            ),
        )

        self.tuning_note.grid(
            row=2,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(8, 0),
        )

        # -----------------------------------------------------
        # Logging
        # -----------------------------------------------------

        logging = ttk.LabelFrame(
            outer,
            text="Logging",
            padding=10,
        )

        logging.pack(
            fill="x",
            pady=5,
        )

        ttk.Checkbutton(
            logging,
            text="Enable result logging",
            variable=self.logging_enabled,
            command=self._refresh,
        ).grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="w",
        )

        ttk.Label(
            logging,
            text="Results directory:",
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=(8, 0),
        )

        self.results_entry = ttk.Entry(
            logging,
            textvariable=self.results_dir,
            width=55,
        )

        self.results_entry.grid(
            row=1,
            column=1,
            sticky="ew",
            padx=8,
            pady=(8, 0),
        )

        logging.columnconfigure(
            1,
            weight=1,
        )

        # -----------------------------------------------------
        # Buttons
        # -----------------------------------------------------

        buttons = ttk.Frame(
            outer,
        )

        buttons.pack(
            fill="x",
            pady=(14, 0),
        )

        ttk.Button(
            buttons,
            text="Cancel",
            command=self._cancel,
        ).pack(
            side="right",
            padx=(8, 0),
        )

        ttk.Button(
            buttons,
            text="START RUN",
            command=self._submit,
        ).pack(
            side="right",
        )

    # =========================================================
    # Dynamic state
    # =========================================================

    def _configuration_changed(self):

        self._refresh()
        self._update_run_name()

    def _update_run_name(self):

        config = StartupConfig()

        config.robot_count = (
            self.robot_count.get()
        )

        config.mission_type = (
            self.mission_type.get()
        )

        config.run_mode = (
            self.run_mode.get()
        )

        self.run_name.set(
            config.default_run_name()
        )

    def _load_saved_missions(self):

        missions, _invalid = (
            list_saved_missions()
        )

        labels = []

        self.saved_items = {}

        for item in missions:

            label = (
                f"{item['name']} "
                f"[{item['mission_type']}]"
            )

            labels.append(
                label
            )

            self.saved_items[
                label
            ] = item

        self.saved_combo.configure(
            values=labels
        )

        if labels:

            self.saved_mission.set(
                labels[0]
            )

    def _refresh(self):

        tuning = (
            self.run_mode.get()
            == "tuning"
        )

        # Tuning uses one frozen saved mission so the seed
        # and every optimization trial run identical geometry.
        if tuning:

            self.mission_source.set(
                "saved"
            )

            self.new_radio.configure(
                state="disabled"
            )

            self.saved_radio.configure(
                state="disabled"
            )

            self.mission_combo.configure(
                state="disabled"
            )

            self.saved_combo.configure(
                state="readonly"
            )

            self.save_check.configure(
                state="disabled"
            )

            self.save_name_entry.configure(
                state="disabled"
            )

        else:

            self.new_radio.configure(
                state="normal"
            )

            self.saved_radio.configure(
                state="normal"
            )

            source = (
                self.mission_source.get()
            )

            if source == "new":

                self.mission_combo.configure(
                    state="readonly"
                )

                self.saved_combo.configure(
                    state="disabled"
                )

                self.save_check.configure(
                    state="normal"
                )

                self.save_name_entry.configure(
                    state=(
                        "normal"
                        if self.save_after_accept.get()
                        else "disabled"
                    )
                )

            else:

                self.mission_combo.configure(
                    state="disabled"
                )

                self.saved_combo.configure(
                    state="readonly"
                )

                self.save_check.configure(
                    state="disabled"
                )

                self.save_name_entry.configure(
                    state="disabled"
                )

        # Followers.
        self.spacing_entry.configure(
            state=(
                "normal"
                if self.robot_count.get() > 1
                else "disabled"
            )
        )

        # Sydney live map is independent of RViz.
        self.map_check.configure(
            state="normal"
        )

        # Logging.
        self.results_entry.configure(
            state=(
                "normal"
                if self.logging_enabled.get()
                else "disabled"
            )
        )

        # Tuning options.
        if tuning:

            targets = [
                "R1",
            ]

            if self.robot_count.get() >= 2:
                targets.append(
                    "R2 <- R1"
                )

            if self.robot_count.get() >= 3:
                targets.append(
                    "R3 <- R2"
                )

            self.tuning_combo.configure(
                state="readonly",
                values=targets,
            )

            current = (
                self.tuning_target.get()
            )

            valid = {
                "R1",
                "R2 <- R1",
                "R3 <- R2",
            }

            if current not in valid:
                self.tuning_target.set(
                    "R1"
                )

            if (
                self.tuning_target.get()
                not in targets
            ):
                self.tuning_target.set(
                    "R1"
                )

            self.trials_entry.configure(
                state="normal"
            )

            self.timeout_entry.configure(
                state="normal"
            )

            self.tuning_note.configure(
                state="normal"
            )

        else:

            self.tuning_combo.configure(
                state="disabled"
            )

            self.trials_entry.configure(
                state="disabled"
            )

            self.timeout_entry.configure(
                state="disabled"
            )

            self.tuning_note.configure(
                state="disabled"
            )

    # =========================================================
    # Submit
    # =========================================================

    @staticmethod
    def _positive_float(
        text,
        field,
    ):

        try:
            value = float(
                text
            )

        except ValueError as exc:
            raise ValueError(
                f"{field} must be a number."
            ) from exc

        if value <= 0.0:
            raise ValueError(
                f"{field} must be > 0."
            )

        return value

    @staticmethod
    def _optional_positive_float(
        text,
        field,
    ):

        text = text.strip()

        if not text:
            return None

        return StartupGUI._positive_float(
            text,
            field,
        )

    @staticmethod
    def _optional_positive_int(
        text,
        field,
    ):

        text = text.strip()

        if not text:
            return None

        try:
            value = int(
                text
            )

        except ValueError as exc:
            raise ValueError(
                f"{field} must be an integer."
            ) from exc

        if value <= 0:
            raise ValueError(
                f"{field} must be > 0."
            )

        return value

    def _submit(self):

        try:

            config = StartupConfig()

            config.robot_count = (
                int(
                    self.robot_count.get()
                )
            )

            config.simulation_profile = (
                "light"
            )

            config.run_mode = (
                self.run_mode.get()
            )

            # ---------------------------------------------
            # Mission
            # ---------------------------------------------

            if config.run_mode == "tuning":

                config.mission_source = (
                    "saved"
                )

                label = (
                    self.saved_mission.get()
                )

                if (
                    label
                    not in self.saved_items
                ):

                    raise ValueError(
                        "Select a valid saved mission "
                        "for tuning."
                    )

                item = (
                    self.saved_items[
                        label
                    ]
                )

                definition = (
                    item[
                        "definition"
                    ]
                )

                config.saved_mission_name = (
                    definition.name
                )

                config.saved_mission_file = str(
                    item[
                        "path"
                    ]
                )

                config.mission_type = (
                    definition.mission_type
                )

                config.mission_parameters = (
                    definition.to_dict()
                )

                config.save_after_accept = (
                    False
                )

            else:

                config.mission_source = (
                    self.mission_source.get()
                )

                if (
                    config.mission_source
                    == "new"
                ):

                    config.mission_type = (
                        self.mission_type.get()
                    )

                    config.save_after_accept = bool(
                        self.save_after_accept.get()
                    )

                    if config.save_after_accept:

                        config.save_mission_name = (
                            self.save_name.get().strip()
                        )

                else:

                    label = (
                        self.saved_mission.get()
                    )

                    if (
                        label
                        not in self.saved_items
                    ):

                        raise ValueError(
                            "Select a valid saved mission."
                        )

                    item = (
                        self.saved_items[
                            label
                        ]
                    )

                    definition = (
                        item[
                            "definition"
                        ]
                    )

                    config.saved_mission_name = (
                        definition.name
                    )

                    config.saved_mission_file = str(
                        item[
                            "path"
                        ]
                    )

                    config.mission_type = (
                        definition.mission_type
                    )

                    config.mission_parameters = (
                        definition.to_dict()
                    )

            # ---------------------------------------------
            # Speed / formation
            # ---------------------------------------------

            config.max_speed = (
                self._positive_float(
                    self.max_speed.get(),
                    "Maximum speed",
                )
            )

            if config.robot_count > 1:

                config.follower_spacing = (
                    self._positive_float(
                        self.follower_spacing.get(),
                        "Follower spacing",
                    )
                )

            else:

                config.follower_spacing = None

            # ---------------------------------------------
            # Display
            # ---------------------------------------------

            config.headless = bool(
                self.headless.get()
            )

            config.show_rviz = bool(
                self.show_rviz.get()
            )

            config.show_map = bool(
                self.show_map.get()
            )

            # ---------------------------------------------
            # Logging
            # ---------------------------------------------

            config.logging_enabled = bool(
                self.logging_enabled.get()
            )

            if config.logging_enabled:

                results = (
                    self.results_dir.get().strip()
                )

                if not results:

                    raise ValueError(
                        "Results directory is required "
                        "when logging is enabled."
                    )

                config.results_dir = (
                    results
                )

            # ---------------------------------------------
            # Tuning
            # ---------------------------------------------

            if config.run_mode == "tuning":

                target_map = {
                    "R1":
                        "r1",

                    "R2 <- R1":
                        "r2",

                    "R3 <- R2":
                        "r3",
                }

                config.tuning_target = (
                    target_map[
                        self.tuning_target.get()
                    ]
                )

                config.tuning_trials = (
                    self._optional_positive_int(
                        self.tuning_trials.get(),
                        "Tuning trials",
                    )
                )

                config.tuning_time_limit_sec = (
                    self._optional_positive_float(
                        self.tuning_timeout.get(),
                        "Per-trial timeout",
                    )
                )

            # ---------------------------------------------
            # Run name
            # ---------------------------------------------

            config.run_name = (
                self.run_name.get().strip()
            )

            config.finalize()

            self.result = config

            self.root.destroy()

        except Exception as exc:

            messagebox.showerror(
                "Invalid configuration",
                str(exc),
            )

    def _cancel(self):

        self.result = None
        self.root.destroy()

    def run(self):

        self.root.mainloop()
        return self.result


def collect_configuration_gui():

    return StartupGUI().run()
