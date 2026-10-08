from tkinter import ttk
import tkinter as tk
from pathlib import Path
import json
from dataclasses import asdict
from GUI_classes.DataJson import TestConfig
from tkinter import simpledialog
from ender_fdm.direction import Direction, UP, DOWN

class ParameterPanel:
    def __init__(self, app):
        self.app = app
        self.custom_btns_container = None
        self.hold_row = None
        self.force_row = None
        self.displacement_row = None
        self.stop_after_row = None
        self.min_down_row = None

    def row(self, frame, label):
        r = ttk.Frame(frame)
        r.pack(fill="x", pady=2)
        ttk.Label(r, text=label, width=30).pack(side="left")
        return r

    def write_config(self):
        """Generate and save a configuration file based on current parameters."""
        config_path = f"{self.app.vars['test_id'].get()}.json"
        full_config_path = self.app.path_configs / config_path

        check_move = self.app.vars["careful_inc"].get()
        # checks move size is not smaller than 0.1 mm
        if check_move < 0.1:
            simpledialog.messagebox.showerror(
                title="Invalid Move Size",
                message="Size of move must be at least 0.1 mm.",
            )
            return
        check_force = self.app.vars["force_threshold"].get()
        if check_force < 0 or check_force >= 5:
            simpledialog.messagebox.showerror(
                title="Invalid Force Threshold",
                message="Force threshold must be between 0 and 5 kgf.",
            )
            return
        check_samples = self.app.vars["n_samples"].get()
        if check_samples < 1 or check_samples > 10:
            simpledialog.messagebox.showerror(
                title="Invalid Sampling Rate",
                message="Sampling rate must be between 1 and 1000 samples per move.",
            )
            return

        defaults = asdict(TestConfig())
        if isinstance(defaults.get("test_direction"), Direction):
            defaults["test_direction"] = defaults["test_direction"].value
        if isinstance(defaults.get("outfile"), Path):
            defaults["outfile"] = ""
        

        path_to_results = self.app.path_to_TestResults / f"{self.app.vars['outfile'].get()}.json"
        overrides = {
            "force_gauge_port": self.app.force_gauge_port,
            "printer_port": self.app.printer_port,
            "test_type": self.app.vars["test_type"].get(),
            "test_direction": self.app.vars["test_direction"].get(),
            "feedrate": self.app.vars["feedrate"].get(),
            "careful_inc": self.app.vars["careful_inc"].get(),
            "do_zero": self.app.vars["do_zero"].get(),
            "do_preMove": self.app.vars["do_preMove"].get(),
            "return_to_zero_after_test": self.app.vars["return_to_zero_after_test"].get(),
            "stop_after": self.app.vars["stop_after"].get(),
            "min_down": self.app.vars["min_down"].get(),
            "n_samples": self.app.vars["n_samples"].get(),
            "displacement_threshold": self.app.vars["displacement_threshold"].get(),
            "force_threshold": self.app.vars["force_threshold"].get(),
            "hold_time": self.app.vars["hold_time"].get() if self.app.vars["hold_time"].get() else 1.0,
            "outfile": str(path_to_results),
        }

        defaults.update(overrides)
        cfg = defaults

        self.app.active_config = full_config_path
        print(f"full_config_path: {full_config_path}")
        self.app.show_ports()

        with open(full_config_path, "w") as f:
            json.dump(cfg, f, indent=2)

    def update_visibility(self, *_args):
        """Update visibility of parameter rows based on test type."""
        test = self.app.vars["test_type"].get().lower()
        if self.custom_btns_container:
            self.custom_btns_container.pack_forget()
        if self.force_row:
            self.force_row.pack_forget()
        if self.hold_row:
            self.hold_row.pack_forget()
        if self.displacement_row:
            self.displacement_row.pack_forget()
        if self.stop_after_row:
            self.stop_after_row.pack_forget()
        if self.min_down_row:
            self.min_down_row.pack_forget()

        if test == "force-limit_test":
            self.force_row.pack(fill="x", pady=2)
            self.hold_row.pack(fill="x", pady=2)
            self.stop_after_row.pack(fill="x", pady=2)

        elif test == "displacement-limit_test":
            self.displacement_row.pack(fill="x", pady=2)
            self.hold_row.pack(fill="x", pady=2)
        elif test == "custom":
            self.custom_btns_container.pack(fill="x", pady=4)
        elif test == "careful":
            self.stop_after_row.pack(fill="x", pady=2)
            self.min_down_row.pack(fill="x", pady=2)

    def build(self, parent):
        """Build the parameter panel UI."""
        frame = ttk.Frame(parent, padding=10)
        frame.pack(side="left", fill="y")

        tk.Button(
            frame,
            text="Generate Configureation file",
            bg="green",
            command=lambda: [self.write_config(), self.app.past_config_panel.refresh()],
        ).pack(fill="x", pady=(10, 2))

        tk.Button(
            frame,
            text="Start Test",
            bg="red",
            command=lambda: [self.app.current_custom_test.clear(), self.app.testing(self.app.active_config)],
        ).pack(fill="x")

        ttk.Entry(self.row(frame, "Test id"), textvariable=self.app.vars["test_id"], width=20).pack(side="right")

        test_type_row = self.row(frame, "Test type")
        ttk.Combobox(
            test_type_row,
            textvariable=self.app.vars["test_type"],
            values=("careful", "custom", "force-limit_test", "displacement-limit_test"),
            state="readonly",
            width=20,
        ).pack(side="right")

        ttk.Entry(self.row(frame, "Feedrate (Speed of printer)"), textvariable=self.app.vars["feedrate"], width=10).pack(side="right")

        ttk.Combobox(
            self.row(frame, "Test direction"),
            textvariable=self.app.vars["test_direction"],
            values=("UP", "DOWN"),
            state="readonly",
            width=20,
        ).pack(side="right")

        careful_row = self.row(frame, "Size of move(mm)")
        ttk.Entry(careful_row, textvariable=self.app.vars["careful_inc"], width=10).pack(side="right")

        do_zero_row = self.row(frame, "Zero on specimen")
        ttk.Checkbutton(do_zero_row, variable=self.app.vars["do_zero"]).pack(side="right")

        return_to_zero_row = self.row(frame, "Return to zero after test")
        ttk.Checkbutton(return_to_zero_row, variable=self.app.vars["return_to_zero_after_test"]).pack(side="right")

        do_preMove_row = self.row(frame, "Pre load before test")
        ttk.Checkbutton(do_preMove_row, variable=self.app.vars["do_preMove"]).pack(side="right")

        self.stop_after_row = self.row(frame, "Stop after (mm)")
        ttk.Entry(self.stop_after_row, textvariable=self.app.vars["stop_after"], width=10).pack(side="right")

        self.min_down_row = self.row(frame, "Min down (mm)")
        ttk.Entry(self.min_down_row, textvariable=self.app.vars["min_down"], width=10).pack(side="right")

        ttk.Entry(self.row(frame, "Sampling Rate(pr. move)"), textvariable=self.app.vars["n_samples"], width=10).pack(side="right")

        ttk.Entry(self.row(frame, "Result file"), textvariable=self.app.vars["outfile"], width=30).pack(side="right")

        self.hold_row = self.row(frame, "Hold time (s)")
        ttk.Entry(self.hold_row, textvariable=self.app.vars["hold_time"], width=10).pack(side="right")

        self.force_row = self.row(frame, "Force threshold (kgf)")
        ttk.Entry(self.force_row, textvariable=self.app.vars["force_threshold"], width=10).pack(side="right")

        self.displacement_row = self.row(frame, "Displacement threshold (mm)")
        ttk.Entry(self.displacement_row, textvariable=self.app.vars["displacement_threshold"], width=10).pack(side="right")

        self.custom_btns_container = ttk.Frame(frame)
        tk.Button(
            self.custom_btns_container,
            text="UP",
            bg="green",
            command=lambda: self.app.customTestBuilder.send_custom_test_command(UP, self.app.vars["careful_inc"].get()),
        ).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(
            self.custom_btns_container,
            text="DOWN",
            bg="green",
            command=lambda: self.app.customTestBuilder.send_custom_test_command(DOWN, self.app.vars["careful_inc"].get()),
        ).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        tk.Button(
            self.custom_btns_container,
            text="UNDO",
            bg="red",
            command=lambda: self.app.customTestBuilder.undo_move_custom_test(),
        ).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(
            self.custom_btns_container,
            text="DONE",
            bg="red",
            command=lambda: self.app.customTestBuilder.finshed_custom_test(),
        ).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        tk.Button(
            self.custom_btns_container,
            text="Upload Test",
            command=lambda: self.app.customTestBuilder.upload_custom_test(),
        ).grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        self.custom_btns_container.columnconfigure(0, weight=1)
        self.custom_btns_container.columnconfigure(1, weight=1)

        self.update_visibility()
        self.app.vars["test_type"].trace_add("write", self.update_visibility)

        return frame