import tkinter as tk
from tkinter import simpledialog
from .DataJson import TestConfig
from GUI_classes.Auxiliary import Auxiliary

class TopRowPanel:
    def __init__(self, app):
        self.app = app
        self.info_label = None

    def build(self, parent):
        # Info label
        self.info_label = tk.Label(parent, text="")
        self.info_label.pack(side="left")

        # Action buttons inline with label
        self.app.stop_event.clear()

        edit_move_by = tk.Button(
            parent,
            text="Set Move Size",
            command=self.set_moveBy
        )

        up = tk.Button(
            parent,
            text="Move Up",
            command=lambda: [
                Auxiliary.safe_call(
                    self.app.stop_event,
                    self.app.communication.msg_queue,
                    self.app.communication.cmd_queue,
                    config=TestConfig(
                        printer_port=self.app.printer_port,
                        force_gauge_port=self.app.force_gauge_port,
                        first_move_z_up_by=self.app.moveBy,
                        exit_after_first_z_move=True,
                    ),
                ),
                print(f"moved by {self.app.moveBy} mm"),
            ],
        )

        down = tk.Button(
            parent,
            text="Move Down",
            command=lambda: [
                Auxiliary.safe_call(
                    self.app.stop_event,
                    self.app.communication.msg_queue,
                    self.app.communication.cmd_queue,
                    config=TestConfig(
                        printer_port=self.app.printer_port,
                        force_gauge_port=self.app.force_gauge_port,
                        first_move_z_down_by=self.app.moveBy,
                        exit_after_first_z_move=True,
                    ),
                ),
                print(f"moved by {self.app.moveBy} mm"),
            ],
        )

        emergency_stop = tk.Button(
            parent,
            text="Emergency Stop",
            bg="red",
            command=self.emergency_stop,
        )

        reset_tests = tk.Button(
            parent,
            text="Reset Tests",
            command=lambda: self.app.plot_panel.reset(),
        )

        save_plot = tk.Button(
            parent,
            text="Save Plot",
            command=lambda: self.app.plot_panel.save_dialog(),
        )

        change_ports = tk.Button(
            parent,
            text="Change Ports",
            command=lambda: [self.setup_ports(), self.show_ports()],
        )

        # Pack buttons
        edit_move_by.pack(side="left", padx=10)
        up.pack(side="left", padx=10)
        down.pack(side="left", padx=10)
        emergency_stop.pack(side="left", padx=10)
        reset_tests.pack(side="left", padx=10)
        save_plot.pack(side="left", padx=10)
        change_ports.pack(side="left", padx=10)

        return parent

    def show_ports(self):
        text = (
            "Move Size: {}    Printer Port: {}    Force Gauge Port: {}    Running Test: {}".format(
                self.app.moveBy,
                self.app.printer_port,
                self.app.force_gauge_port,
                self.app.active_config.stem if self.app.active_config else "None",
            )
        )
        self.app.communication.active_config = self.app.active_config
        
        if self.info_label and self.info_label.winfo_exists():
            self.info_label.config(text=text)
        else:
            # In case build() hasn't run yet
            self.info_label = tk.Label(self.app.top_row, text=text)
            self.info_label.pack(side="left")

    def setup_ports(self):
        printer = simpledialog.askstring("Printer Port", "Printer Port:")
        force_meter = simpledialog.askstring("Force Gauge Port", "Force Gauge Port:")
        self.app.printer_port = printer #or "COM10"
        self.app.force_gauge_port = force_meter #or "COM7"
        self.show_ports()

    def set_moveBy(self):
        value = simpledialog.askfloat("Set Move By", "Enter the Z-axis move distance:")
        if value is not None:
            self.app.moveBy = value
        self.show_ports()

    def emergency_stop(self):
        if self.app.testing_process and self.app.testing_process.is_alive():
            self.app.testing_process.kill()
            self.app.testing_process.join()
            print("Emergency stop issued, terminating process.")
        else:
            print("No active testing process to stop.")