from pathlib import Path
import multiprocessing
import tkinter as tk
from tkinter import messagebox

# Import GUI classes
from GUI_classes.Communication import Communication
from GUI_classes.ParameterPanel import ParameterPanel
from GUI_classes.PlotPanel import PlotPanel
from GUI_classes.TopRowPanel import TopRowPanel
from GUI_classes.PastConfigPanel import PastConfigPanel
from GUI_classes.CustomTestBuilder import CustomTestBuilder
from GUI_classes.Auxiliary import Auxiliary
from GUI_classes.TestReportPanel import TestReportPanel


class App(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        # State 
        self.printer_port = None
        self.force_gauge_port = None 
        self.moveBy = 1.0

        self.stop_event = multiprocessing.Event()
        self.testing_process = None
        self.active_config = None 
        self.current_custom_test = []
       
        # paths for configs, custom tests, plots, and test results
        self.path_configs = Path("GUI_classes", "configs")
        self.path_custom_tests = Path("GUI_classes", "custom_tests")
        self.path_plots = Path("GUI_classes", "plots")
        self.path_to_TestResults = Path("GUI_classes", "test_reports")
        
        # Application 
        self.vars = {
            "test_id": tk.StringVar(value="test1"),
            "test_type": tk.StringVar(value="careful"),
            "feedrate": tk.IntVar(value=300),
            "test_direction": tk.StringVar(value="DOWN"),
            "careful_inc": tk.DoubleVar(value=0.5),
            "stop_after": tk.IntVar(value=16),
            "min_down": tk.DoubleVar(value=2.0),
            "n_samples": tk.IntVar(value=1),
            "outfile": tk.StringVar(value="outputFile"),
            "force_threshold": tk.DoubleVar(value=1.0),
            "displacement_threshold": tk.DoubleVar(value=1.0),
            "hold_time": tk.DoubleVar(value=1.0),
            "do_zero": tk.BooleanVar(value=True),
            "do_preMove": tk.BooleanVar(value=True),
            "return_to_zero_after_test": tk.BooleanVar(value=True),
        }
        self.master = master
        self.pack(fill="both", expand=True)
        self.canvasSize = 1200
    
        # right panel for test reports 
        self.test_report_panel = tk.Frame(self)
        self.test_report_panel.pack(side="right", fill="y", padx=10, pady=10)
        # Test report panel 
        self.test_report_panel_obj = TestReportPanel(self)
        self.test_report_panel_obj.build(self.test_report_panel)

        # Top row frame to hold ports label and action buttons inline
        self.top_row = tk.Frame(self)
        self.top_row.pack(fill="x", padx=10, pady=10)

        # Main canvas below the top row
        self.main_row = tk.Canvas(self, width=self.canvasSize)
        self.main_row.pack(fill="both", expand=True)
        
        # left parameters panel 
        self.parameter_panel = ParameterPanel(self)
        self.param_frame = self.parameter_panel.build(self.main_row)
        # Past configs panel 
        self.past_config_panel = PastConfigPanel(self)
        self.past_config_panel.build(self.param_frame)

        # main canvas 
        self.canvas = tk.Canvas(
            self.main_row, 
            width=self.canvasSize, 
            height=self.canvasSize, 
            bg="white")
        self.canvas.pack(side="left", fill="both", expand=True)

        # plotting
        self.plot_panel = PlotPanel(self)
        self.plot_panel.build(self.canvas)

        # Setup 
        self.communication = Communication(path_to_TestResults=self.path_to_TestResults)
        self.schedule_check_messages()

        self.customTestBuilder = CustomTestBuilder(app=self)

        # Top bar (ports, movement, emergency stop, etc.)
        self.top_bar = TopRowPanel(self)
        self.top_bar.build(self.top_row)
        self.top_bar.show_ports()
        self.top_bar.setup_ports()
        self.plot_panel.start()

    def testing(self, config_path):
        """Start a new testing process with the given configuration."""
        self.stop_event.clear()
        
        if self.testing_process and self.testing_process.is_alive():
            messagebox.showwarning("Warning", "A test is already running.")
            return
        
        if self.active_config is None:
            messagebox.showwarning("Warning", "No active configuration selected.")
            return
        
        config = Auxiliary.load_config(config_path)
        if config.printer_port == "" or config.force_gauge_port == "":
            messagebox.showerror("Error", "Please set both printer and force gauge ports before starting a test.")
            return
        
        process = multiprocessing.Process (
            target=Auxiliary.safe_call,
            args=(self.stop_event, self.communication.msg_queue, self.communication.cmd_queue, config),     
            daemon=True   # optional
        )
        process.start()
        self.testing_process = process
    
    def schedule_check_messages(self):
        """Periodically poll communication messages at 100 ms intervals."""
        self.communication.check_messages()
        self.master.after(100, self.schedule_check_messages)

     # top row delegates for compatibility with existing call sites
    def show_ports(self):
        return self.top_bar.show_ports()

    def setup_ports(self):
        return self.top_bar.setup_ports()

    def emergency_stop(self):
        return self.top_bar.emergency_stop()

    def set_moveBy(self):
        return self.top_bar.set_moveBy()

    def build_parameter_panel(self, parent):
        # Backward-compatible wrapper in case other callers still use this
        if not hasattr(self, 'parameter_panel'):
            self.parameter_panel = ParameterPanel(self)
        return self.parameter_panel.build(parent)   

def main_gui():
    app = App(master=tk.Tk())
    app.master.title("ForceMeterGUI")
    app.mainloop()

if __name__ == "__main__":
    # Needed on Windows when using multiprocessing (spawn start method)
    try:
        from multiprocessing import freeze_support
        freeze_support()
    except Exception:
        pass
    main_gui()