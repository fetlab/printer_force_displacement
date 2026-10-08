from pathlib import Path
from tkinter import simpledialog
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class PlotPanel:
    def __init__(self, app):
        self.app = app
        self.fig = None
        self.ax = None
        self.canvas_fig = None
        self._running = False

    def build(self, parent):
        self.fig = Figure(figsize=(10, 9), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.canvas_fig = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas_fig.get_tk_widget().pack(side="left", fill="both", expand=True)
        self.canvas_fig.draw()
        return self.canvas_fig

    def start(self):
        self._running = True
        self._schedule_update()

    def stop(self):
        self._running = False

    def _schedule_update(self):
        if not self._running:
            return
        self.update()
        # Schedule next update in 40 ms
        self.app.master.after(40, self._schedule_update)

    def update(self):
        if not (self.ax and self.canvas_fig):
            return
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_title("Force vs Displacement")
        self.ax.set_xlabel("Displacement")
        self.ax.set_ylabel("Force")

        for testno, data in self.app.communication.tests.items():
            self.ax.plot(data["x"], data["y"], label=f"Test {data['config_name']} #{testno}", marker='o')

        if self.app.communication.tests:
            self.ax.legend()
        self.canvas_fig.draw()

    def reset(self):
        # Clear tests and plot
        self.app.communication.tests = {}
        self.app.communication.number_of_tests = 0
        if self.ax and self.canvas_fig:
            self.ax.clear()
            self.canvas_fig.draw()

    def save(self, location: Path):
        if self.fig:
            self.fig.savefig(location)

    def save_dialog(self):
        file_name = simpledialog.askstring("Save Plot", "Enter file name to save plot (e.g., plot.png):")
        if file_name:
            location = self.app.path_plots / file_name
            self.save(location)
    
    def plot_old_testResult(self):
        pass