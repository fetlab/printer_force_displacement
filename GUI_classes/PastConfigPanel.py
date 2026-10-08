from tkinter import ttk
import tkinter as tk
from pathlib import Path

class PastConfigPanel:
    def __init__(self, app):
        self.app = app
        self.panel = None
        self.parent = None
        self.lb = None
        self.names = []
        self.past_configs = []

    def build(self, parent):
        self.parent = parent
        if self.panel and self.panel.winfo_exists():
            self.panel.destroy()

        panel = ttk.Frame(parent, padding=(6, 6))
        self.panel = panel
        # Bottom-left placement within the parameter panel
        panel.pack(side="bottom", fill="x", pady=(8, 0))

        ttk.Label(panel, text="Past Configuration Files:").pack(anchor="w")

        list_frame = ttk.Frame(panel)
        list_frame.pack(fill="x")

        self.lb = tk.Listbox(list_frame, height=8, exportselection=False)
        self.lb.pack(side="left", fill="x", expand=True)

        sb = ttk.Scrollbar(list_frame, orient="vertical", command=self.lb.yview)
        sb.pack(side="right", fill="y")
        self.lb.config(yscrollcommand=sb.set)

        self.lb.bind('<Double-Button-1>', self.on_double_click)

        btn_row = ttk.Frame(panel)
        btn_row.pack(fill="x", pady=(4, 0))

        tk.Button(btn_row, text="Delete", command=self.remove_item).pack(side="left", padx=4)
        tk.Button(btn_row, text="Refresh", command=self.refresh).pack(side="left", padx=4)
        tk.Button(btn_row, text="Delete All", command=self.remove_all_items).pack(side="left", padx=4)

        self.refresh()
        return panel

    def gather_names(self):
        names = []
        try:
            for p in sorted(self.app.path_configs.glob("*.json")):
                names.append(p.name)
        except Exception:
            pass

        for k in self.past_configs:
            name = Path(k).name if isinstance(k, (str, Path)) else str(k)
            if name not in names:
                names.append(name)
        return names

    def refresh(self):
        if not self.lb:
            return
        self.names = self.gather_names()
        self.lb.delete(0, tk.END)
        for name in self.names:
            self.lb.insert(tk.END, name)

    def on_double_click(self, event):
        selection = self.lb.curselection()
        if not selection:
            return
        name = self.lb.get(selection[0])
        self.app.active_config = self.app.path_configs / name
        self.app.customTestBuilder.overwrite_config(self.app.active_config)
        self.app.show_ports()

    def remove_item(self):
        if not self.lb:
            return
        selection = self.lb.curselection()
        if not selection:
            return
        name = self.lb.get(selection[0])
        path_to_remove = self.app.path_configs / name
        if self.app.active_config and path_to_remove == self.app.active_config:
            self.app.active_config = None
            self.app.show_ports()
        try:
            path_to_remove.unlink()
        except Exception as e:
            print(f"Error deleting file {path_to_remove}: {e}")
        try:
            self.names.remove(name)
        except ValueError:
            pass
        self.refresh()

    def remove_all_items(self):
        for name in list(self.names):
            path_to_remove = self.app.path_configs / name
            try:
                path_to_remove.unlink()
            except Exception as e:
                print(f"Error deleting file {path_to_remove}: {e}")
        self.names.clear()
        self.app.active_config = None
        self.app.show_ports()
        self.refresh()