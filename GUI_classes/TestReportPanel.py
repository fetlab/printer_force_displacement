import tkinter as tk
from tkinter import ttk
import json
from GUI_classes.Auxiliary import Auxiliary
class TestReportPanel:
    def __init__(self, app):
        self.app = app
        self.names = []
        self.lb = None
    
    def build(self, panel):

        # top-Right placement 
        panel.pack(side="right", fill="both", expand=True, pady=(8, 0))

        ttk.Label(panel, text="Test Reports:").pack(anchor="w")

        list_frame = ttk.Frame(panel)
        list_frame.pack(fill="both", expand=True)

        self.lb = tk.Listbox(list_frame, height=8, exportselection=False)
        self.lb.pack(side="right", fill="both", expand=True)

        sb = ttk.Scrollbar(list_frame, orient="vertical", command=self.lb.yview)
        sb.pack(side="right", fill="y")
        self.lb.config(yscrollcommand=sb.set)

        btn_row = ttk.Frame(panel)
        btn_row.pack(fill="x", pady=(4, 0))

        tk.Button(btn_row, text="Delete", command=self.delete_test).pack(side="left", padx=4, pady=4)
        tk.Button(btn_row, text="Refresh", command=self.refresh).pack(side="left", padx=4, pady=4)
        tk.Button(btn_row, text="Invalidate", command=self.invalidate_test).pack(side="left", padx=4, pady=4)

        self.refresh()
        return panel
    
    def gather_names(self):
        names = []
        try:
            for p in sorted(self.app.path_to_TestResults.glob("*.json")):
                names.append(p.name)
        except Exception:
            pass
        
        for name in names:
            path_to_file = self.app.path_to_TestResults / name
            with open(path_to_file, "r") as f:
                data = json.load(f)
            if data.get("test_params", {}).get("testInvialided", False):
                names[names.index(name)] = f"{name} (INVALID)"
        return names

    def refresh(self):
        if not self.lb:
            return
        self.names = self.gather_names()
        self.lb.delete(0, tk.END)
        for name in self.names:
            self.lb.insert(tk.END, name)

    def invalidate_test(self):
        if not self.lb:
            return
        selection = self.lb.curselection()
        if not selection:
            return
        
        path_to_file = self.app.path_to_TestResults / self.names[selection[0]]
        with open(path_to_file, "r") as f:
            data = json.load(f)
        
        data["test_params"]["testInvialided"] = True
        
        with open(path_to_file, "w") as f:
            json.dump(data, f, indent=2, default=Auxiliary.json_encode)

        # change the name on the selected item to show it's invalidated, 
        # but does not change the file name of the report
        self.names[selection[0]] = f"{self.names[selection[0]]} (INVALID)"


    def delete_test(self):
        if not self.lb:
            return
        selection = self.lb.curselection()
        if not selection:
            return
        selected_name = self.names[selection[0]]
        if "(INVALID)" in selected_name:
            selected_name = selected_name.replace(" (INVALID)", "")
        
        file_path = self.app.path_to_TestResults / selected_name

        try:
            # Delete the file
            file_path.unlink()

        except Exception as e:
            print(f"Error deleting test report: {e}")
    
