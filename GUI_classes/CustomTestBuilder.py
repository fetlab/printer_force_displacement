from pathlib import Path
import json
from tkinter import simpledialog
from tkinter import messagebox
from dataclasses import asdict
from .DataJson import custom_test_recipe
from GUI_classes.custom_msg import custom_test_msg, panic_msg
from GUI_classes.Auxiliary import Auxiliary

class CustomTestBuilder:
    def __init__(self, app):
        # Hold a live reference to the App instance 
        self.app = app

    def send_custom_test_command(self, direction, increment):  
        if increment < 0.1:
            messagebox.showerror(
                title="Invalid Move Size",
                message="Size of move must be at least 0.1 mm.",
            )
            return

        # only send if test is running
        proc = self.app.testing_process
        if proc and proc.is_alive():
            msg = custom_test_msg(
                direction=direction,
                z_inc=increment
            )
            # send command to testing process
            self.app.communication.send_cmd_msg(msg)
            # add to current custom test moves
            self.app.current_custom_test.append(msg)
    
    def undo_move_custom_test(self):
        if not self.app.communication.tests:
            return
        k, v = next(reversed(self.app.communication.tests.items()))
        
        if self.app.current_custom_test:
            # get last move and create undo message
            last_msg = self.app.current_custom_test.pop()
            undo_direction = last_msg.direction.flip()
            undo_msg = custom_test_msg(
                direction=undo_direction,
                z_inc=last_msg.z_inc
            )
            # remove last data point from plot
            self.app.communication.tests[k]["x"].pop()
            self.app.communication.tests[k]["y"].pop()
            # send undo message to force gauge
            undo_msg = panic_msg(
                reason="UNDO",
                undo_steps=undo_msg
            )
            # send undo command
            self.app.communication.send_cmd_msg(undo_msg)
    
    def finshed_custom_test(self):
        # send done message to force gauge
        done_msg = "TEST_DONE"
        self.app.communication.send_cmd_msg(done_msg)
        
        # converting all moves from current_custom_test to json and save it
        json_recipe = custom_test_recipe(
            name=self.app.vars["test_id"].get(),
            test_config=self.app.active_config.stem if self.app.active_config is not None else None,
            moves=self.app.current_custom_test
        )
        recipe_path = self.app.path_custom_tests / f"{json_recipe.name}.json"
        with open(recipe_path,"w") as f:
            json.dump(asdict(json_recipe), f, indent=2, default=Auxiliary.json_encode)
        # Clear current custom test moves after saving
        self.app.current_custom_test.clear()
   
    def upload_custom_test(self):
        recipe_name = simpledialog.askstring("Load Custom Test", "Enter the custom test recipe name:")
        recipe_path = self.app.path_custom_tests / f"{recipe_name}.json"
        if recipe_path.exists():
            
            with open(recipe_path, "r") as f:
                data = json.load(f)
            recipe = custom_test_recipe(**data)
            config_path = self.app.path_configs / f"{recipe.test_config}.json"
            if config_path.exists():
                self.app.active_config = config_path
                self.overwrite_config(self.app.path_configs / f"{recipe.test_config}.json")
                self.app.show_ports()

                check = messagebox.askyesno("Start Test", f"Configuration '{recipe.test_config}' loaded. Start test now?")
                if not check:
                    return

                # start test with loaded recipe
                self.app.testing(self.app.active_config)
                
                for move in recipe.moves:
                    self.send_custom_test_command(move["direction"], move["z_inc"])

                done_msg = "TEST_DONE"
                self.app.communication.send_cmd_msg(done_msg)
            else:
                messagebox.showerror("Error", f"Test config '{recipe.test_config}' not found.")
        else:
            messagebox.showerror("Error", f"Custom test recipe '{recipe_name}' not found.")
    
    def overwrite_config(self, config_path:Path):
        new_config = Auxiliary.load_config(config_path)
        for var_name, value in new_config.__dict__.items():
            if var_name in self.app.vars:

                if var_name == "outfile":
                    value = Path(value).name 
                else:
                    self.app.vars[var_name].set(value)
        
        self.app.vars["test_id"].set(config_path.stem)