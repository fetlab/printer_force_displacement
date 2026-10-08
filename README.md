# ForceMeter GUI
The dir `ender_fdm`, `force_gauge.py`, `results2csv.py`, `combined_tests.py` and `defined_models.csv` is forked from https://github.com/fetlab/printer_force_displacement!

## How to run
To run the GUI, run: 
  - Windows: `python GUI_main.py`
  - Linux/Mac: `python3 GUI_main.py` 

## Tips while running
- Do keep a the console open while running, it is nice to have while preforming tests.  

## 3D models 
- 2040 extrusion: Desgined by nanuk : http://printables.com/model/836963-vslot-2040/comments. My modified version can be found in `3D_models/2040Vslot15cm.stl`
- 2040 extender piece. Desgined by elimile: https://www.printables.com/model/1412698-t-nut-insert-for-tv-slot-2020-extrusion. My modified version can be found in `3D_models/extender_vslot_size_x5.stl`
- Bed clamp: Desgin by Daniel Ashbrook. My modified version can be found in `3D_models/frame_clamp_v3-Body.mf3`

## Architecture
### Overview: 
Tkinter-based desktop GUI that orchestrates mechanical tests in a background process, streams readings back via queues, and plots Force–Displacement in real time with Matplotlib. All code related to executing the force-displacement tests are keept in `ender_fdm` and its main file `force_test.py`. Inside the dirs `configs`, `test_reports`, `custom_tests` and `plots` are saved configuration files, JSON test reports, custom test recipes and saved plots respectively.

### Classes
Each class from the dir `GUI_classes` are initilized inside the GUI_main.py. The GUI_main.py holds most of the state information.

#### Auxiliary.py
Holds AUX function that does not take self, most importanly `safe_call()` that calls the main function of the force testing logic within force_test.py.

#### Communication.py
Keeps track of the two one directinal Queues used for commnication between the GUI and back-end testing logic. 
*msg_queue*: the GUI reads from this 
*cmd_queue*: the backend read from this, currently only inside the custom test function in ender_fdm/force_gauge.py.

### PlotPanel.py
Embeds a Matplotlib figure and periodically updates the Force–Displacement plot using data buffered in `Communication.tests`. Supports reset and saving plots.

### TopRowPanel.py
Controls ports and movement: display/setup ports, set move size, manual Z up/down, emergency stop (terminates the process), reset plot, save plot, change ports.

### PastConfigPanel.py
Lists previously saved configuration files; provides refresh, delete (single/all), and double-click to load a config and set it as `active_config`.

### CustomTestBuilder.py
Sends stepwise custom commands to the running test via `cmd_queue`; supports undo last move,     finish/save a `custom_test_recipe`, and upload/playback recipes for a selected config. Weeh uploading/rerunning a custom test the config used for the test, needs to be present in the dir `GUI_classes/configs`.

### TestReportPanel.py
Lists JSON test reports, supports invalidate (marks a JSON report unuseable) and delete; 

### DataJson.py
Defines the data models:
- **TestConfig**: Dataclass of test parameters (serial ports, motion settings, test options);     JSON-serializable.
- **custom_test_recipe**: Dataclass representing a named sequence of custom moves; JSON-          serializable.

### custom_msg.py
Defines message types used in custom testing:
- **custom_test_msg**: A single move instruction with `direction` and `z_inc`.
- **panic_msg**: An undo signal carrying the step to revert.

## Testing logic
Inside the `force_gauge.py` is the config from the GUI read and the test function according to the `Test_type` is executed and all parameters need for the test can be found in the config file.
Inside the `ender_fdm/force_gauge.py` is the main logic of each test type implemented. 

# Parameters of the config file
### Serial Connection Parameters
| Parameter | Description |
|----------|------------|
| `force_gauge_port` | Serial port used to communicate with the force gauge. |
| `force_gauge_baud` | Baud rate of the force gauge serial connection. |
| `printer_port` | Serial port used to communicate with the 3D printer. |
| `printer_baud` | Baud rate of the printer serial connection. |
---

### Motion Options
| Parameter | Description |
|----------|------------|
| `feedrate` | Default feedrate for all printer movements (mm/min). |
---

### Startup Move Options
| Parameter | Description |
|----------|------------|
| `first_move_z_up_by` | Move the Z-axis up by this distance (mm) before starting. |
| `first_move_z_down_by` | Move the Z-axis down by this distance (mm) before starting. |
| `exit_after_first_z_move` | Exit the program after performing the initial Z movement. |
| `do_zero` | Automatically zero the printer by moving down until force is detected. |
| `zero_coarse_inc` | Coarse movement step used during zeroing (mm). |
| `zero_fine_inc` | Fine movement step used during zeroing (mm). |
---

### General Test Options
| Parameter | Description |
|----------|------------|
| `test_type` | Type of test to perform: `careful`, `force-limit_test`, `displacement-limit_test`, `custom`  |
| `test_direction` | Direction in which the movement test is performed. |
| `test_loops` | Number of repeated loops; set to `0` to perform a single movement only. |
| `test_num` | Starting index for test numbering. |
| `return_to_zero_after_test` | Return to the zero position after a single test (`test_loops == 0`). |
| `outfile` | Output CSV file for recorded test data. |
---

### Careful Test Options
| Parameter | Description |
|----------|------------|
| `n_samples` | Number of samples averaged per measurement increment. |
| `careful_inc` | Step size per measurement increment (mm). |
| `stop_after` | Maximum travel distance before stopping if no snap-through occurs (mm). |
| `min_down` | Minimum downward travel before snap-through detection begins (mm). |
| `max_down` | Fixed downward travel distance when automation is disabled (mm). Requires `max_up`. |
| `upg_max` | Fixed upward travel distance when automation is disabled (mm). Requires `max_down`. |
---

### Limit Test Options
| Parameter | Description |
|----------|------------|
| `force_threshold` | Stop the test if this force is exceeded. |
| `displacement_threshold` | Stop the test when this displacement is reached. |
| `hold_time` | Time (seconds) to hold the final test position. |
---

### Utility Options
| Parameter | Description |
|----------|------------|
| `force_info` | If `> 0`, print force information the specified number of times and exit. If `< 0`, print force information continuously until interrupted. |
| `quicktest` | Perform the initial Z movement and print force data. |
| `debug_gcode` | Print every G-code command as it is issued. |
---


