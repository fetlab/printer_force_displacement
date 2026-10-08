from ender_fdm.direction import Direction, UP, DOWN, STILL
from ender_fdm.constants import DEFAULT_FEEDRATE, MAX_FEEDRATE
import queue
from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import json

from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from .custom_msg import custom_test_msg

@dataclass
class TestConfig:
	"""
	Serial connection parameters:
	:param force_gauge_port: Serial port for the force gauge.
	:param force_gauge_baud: Force gauge serial port baud rate.
	:param printer_port: Serial port for the printer.
	:param printer_baud: Printer serial port baud rate.

	Motion options:
	:param feedrate: Set the default feedrate for moves in mm/minute.

	Startup move options:
	:param first_move_z_up_by: Move the Z-axis up by N mm before doing anything else.
	:param first_move_z_down_by: Move the Z-axis down by N mm before doing anything else.
	:param exit_after_first_z_move: Exit after first_move_z_{up,down}_by.
	:param do_zero: Zero the printer by moving down until force is nonzero.
	:param zero_coarse_inc: Large move amount for zeroing (mm).
	:param zero_fine_inc: Small move amount for zeroing (mm).

	General test options:
	:param test_type: Specify 'careful' or 'smooth'
	:param test_direction: Do a movement test in this direction.
	:param test_loops: Do repeated loops; if 0, only move a single direction then stop.
	:param test_num: Start numbering tests here.
	:param return_to_zero_after_test: Return to the zeroed point if test_loops == 0.
	:param outfile: Write a CSV file here.

	Careful test options:
	:param n_samples: Average this many samples per increment.
	:param careful_inc: Step this many mm per measurement.
	:param stop_after: Stop moving after this many mm if no snap-through has happened.
	:param min_down: Move at least this much before trying to auto-detect
		snap-through.
	:param max_down: Don't automate movment, just move down this much. Also
		specify max_up.
	:param upg_max: Don't automate movment, just move up this much. Also
		specify max_down.
	
	Limit test options:
	:param force_threshold: Stop the test if this force is exceeded.
	:param displacement_threshold: Stop the test when displacement reaches this value.
	:param hold_time: Hold this many seconds at the final posistion of the test.

	:param force_info:  If > 0, print force information that many times and exit.
		If < 0, print force information until ctrl-C.
	:param quicktest: Move in first_move_z* and print force data
	:param debug_gcode: Print every Gcode command as it is issued.
	"""

	# Required
	force_gauge_port: str = None
	printer_port: str = None


	# Communication
	force_gauge_baud: int = 2400
	printer_baud: int = 115200
	force_gauge_timeout: float = 1
	printer_timeout: Optional[float] = None

	# Motion
	feedrate: float = DEFAULT_FEEDRATE

	first_move_z_up_by: float = 0.0
	first_move_z_down_by: float = 0.0
	exit_after_first_z_move: bool = False

	do_zero: bool = True
	do_preMove : bool = True
	zero_coarse_inc: float = 0.5
	zero_fine_inc: float = 0.1

	# Test
	test_type: str = ""
	test_direction: Direction = STILL
	test_loops: int = 0
	test_num: int = 1
	n_samples: int = 1

	min_displacement: float = 0.0

	careful_inc: float = 0.25
	stop_after: float = 15
	min_down: float = 0
	max_down: float = 0
	max_up: float = 0

	return_to_zero_after_test: bool = True
	smooth_displacement: float = 0.0

	outfile: Path = Path("")

	# Flags
	force_info: int = 0
	quicktest: bool = False
	debug_gcode: bool = False

	# Limite test options
	force_threshold: float = 0.0
	hold_time: float = 1.0
	displacement_threshold: float = 0.0

	# custom test option
	max_force: float = 3.0 # find the real limit
	max_displacement: float = None 
	recording: bool = False

	def to_json_encodable(self):
		return self.__dict__

@dataclass
class custom_test_recipe:
	name: str 
	test_config: str
	moves: list["custom_test_msg"]

	def to_json_encodable(self):
		return self.__dict__
