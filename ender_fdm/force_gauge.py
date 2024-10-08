from pathlib import Path
from serial import Serial
from serial.threaded import ReaderThread
from .threaded_force_meter import ThreadedForceMeter
from .direction import Direction, UP, DOWN, inc2dir, force2dir
from time import time
from typing import Callable
from rich import print
from dataclasses import dataclass
from functools import partial
import sys, csv, json

DEFAULT_FEEDRATE = 180
MAX_FEEDRATE = 300    #From marlin Configuration.h or issue M503
MIN_Z_MOVE = 0.1

def sign(v:int|float|Direction) -> int:
	if isinstance(v, Direction): v = v.sign
	if v == 0: return  0
	if v <  0: return -1
	if v >  0: return  1
	raise ValueError(f"Can't take sign of {v}")

def zero(v):       return v == 0
def nonzero(v):    return v != 0

def zeroeps(v):
	"""Return True if v is 0 or very close to 0, otherwise False."""
	return abs(v) <= .001

def nonzeroeps(v):
	"""Return False if v is 0 or very close to 0, otherwise True."""
	return abs(v) > .001

#Return test functions based on comparison with x
def oppsign(cmp):
	"""Return a function `test(val)` which returns True if `cmp` and `val` have
	opposite signs."""
	return lambda testval: sign(cmp) + sign(testval) == 0


def samesign(cmp):
	return lambda testval: sign(cmp) == sign(testval)


def oppsign_or_zero(cmp):
	"""Return a function, test(val), which returns True if `cmp` and `val` have
	opposite signs or if `val` is 0."""
	return lambda testval: zero(testval) or oppsign(cmp)(testval)


def samesign_or_zero(cmp):
	return lambda testval: zero(testval) or samesign(cmp)(testval)


def oppdir_or_zero(direction, force):
	return oppsign_or_zero(direction)(force)


def samedir_or_zero(direction, force):
	return oppsign_or_zero(direction.flip())(force)


def oppdir(direction, force):
	return oppsign(direction)(force)


def samedir(direction, force):
	return oppsign(direction.flip())(force)


@dataclass
class TestResult:
	timestamp:    float
	direction:    Direction
	force:        float
	test_type:    str
	z:            float = float('inf')
	displacement: float = float('inf')
	testno:       int = -1

	def to_json_encodable(self):
		return self.__dict__


def results_to_json(test_params:dict, test_results: list[TestResult], outfile:Path) -> Path:
	"""Save the results to JSON, renaming the output file if it exists. Return
	the Path that the data was saved to."""
	if outfile.exists():
		stem = outfile.stem
		newoutfile = outfile.with_stem(stem + '-01')
		if len(p := stem.rsplit('-', maxsplit=1)) > 1:
			try:
				newoutfile = outfile.with_stem(f'{p[0]}-{int(p[1])+1:02}')
			except ValueError:
				pass
		return results_to_json(test_params, test_results, newoutfile)

	print(f"Saving to {outfile}")
	with open(outfile, 'w') as f:
		json.dump({'test_params': test_params, 'test_results': test_results}, f,
						indent=2,
						default=lambda o: o.to_json_encodable())

	return outfile


def results_to_csv(test_results: list[TestResult], outfile:Path):
	fieldnames = list(test_results[0].__dict__.keys())
	outfile_exists = outfile.exists()
	if outfile_exists:
		print(f"Warning: appending to file {outfile}")
	with open(outfile, 'a' if outfile_exists else 'w', newline='') as f:
		writer = csv.DictWriter(f, fieldnames=fieldnames)
		if not outfile_exists:
			writer.writeheader()
		writer.writerows(r.__dict__ for r in test_results)


class FDMeter:
	def __init__(self, printer_port, force_gauge_port,
				 printer_baud=115200, force_gauge_baud=2400,
				 printer_timeout=None, force_gauge_timeout=1,
				 z_coarse_inc=.5, z_fine_inc=.1,
				) -> None:

		self.z: float            = float('inf')
		self.zeroed              = False
		self.precision: int|None = None
		self._debug_gcode        = False
		self.coarse_inc          = z_coarse_inc
		self.fine_inc            = z_fine_inc

		# Init printer communication
		self.printer = Serial(port=printer_port, baudrate=printer_baud, timeout=printer_timeout)
		print(self.G('M114'))
		#Turn off print cooling fan
		self.G('M107')
		# Set printer to relative mode
		self.G('G91')
		# Disable Z stepper timeout
		self.G('M18 S0 Z')
		# Set default feedrate
		self.G(f'G0 Z0 F{DEFAULT_FEEDRATE}')

		# Init force gauge communication
		if force_gauge_port is not None:
			self.force_serial = Serial(port=force_gauge_port, baudrate=force_gauge_baud, timeout=force_gauge_timeout)
			self.force_thread = ReaderThread(self.force_serial, ThreadedForceMeter)
			self.force_thread.start()
			_, self.force = self.force_thread.connect()
			while not self.force.ready:
				pass
			print('   Wait for force to stabilize')
			print(f'Force now {self.stable_force()}')


	def get_force(self) -> float:
		"""Return the next new value from the force meter, blocking until there is
		one."""
		while not self.force.new_value:
			pass
		return self.force.value


	def get_tsforce(self) -> tuple[float,float]:
		"""Return a tuple of (timestamp, force) for the next new value from the
		force meter, blocking until a new value appears."""
		while not self.force.new_value:
			pass
		return self.force.timestamp, self.force.value


	def z_endstop(self) -> bool:
		"""Ask the printer for the state of the z endstop and return True if it's
		triggered and False if not."""
		self.printer.reset_input_buffer()
		self.printer.reset_output_buffer()
		result = b'Unknown'
		while b'Unknown' in result:
			self.printer.write(b'M119\n')
			result = self.printer.read_until(b'ok\n')
		if b'z_min: TRIGGERED' in result:
			return True
		elif b'z_min: open' in result:
			return False

		print(result)
		raise ValueError("Didn't find `z_min: {TRIGGERED|open}` in result above")


	def avg_force(self, n=3) -> float:
		"""Get `n` force readings and return the average."""
		vals = [self.get_force() for _ in range(n)]
		return sum(vals) / len(vals)


	def stable_force(self, n_same=3, max_n=20) -> float:
		"""Get force readings until `n_same` subsequent readings are the same.
		Raise an error after `max_n` readings."""
		same = 0
		count = 0
		last = self.get_force()
		while same < n_same:
			if (v := self.get_force()) != last:
				last = v
				same = 0
			else:
				same += 1
			if (count := count + 1) >= max_n:
				raise ValueError('Max samples exceeded, readings never stabilized')
		return last



	def G(self, gcode, wait=True) -> str:
		"""Send a Gcode command and wait for 'ok\n'. Return the printer's response, if any."""
		assert gcode
		if self._debug_gcode:
			print(f'[yellow]{gcode}[/]')
		self.printer.write(bytes(gcode.encode()) + b'\n')
		if wait:
			return self.printer.read_until(b'ok\n').removesuffix(b'ok\n').strip().decode()
		return ''


	def move_z(self, inc:float, direction:Direction, feedrate=None, wait=True, pre='', post='') -> None:
		"""Move the Z axis by `inc` mm"""
		if inc == 0: return
		inc = inc2dir(inc, direction)
		self.G((f'{pre}\n' if pre else '') +
				   (f'G0 Z{inc}' + (f' F{feedrate}' if feedrate is not None else '')) +
				   (f'{post}' if post else ''),
				 wait)
		if wait:
			self.G('M400')   #Wait for moves to complete

		#Only keep track of z location once we've zeroed z
		if self.zeroed:
			self.z += inc


	def move_z_until(self, inc:float, direction:Direction, test=Callable[[float], bool],
									max_move=float('inf')) -> float:
		"""Move z by `inc` until `test(force)` is True. If `max` is specified, move by at
		most that amount."""
		dist = 0.0
		inc = inc2dir(inc, direction)

		#Try to get a stable force reading
		try:
			force = self.stable_force()
		except ValueError:
			force = self.get_force()
			print(f"Force didn't stabilize, going with {force} and hoping for the best")

		while abs(dist) <= max_move and not test(force):
			if abs(force) > 4.9:
				raise ValueError(f"Force {force} too high, aborting!")
			dist = dist + inc
			self.move_z(inc, direction)
			force = self.get_force()
			print(f'    Moved {dist}, force is {force}')

		print(f'    -> Test on force {force} = {test(force)}')

		if not test(force := self.stable_force()):
			print(f'    Stable force of {force} failed test, trying slowly')

			while abs(dist) <= max_move and not test(force):
				if abs(force) > 4.9:
					raise ValueError(f"Force {force} too high, aborting!")
				dist = dist + inc
				self.move_z(inc, direction)
				force = self.stable_force()
				print(f'    Moved {dist}, stable force is {force}')

		if abs(dist) >= max_move:
			print(f'  Stopped due to abs({dist}) >= max_move {max_move}')

		return dist


	def drop_z_until_stop(self, inc:float, direction:Direction=DOWN) -> float:
		"""Drop the z axis zy `inc` until either the force meter reads nonzero or
		the zstop switch is activated."""
		z = 0.0
		inc = inc2dir(inc, direction)
		while True:
			f = self.get_force()
			endstop = self.z_endstop()
			print(f'Relative z: {z}, force: {f}, endstop: {endstop}')
			if f != 0 or endstop:
				break
			self.move_z(inc, direction)
			z += inc
		return z


	def move_to_zero(self, inc:float|None=None) -> float:
		"""If force isn't zero, move in the opposite direction until it is."""
		moved = 0.0
		inc = (inc or self.fine_inc) * 2
		while (force := self.get_force()) != 0:
			inc = min(inc / 2, MIN_Z_MOVE)
			direction = self.force.direction.flip()
			print(f'Move to zero: force is {force} ({self.force.direction}), moving by {inc} {direction} until 0')
			moved += self.move_z_until(inc=inc,
																direction=direction,
																test=samesign_or_zero(direction),
																max_move=min(5,abs(self.coarse_inc*10)))
		print(f'Moved by {moved} to get zero force')
		return moved


	def zero_z_axis(self, direction:Direction=DOWN, backoff=True) -> None:
		"""Manually zero the printer on z. Move z by `self.coarse_inc` in `direction`
		until either the endstop closes or the force gauge registers != 0, then
		back off and do it again with `self.fine_inc`."""
		rel_z = 0.0

		print('Zero z: move to zero')
		self.move_to_zero()

		if force := self.get_force():
			if force := self.stable_force():
				raise ValueError(f"Uh-oh, force of {force} still nonzero")

		#Move with coarse movement until we get a stop
		print(f'Coarse move {direction} by {self.coarse_inc} until f != 0')
		rel_z += self.move_z_until(inc=self.coarse_inc, direction=direction, test=nonzeroeps)
		if self.get_force() == 0 and self.stable_force() == 0:
				raise ValueError("Stopped zeroing but force is 0")

		#Back off until force is 0
		print(f'Coarse backoff {direction.flip()} by {self.coarse_inc} until f == 0 (now {self.get_force()})')
		# self.move_z_until(inc=self.coarse_inc, direction=direction.flip(), test=oppsign_or_zero(direction.flip()))
		# if self.get_force != 0:
		self.move_to_zero(inc=self.coarse_inc)

		#Drop with fine movement until we get a stop
		print(f'Fine move {direction} by {self.fine_inc} until f != 0 (now {self.get_force()})')
		self.move_z_until(inc=self.fine_inc, direction=direction, test=nonzeroeps)

		self.z = 0
		self.zeroed = True

		#Finally back off again until we get zero force
		if backoff:
			print(f'Fine backoff {direction.flip()} by {self.fine_inc} until f == 0 (now {self.get_force()})')
			self.move_to_zero(inc=self.fine_inc)
			# self.move_z_until(inc=self.fine_inc, direction=direction.flip(), test=zero)

		print(f"Zeroed Z axis, backed off to {self.z}, force = {self.get_force()}")



	def test_loop(self, z_inc, repetitions, start_direction:Direction, test_no=1, smooth=False,
							 max_down=0.0, max_up=0.0, stop_after=30, **kwargs) -> list[TestResult]:
		"""Conduct `repetitions` cycles of testing. Start in `start_direction`;
		move by `z_inc`; after snap-through, reverse. Set `smooth` to True to use
		`smooth_move_test()` (`z_inc` will then be the total displacement).
		`kwargs` are passed to `careful_move_test()` or `smooth_move_test()`.  If
		`down_max` is given then `up_max` must also be; in this case, don't detect
		snap-through, just move up and down.  Return a CSV-formatted string.  """
		data: list[TestResult] = []
		direction = start_direction
		kwargs.pop('return_to_zero', None)

		test = partial(self.smooth_move_test if smooth else self.careful_move_test,
									 z_inc, return_to_zero=False, **kwargs)

		if max_down != max_up and 0 in (max_down, max_up):
			raise ValueError(f"Both max_down and max_up must be specifed (given: {max_down}, {max_up}")

		max_displacement = stop_after
		for rep in range(test_no, repetitions+1):
			if max_down: max_displacement = max_down
			data.extend(test(direction=direction, test_no=rep,
										min_displacement=max_displacement*.5,
										stop_after=max_displacement))
			# if max_down == 0:
			# 	if rep == test_no:
			# 		max_displacement = abs(data[-1].displacement + 2)
			direction = direction.flip()
			if max_up: max_displacement = max_up
			data.extend(test(direction=direction, test_no=rep,
										min_displacement=max_displacement*.5,
										stop_after=max_displacement))
			direction = direction.flip()

		return data



	def careful_move_test(self, z_inc, direction:Direction, n_samples=1,
											 test_no=-1, return_to_zero=True, stop_after=100,
											 min_displacement=0,
											 **kwargs) -> list[TestResult]:
		"""Conduct a moving force test. Move the meter until the force goes
		non-zero (touching), then move until it reads zero (snap-through) or the
		meter has been moved more than `stop_after` mm."""
		print(f'Carefully testing moving {direction} by {z_inc}mm')

		if abs(min_displacement) > abs(stop_after):
			raise ValueError(f"{abs(min_displacement)=} must be <= {abs(stop_after)=}")

		if (f := self.get_force()) != 0:
			print(f"Force isn't 0, it's {f}")
			self.move_to_zero()

		z_inc = inc2dir(z_inc, direction)
		displacement = 0
		data: list[TestResult] = []

		def move_one(z_inc, direction):
			nonlocal displacement
			data.append(TestResult(
										timestamp=time(),
										test_type='careful',
										direction=direction,
										z=self.z,
										displacement=displacement,
										force=(f := self.avg_force(n=n_samples)),
										testno=test_no,
								))
			print(data[-1])
			self.move_z(z_inc, direction)
			displacement += z_inc
			return f

		#Move until the probe is just touching the object
		print(f'\nTEST PRE-MOVE by {z_inc} ({direction}) until force != 0')
		self.move_z_until(inc=self.fine_inc, direction=direction, test=nonzero)

		print(f'\nForce: {self.get_force()} -> START MOVE TEST stepping by {z_inc} ({direction}) -----')

		#Move in the test direction until the force is zero or the opposite
		# direction from the test direction. Ensure we move at least
		# min_displacement.
		print(f"Move until at least min of {min_displacement}")
		while abs(displacement) < abs(min_displacement):
			f = move_one(z_inc, direction)
		print(f"Move until force == 0 or opposite of {direction}, but to max of {stop_after}")
		while (f != 0 or samedir(direction, f)) and abs(displacement) < stop_after:
			f = move_one(z_inc, direction)

		print(f'Force: {f}; Gate triggered or max displacement {stop_after}, moving until force is 0')

		#If force isn't zero, move till it is
		if f != 0:
			self.move_to_zero()
			# self.move_z_until(inc=self.fine_inc, direction=self.force.direction.flip(),
			# 									test=zero, max_move=min(5,abs(z_inc*5)))

		print(f'oppdir_or_zero({direction=}, {f=}) = {oppdir_or_zero(direction, f)}')
		print(f'Force: {f} -> END MOVE TEST stepping by {z_inc} ({direction}) -----\n')

		if return_to_zero:
			self.move_z(displacement, direction.flip(), feedrate=DEFAULT_FEEDRATE)

		return data


	def smooth_move_test(self, target_displacement:float, direction:Direction,
											return_to_zero=False, feedrate=DEFAULT_FEEDRATE, test_no=-1,
											zero_z=True,
											**kwargs) -> list[TestResult]:
		"""Conduct a pushing force test based on known displacement values. Feedrate is mm/minute."""

		#Start by zeroing without backoff so the probe is touching
		if zero_z:
			self.zero_z_axis(direction=direction, backoff=False)

		data = [TestResult(
								timestamp=time(),
								direction=direction,
								test_type='smooth',
								z=self.z,
							  displacement=0,
							  force=self.get_force(),
								testno=test_no,
							)]
		start_z = self.z

		#Tell the printer to move through the entire length of the displacement
		target_displacement = inc2dir(target_displacement, direction)
		self.move_z(target_displacement, direction, feedrate=feedrate, wait=False,
							pre='M300 S440 P20')

		#Read until snap-through
		while force2dir(data[-1].force) == direction:
			t, f = self.get_tsforce()
			data.append(TestResult(
				timestamp=t,
				test_type='smooth',
				direction=direction,
				force=f,
				testno=test_no,
				))
			print(data[-1])

		#Wait until printer stops moving then add a last point with assumption that
		# target displacement has been reached
		self.G('M400')
		self.G('M300 S100 P20')
		t, f = self.get_tsforce()
		data.append(TestResult(timestamp=t,
										direction=direction,
										test_type='smooth',
										force=f,
										testno=test_no,
										z=self.z,
										displacement=target_displacement))
		print(data[-1])

		if return_to_zero:
			self.move_z(target_displacement, direction.flip(), feedrate=DEFAULT_FEEDRATE)

		return data



if __name__ == "__main__":
	from clize import run, ArgumentError, parameters, parser

	@parser.value_converter
	def arg2dir(arg):
		if isinstance(arg, Direction): return arg
		return UP if arg.upper == 'UP' else DOWN

	def main(force_gauge_port, printer_port, *,
			 force_gauge_baud=2400, printer_baud=115200,
			 force_gauge_timeout=1, printer_timeout=None,

			 feedrate=DEFAULT_FEEDRATE,

			 first_move_z_up_by=0.0,
			 first_move_z_down_by=0.0,
			 exit_after_first_z_move=False,
			 do_zero=True,
			 zero_coarse_inc=.5,
			 zero_fine_inc=.1,

			 test_type:parameters.one_of('careful', 'smooth', case_sensitive=False)='careful',
			 test_direction:arg2dir=DOWN,
			 test_loops=0,
			 test_num=1,
			 n_samples=1,
			 careful_inc=.25,
			 stop_after=15,
			 return_to_zero_after_test=True,
			 outfile:Path='',
			 # smooth_displacement=0.0,

			 debug_gcode=False,
			) -> None:
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


		Test options:

		:param test_type: Specify 'careful' or 'smooth'
		:param test_direction: Do a movement test in this direction.
		:param test_loops: Do repeated loops; if 0, only move a single direction then stop.
		:param test_num: Start numbering loop tests here.
		:param n_samples: Average this many samples per increment.
		:param careful_inc: Step this many mm per measurement.
		:param stop_after: Stop moving after this many mm if no snap-through has happened.
		:param return_to_zero_after_test: Return to the zeroed point if test_loops == 0.
		:param outfile: Write a CSV file here.


		Other options:

		:param debug_gcode: Print every Gcode command as it is issued.
		"""

		meter = FDMeter(
				printer_port        = printer_port,
				force_gauge_port    = None if exit_after_first_z_move else force_gauge_port,
				printer_baud        = printer_baud,
				force_gauge_baud    = force_gauge_baud,
				printer_timeout     = printer_timeout,
				force_gauge_timeout = force_gauge_timeout,
				z_coarse_inc        = z_coarse_inc,
				z_fine_inc          = z_fine_inc,
		)
		meter._debug_gcode = debug_gcode

		meter.move_z(first_move_z_up_by, UP, feedrate=feedrate)
		meter.move_z(first_move_z_down_by, DOWN, feedrate=feedrate)

		if exit_after_first_z_move and max(first_move_z_up_by, first_move_z_down_by):
			print('Moved z, exiting')
			sys.exit(0)

		z = meter.z_endstop()
		print(f'endstop {z}')

		if do_zero:
			print('Zeroing z axis')
			meter.zero_z_axis()

		if test_direction:
			direction = Direction(test_direction)
			print(f'Going to do test {test_direction}')
			if test_type == 'smooth' or test_loops <= 0:
				data = meter.careful_move_test(careful_inc,
																			 direction,
																			 n_samples=n_samples,
																			 return_to_zero=return_to_zero_after_test)
			if test_loops > 0:
				data = meter.test_loop(
						careful_inc,
						test_loops, direction,
													 test_no=test_num,
													 n_samples=n_samples,
													 return_to_zero=return_to_zero_after_test)

			if outfile:
				results_to_csv(data, outfile)
				print(f'Saved data to {outfile}')


	run(main)

