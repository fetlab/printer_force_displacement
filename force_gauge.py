from enum import Enum
from serial import Serial
from serial.threaded import ReaderThread
from threaded_force_meter import ThreadedForceMeter
from time import time
from typing import Callable
from rich import print
from dataclasses import dataclass
import sys, csv

DEFAULT_FEEDRATE = 180

#Movement directions
class Direction(Enum):
	DOWN = -1
	UP   = 1
	def flip(self):
		return self.__class__(self.value * -1)
	def __str__(self):
		return 'UP' if self == self.__class__.UP else 'DOWN'

UP = Direction.UP
DOWN = Direction.DOWN
#Set sign of `inc` based on `dir`
inc2dir = lambda inc, direction: abs(inc) * direction.value

@dataclass
class TestResult:
	timestamp:    float
	direction:    Direction
	displacement: float
	force:        float
	testno:       int = -1


def results_to_csv(test_results: list[TestResult], outfile=str):
	fieldnames = list(test_results[0].__dict__.keys())
	with open(outfile, 'w', newline='') as f:
		writer = csv.DictWriter(f, fieldnames=fieldnames)
		writer.writeheader()
		writer.writerows(r.__dict__ for r in test_results)


class FDMeter:
	def __init__(self, printer_port, force_gauge_port,
				 printer_baud=115200, force_gauge_baud=2400,
				 printer_timeout=None, force_gauge_timeout=1) -> None:

		self.z: float|None = None
		self.precision: int|None = None
		self._debug_gcode = False

		# Init printer communication
		self.printer = Serial(port=printer_port, baudrate=printer_baud, timeout=printer_timeout)
		print(self.G('M114'))
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
			if (f := self.get_force()) > 0:
				raise ValueError(f'Force on startup is {f}, but should be zero')
			print(f'Force now {self.get_force()}')


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


	def G(self, gcode) -> str:
		"""Send a Gcode command and wait for 'ok\n'. Return the printer's response, if any."""
		assert gcode
		if self._debug_gcode:
			print(f'[yellow]{gcode}[/]')
		self.printer.write(bytes(gcode.encode()) + b'\n')
		return self.printer.read_until(b'ok\n').removesuffix(b'ok\n').strip().decode()


	def move_z(self, inc:float, direction:Direction, feedrate=None, wait=True) -> None:
		"""Move the Z axis by `inc` mm"""
		if inc == 0: return
		inc = inc2dir(inc, direction)
		self.G(f'G0 Z{inc}' + (f' F{feedrate}' if feedrate is not None else ''))
		if wait:
			self.G('M400')   #Wait for moves to complete

		#Only keep track of z location once we've zeroed z
		if self.z is not None:
			self.z += inc


	def move_z_until(self, inc:float, direction:Direction, test=Callable[[float], bool], max_move=float('inf')) -> float:
		"""Move z by `inc` until `test(force)` is True. If `max` is not None, move by at
		most that amount."""
		dist = 0.0
		inc = inc2dir(inc, direction)
		while abs(dist) <= max_move and not test(f := self.get_force()):
			dist = dist + inc
			self.move_z(inc, direction)
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


	def zero_z_axis(self, z_coarse_inc=1, z_fine_inc=.25, direction:Direction=DOWN) -> None:
		"""Manually zero the printer on z. Drop z by z_coarse_inc until either the endstop closes or
		the force gauge registers != 0, then back off and do it again with
		z_fine_inc."""
		rel_z = 0.0

		#If we have a force reading, move until we get to zero
		# If it's positive, move up; negative, move down. z_fine_inc should already
		# have the correct sign, so multiply by -1 to move up!
		if force := self.get_force():
			print(f'Initial force is {force}, moving until 0')
			moved = self.move_z_until(z_fine_inc, UP if force < 0 else DOWN,
																 lambda f:f==0, max_move=abs(z_coarse_inc*2))
			print(f'Moved by {moved}')

		if force := self.get_force():
			raise ValueError(f"Force of {force} still nonzero")


		#Drop with coarse movement until we get a stop
		rel_z += self.drop_z_until_stop(z_coarse_inc, direction)
		if self.get_force() == 0:
			raise ValueError("Stopped zeroing but force is 0")

		#Back off until force is 0
		self.move_z_until(z_coarse_inc, direction.flip(), lambda f:f==0)

		#Drop with fine movement until we get a stop
		self.move_z_until(z_fine_inc, direction, lambda f:f!=0)

		self.z = 0

		#Finally back off again
		self.move_z_until(z_fine_inc, direction.flip(), lambda f:f==0)

		print(f"Zeroed Z axis, backed off to {self.z}")



	def test_loop(self, z_inc, repetitions, start_direction:Direction, **kwargs) -> list[TestResult]:
		"""Conduct `repetitions` cycles of testing. Start in `start_direction`;
		move by `z_inc`; after snap-through, reverse. Other arguments are passed to
		`careful_move_test()` Return a CSV-formatted string.
		"""
		data: list[TestResult] = []
		direction = start_direction

		for rep in range(repetitions):
			data.extend(self.careful_move_test(z_inc, direction, test_no=rep+1, return_to_zero=False, **kwargs))
			direction = direction.flip()
			data.extend(self.careful_move_test(z_inc, direction, test_no=rep+1, return_to_zero=False, **kwargs))

		return data



	def careful_move_test(self, z_inc, direction:Direction, n_samples=1,
											 stop_after=15, test_no=-1, return_to_zero=True) -> list[TestResult]:
		"""Conduct a moving force test. Move the meter until the force goes
		non-zero (touching), then move until it reads zero (snap-through) or the
		meter has been dropped more than `stop_after` mm."""
		print(f'Carefully testing moving {direction} by {z_inc}mm')

		if (f := self.get_force()) != 0:
			raise ValueError(f"Force isn't 0, it's {f}")

		z_inc = inc2dir(z_inc, direction)
		displacement = 0
		data: list[TestResult] = []

		f = 0
		while f == 0:
			self.move_z(z_inc, direction)
			displacement += z_inc
			f = self.get_force()
			if((self.force.pushing and direction == UP) or
				 (self.force.pulling and direction == DOWN)):
				raise ValueError(f"Force {f} is in the opposite direction of the test {direction}")

		print('\nSTART PUSH TEST -----')

		rel_z = 0
		while f != 0 and abs(rel_z) < stop_after:
			data.append(TestResult(
										timestamp=time(),
										direction=direction,
										displacement=displacement,
										force=self.avg_force(n=n_samples),
										testno=test_no,
								))
			print(data[-1])
			self.move_z(z_inc, direction)
			displacement += z_inc

		print('END PUSH TEST -----\n')

		if return_to_zero:
			self.move_z(displacement, direction.flip(), feedrate=DEFAULT_FEEDRATE)

		return data


	def smooth_move_test(self, displacement:float, direction:Direction, return_to_zero=True, feedrate=60) -> str:
		"""Conduct a pushing force test based on known displacement values. Return a
		list [(timestamp, force),...] . Feedrate is mm/minute."""
		if self.z is None:
			raise ValueError("Printer not zeroed, too dangerous to continue.")

		if (val := self.get_force()) != 0:
			raise ValueError(f"Probe is touching something; force reads {val}")

		data = []
		start_z = self.z

		#Tell the printer to move through the entire length of the displacement
		displacement = inc2dir(displacement, direction)
		self.move_z(displacement - self.z, direction, feedrate=feedrate, wait=False)

		#Read until the probe touches
		t, f = self.get_tsforce()
		while f == 0:
			t, f = self.get_tsforce()

		#Read until snap-through
		data.append((t,f))
		while f != 0:
			t, f = self.get_tsforce()
			data.append((t,f))
			print(t,f)

		out = '\n'.join(['Timestamp,Force'] + [f'{t},{f}' for t,f in data])
		print(out)

		if return_to_zero:
			self.move_z(displacement, direction.flip(), feedrate=DEFAULT_FEEDRATE)

		return out



if __name__ == "__main__":
	from clize import run, ArgumentError, parameters

	def main(force_gauge_port, printer_port, *,
			 force_gauge_baud=2400, printer_baud=115200,
			 force_gauge_timeout=1, printer_timeout=None,

			 feedrate=DEFAULT_FEEDRATE,

			 first_move_z_up_by=0.0,
			 first_move_z_down_by=0.0,
			 exit_after_first_z_move=False,
			 do_zero=True,

			 do_test:parameters.one_of('UP', 'DOWN', case_sensitive=False)='',
			 n_samples=1,
			 careful_inc=.25,
			 stop_after=15,
			 return_to_zero_after_test=True,
			 outfile='',
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


		Test options:

		:param do_test: Do a movement test in this direction.
		:param n_samples: Average this many samples per increment.
		:param careful_inc: Step this many mm per measurement.
		:param stop_after: Stop moving after this many mm if no snap-through has happened.
		:param return_to_zero_after_test: Return to the zeroed point.
		:param outfile: Write a CSV file here.


		Other options:

		:param debug: Print every Gcode command as it is issued.
		"""

		meter = FDMeter(
				printer_port        = printer_port,
				force_gauge_port    = None if exit_after_first_z_move else force_gauge_port,
				printer_baud        = printer_baud,
				force_gauge_baud    = force_gauge_baud,
				printer_timeout     = printer_timeout,
				force_gauge_timeout = force_gauge_timeout,
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

		if do_test:
			direction = do_test.upper()
			print(f'Going to do test {do_test}')
			if smooth_displacement:
				csv = meter.smooth_move_test(smooth_displacement,
																 UP if direction == 'UP' else DOWN,
														return_to_zero=return_to_zero_after_test)
			else:
				data = meter.careful_move_test(careful_inc,
																 UP if direction == 'UP' else DOWN,
																	n_samples=n_samples,
														return_to_zero=return_to_zero_after_test)

			if outfile:
				results_to_csv(data, outfile)
				print(f'Saved data to {outfile}')


	run(main)

