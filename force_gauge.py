from serial import Serial
from serial.threaded import ReaderThread
from threaded_force_meter import ThreadedForceMeter
from time import time
import sys

OVERLOAD_N = 5
OVERLOAD_LBF = 1.1
OVERLOAD_KGF = 0.5



class FDMeter:
	def __init__(self, printer_port, force_gauge_port,
				 printer_baud=115200, force_gauge_baud=2400,
				 printer_timeout=None, force_gauge_timeout=1) -> None:

		self.z: float|None = None
		self.precision: int|None = None

		# Init printer communication
		self.printer = Serial(port=printer_port, baudrate=printer_baud, timeout=printer_timeout)
		print(self.G('M114'))
		# Set printer to relative mode
		self.G('G91')

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
		while not self.force.new_value:
			pass
		return self.force.value


	def z_endstop(self) -> bool:
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



	def get_force_until_threshold(self, start_threshold=0.0, end_threshold=0.0) -> list[tuple[float,float]]:
		"""Read force as quickly as possible from the force meter until the
		force reaches or exceed start_threshold, discarding the measurements.
		Then read until the force reaches or exceedes end_threshold. Return a list of
		[(timestamp, force), ...] ."""
		data : list[tuple[float,float]] = []

		#Init reading and get to the right alignment
		val = self.get_force()
		if val == start_threshold:
			print(f"Wait for force to exceed {start_threshold}")
		while val == start_threshold:
			val = self.get_force()
		print(f"Force became {val}")

		print(f"Wait for force to reach {end_threshold}")
		while val != end_threshold:
			val = self.get_force()
			data.append((time(), val))

		return data


	def avg_force(self, n=3) -> float:
		vals = [self.get_force() for _ in range(n)]
		return sum(vals) / len(vals)


	def G(self, gcode) -> str:
		"""Send a command and wait for 'ok\n'. Return the printer's response, if any."""
		self.printer.write(bytes(gcode.encode()) + b'\n')
		return self.printer.read_until(b'ok\n').removesuffix(b'ok\n').strip().decode()


	def move_z(self, inc) -> None:
		"""Move the Z axis by `inc` mm"""
		self.G(f'G0 Z{inc}')
		self.G('M400')   #Wait for moves to complete

		#Only keep track of z location once we've zeroed z
		if self.z is not None:
			self.z += inc


	def move_z_until_zero_force(self, inc:float):
		while (f := self.get_force()) != 0:
			self.move_z(inc)


	def _drop_z_until_stop(self, inc:float) -> float:
		if inc >= 0: raise ValueError(f"Bad inc: {inc} >= 0")
		z:float = 0
		while True:
			f = self.get_force()
			endstop = self.z_endstop()
			print(f'Relative z: {z}, force: {f}, endstop: {endstop}')
			if f != 0 or endstop:
				break
			self.move_z(inc)
			z += inc
		return z


	def zero_z_axis(self, z_coarse_inc=-1, z_fine_inc=-.25, backoff_at_least=0) -> None:
		"""Manually zero the printer on z. Drop z by z_coarse_inc until either the endstop closes or
		the force gauge registers != 0, then back off and do it again with
		z_fine_inc."""
		rel_z:float = 0

		#Drop with coarse movement until we get a stop
		rel_z += self._drop_z_until_stop(z_coarse_inc)
		if self.get_force() == 0:
			raise ValueError("Stopped zeroing but force is 0")

		#Back off until force is 0 or until we get back to the starting point
		start_rel_z = rel_z
		while (f := self.get_force()) != 0 and rel_z < 0:
			self.move_z(-z_coarse_inc)
			rel_z += -z_coarse_inc
		if f := self.get_force():
			raise ValueError(f"Backed off but force is {f}, not 0")
		backoff_amount = start_rel_z - rel_z

		#Drop with fine movement until we get a stop
		self._drop_z_until_stop(z_fine_inc)
		if self.get_force() == 0:
			raise ValueError("Stopped zeroing but force is 0")
		self.z = 0

		#Finally back off again
		self.move_z_until_zero_force(-z_fine_inc)

		print(f"Zeroed Z axis, backed off to {self.z}")


	def careful_push_test(self, z_inc=-.25, n_samples=1, stop_after=15,
							 return_to_zero=True) -> str:
		"""Conduct a pushing force test. Drop the meter until the force goes
		non-zero, then drop until it goes until zero or the meter has been dropped
		more than `stop_after` mm."""
		if z_inc >= 0:
			raise ValueError(f"z_inc must be < 0, not {z_inc}")
		if (f := self.get_force()) != 0:
			raise ValueError(f"Force isn't 0, it's {f}")

		f = 0
		while f == 0:
			self.move_z(z_inc)
			f = self.get_force()

		print('\nSTART PUSH TEST -----')

		data = ['Timestamp,Displacement (mm),Force (Kgf)']
		rel_z = 0
		while f != 0 and abs(rel_z) < stop_after:
			line = ','.join(map(str,(time(), self.z, f := self.avg_force(n=n_samples))))
			print(line)
			data.append(line)
			self.move_z(z_inc)

		print('END PUSH TEST -----\n')

		if return_to_zero:
			assert self.z is not None
			self.move_z(-self.z)

		return '\n'.join(data)


#def smooth_push_test(self, displacement:float) -> list[tuple[float, float]]:
#	"""Conduct a pushing force test based on known displacement values. Return a
#	list [(timestamp, force),...] ."""
#	if self.z is None:
#		raise ValueError("Printer not zeroed, too dangerous to continue.")

#	if (val := self.get_force) != 0:
#		raise ValueError(f"Probe is touching something; force reads {val}")

#	#Tell the printer to move through the entire length of the displacement
#	G('G0 {displacement - self.z}')

#	#Now read data as fast as we can!



def main(force_gauge_port, printer_port, *,
		 force_gauge_baud=2400, printer_baud=115200,
		 force_gauge_timeout=1, printer_timeout=None,
		 do_zero=True,
		 backoff_at_least=1,
		 first_move_z_by=0.0,
		 exit_after_first_z_move=False,
		 do_push_test=False,
		 n_samples=1,
		 outfile='',
		 return_to_zero_after_test=True,
		) -> None:

	meter = FDMeter(
			printer_port        = printer_port,
			force_gauge_port    = None if exit_after_first_z_move else force_gauge_port,
			printer_baud        = printer_baud,
			force_gauge_baud    = force_gauge_baud,
			printer_timeout     = printer_timeout,
			force_gauge_timeout = force_gauge_timeout,
	)

	if first_move_z_by:
		meter.move_z(first_move_z_by)
		if exit_after_first_z_move:
			print(f'Moved by {first_move_z_by}, exiting')
			sys.exit(0)

	z = meter.z_endstop()
	print(f'endstop {z}')

	if do_zero:
		print('Zero z axis!')
		meter.zero_z_axis(backoff_at_least=backoff_at_least)

	if do_push_test:
		csv = meter.careful_push_test(n_samples=n_samples,
												return_to_zero=return_to_zero_after_test)
		if outfile:
			with open(outfile, 'w') as f:
				f.write(csv)
			print(f'Saved data to {outfile}')


if __name__ == "__main__":
	from clize import run
	run(main)

