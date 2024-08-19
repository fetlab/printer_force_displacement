from serial import Serial
import sys


class FDMeter:
	def __init__(self, printer_port, force_gauge_port,
				 printer_baud=115200, force_gauge_baud=2400,
				 printer_timeout=None, force_gauge_timeout=1) -> None:
		self.printer = Serial(port=printer_port, baudrate=printer_baud, timeout=printer_timeout)
		self.force = Serial(port=force_gauge_port, baudrate=force_gauge_baud, timeout=force_gauge_timeout)

		self.z: float|None = None
		self.precision: int|None = None

		# Init printer communication
		print(self.G('M114'))
		# Set printer to relative mode
		self.G('G91')

		# Init force gauge communication
		if (f := self.get_force()) > 0:
			raise ValueError(f'Force on startup is {f}, but should be zero')
		print(f'Force now {self.get_force()}')


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


	def init_force(self) -> None:
		"""Initialze the force meter. The serial port displays exactly what is on
		the display with no line breaks and no units, so we have to guess how many
		bytes to read. There's either a `-` or `0`. Look for a `.` to know where to
		break the string. There's no way to figure out how many bytes to read after
		a decimal point to divide in the right place until we get a negative number."""
		#Cases:
		#  Kgf: -0.000-0.00000.00000.00000.000
		#       s1.123s1.123s1.123
		#         .12345.12345.
		#  Lbf: -01.00-00.71000.13000.13
		#       s12.34s12.34s12.34
		#          .12345.12345.
		#    N: -00.23-00.87000.29000.56
		#       s12.34s12.34s12.34
		#          .12345.12345.
		#
		# Positive:
		# 	N/L: 000.00|000.00 -> 000.00000.00000.00 -> 000.00000.00000.00
		# 	Kgf: 00.000|00.000 -> 00.00000.00000.000 ->  00.00000.00000.000
		# Negative:
		# 	N/L: -00.00|-00.00 -> -00.00-00.00-00.00 -> -00.00-00.00-00.00
		# 	Kgf: -0.000|-0.000 -> -0.000-0.000-0.000 ->  -0.00-00.00-00.00-
		print('Please make the force meter read negative')

		self.force.reset_input_buffer()

		val = b''
		#Loop to account for serial timeout
		while len(val) < 6 or val.count(b'-') != 2:
			val = self.force.read_until(b'-')
			val += self.force.read_until(b'-')

		try:
			between = val.split(b'-')[1]
		except IndexError:
			print(f'{between=}')
			raise

		if len(between) != 5:
			raise ValueError(f"Too many bytes between '-'s in '{between.decode()}'")
		if b'.' not in between:
			raise ValueError(f"Didn't find '.' in '{between.decode()}'")

		self.precision = len(between.split(b'.')[1])
		if self.precision not in [3,4]:
			raise ValueError(f"Unknown number of bytes of precison {self.precision}")

		print(f"Found {self.precision} bytes after '.' so unit is ", end='')
		if self.precision == 3:
			print("Kgf")
		elif self.precision == 4:
			print("either Lbf or N")


	def get_force(self) -> float:
		if self.precision is None:
			self.init_force()

		self.force.reset_input_buffer()
		val = self.force.read_until(b'.')
		val += self.force.read(self.precision)
		if len(val) != 6:
			val = self.force.read(6)
		if len(val) < 6:
			raise ValueError(f"Only read {len(val)} bytes from force gauge; is it on?")
		return float(val)


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
		self.G('M400')

		#Only keep track of z location once we've zeroed z
		if self.z is not None:
			self.z += inc


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
		self.move_z(max(abs(backoff_amount), backoff_at_least))

		print(f"Zeroed Z axis, backed off to {self.z}")



def main(force_gauge_port, printer_port, *,
		 force_gauge_baud=2400, printer_baud=115200,
		 force_gauge_timeout=1, printer_timeout=None,
		 do_zero=True,
		 backoff_at_least=4,
		 first_move_z_by=0,
		 exit_after_first_z_move=False) -> None:

	meter = FDMeter(printer_port, force_gauge_port,
					printer_baud, force_gauge_baud,
					printer_timeout, force_gauge_timeout)

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


if __name__ == "__main__":
	from clize import run
	run(main)

