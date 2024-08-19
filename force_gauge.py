from serial import Serial
from clize import run
import sys


def z_endstop(printer):
	printer.reset_output_buffer()
	printer.write(b'M119\n')
	m119_result = printer.read_until(b'ok\n')
	if b'z_min: TRIGGERED' in m119_result:
		return True
	elif b'z_min: open' in m119_result:
		return False

	print(m119_result)
	raise ValueError("Didn't find `z_min: {TRIGGERED|open}` in result above")


def get_force(force):
	force.reset_input_buffer()
	val = force.read_until(b'.')
	val += force.read(3)
	if len(val) != 6:
		val = force.read(6)
	if len(val) < 6:
		raise ValueError(f"Only read {len(val)} bytes from force gauge; is it on?")
	return float(val)


def avg_force(force, n=3):
	vals = [get_force(force) for i in range(n)]
	return sum(vals)/len(vals)


def G(printer, gcode):
	"""Send a command and wait for 'ok\n'. Return the printer's response, if any."""
	printer.write(bytes(gcode.encode()) + b'\n')
	return printer.read_until(b'ok\n').removesuffix(b'ok\n').strip().decode()


def z_zero(printer, force, z_inc=-1):
	"""Manually zero the printer on z. Drop z until either the endstop closes or
	the force gauge registers != 0"""
	if z_inc >= 0:
		raise ValueError("Bad z_inc")
	z = 0
	while True:
		f = get_force(force)
		endstop = z_endstop(printer)
		print(f'Relative z: {z}, force: {f}, endstop: {endstop}')
		if f != 0 or endstop:
			break
		G(printer, f'G0 Z{z_inc}')
		G(printer, 'M400')
		z += z_inc



def main(force_gauge_port, printer_port,
				 force_gauge_baud=2400, printer_baud=115200,
				 force_gauge_timeout=1, printer_timeout=None,
				 start_z=0, z_inc=1, readings_per_z=3):

	printer = Serial(port=printer_port, baudrate=printer_baud, timeout=printer_timeout)
	#Init printer communication
	print(G(printer, 'M114'))
	#Set printer to relative mode
	G(printer, 'G91')

	force = Serial(port=force_gauge_port, baudrate=force_gauge_baud, timeout=force_gauge_timeout)
	#Init force gauge communication
	print(f'Force now {get_force(force)}')

	z = z_endstop(printer)
	print(f'endstop {z}')

	print('Zero!')
	z_zero(printer, force)

if __name__ == "__main__":
	run(main)
