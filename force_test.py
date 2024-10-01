from ender_fdm import FDMeter, Direction, UP, DOWN, STILL, DEFAULT_FEEDRATE, MAX_FEEDRATE, results_to_csv, results_to_json
from clize import run, ArgumentError, parameters, parser
from pathlib import Path
import sys

@parser.value_converter
def arg2dir(arg):
	if isinstance(arg, Direction): return arg
	return Direction(arg.upper())


def main(force_gauge_port, printer_port, *,
			force_gauge_baud=2400, printer_baud=115200,
			force_gauge_timeout=1, printer_timeout=None,

			feedrate=DEFAULT_FEEDRATE,

			#Short options -U -D -X
			first_move_z_up_by:      'U'=0.0,
			first_move_z_down_by:    'D'=0.0,
			exit_after_first_z_move: 'X'=False,
			do_zero=True,
			zero_coarse_inc=.5,
			zero_fine_inc=.1,

			test_type:parameters.one_of('careful', 'smooth', case_sensitive=False)='',
			test_direction:arg2dir=STILL,
			test_loops=0,
			test_num=1,
			n_samples=1,
			careful_inc=.25,
			stop_after=15,
			min_down=0,
			max_down=0,
			max_up=0,

			return_to_zero_after_test=True,
			smooth_displacement=0.0,
			outfile:Path='',

			force_info: 'F'=0,
			quicktest: 'Q'=False,
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

	Smooth test options:

	:param smooth_displacement: Move this far in mm for smooth tests. If not
		provided or 0, will be automatically calculated by an inital careful test.

	Other options:

	:param force_info:  If > 0, print force information that many times and exit.
		If < 0, print force information until ctrl-C.
	:param quicktest: Move in first_move_z* and print force data
	:param debug_gcode: Print every Gcode command as it is issued.
	"""

	meter = FDMeter(
			printer_port        = printer_port,
			force_gauge_port    = None if exit_after_first_z_move else force_gauge_port,
			printer_baud        = printer_baud,
			force_gauge_baud    = force_gauge_baud,
			printer_timeout     = printer_timeout,
			force_gauge_timeout = force_gauge_timeout,
			z_coarse_inc        = zero_coarse_inc,
			z_fine_inc          = zero_fine_inc,
	)
	meter._debug_gcode = debug_gcode

	if force_info:
		i = 0
		while i < force_info if force_info > 0 else float('inf'):
			print(f"Force: {meter.get_force()}, direction: {meter.force.direction}")
			i += 1
		sys.exit(0)

	feedrate = min(feedrate, MAX_FEEDRATE)

	if first_move_z_up_by or first_move_z_down_by:
		direction = UP if first_move_z_up_by else DOWN
		amount = first_move_z_up_by or first_move_z_down_by
		if quicktest:
			meter.move_z(amount, direction, feedrate=feedrate, wait=False)
			while True:
				print(meter.get_tsforce())
		else:
			meter.move_z(amount, direction, feedrate=feedrate)

		if exit_after_first_z_move or quicktest:
			print('Moved z, exiting')
			sys.exit(0)

	z = meter.z_endstop()
	print(f'endstop {z}')

	#Don't zero for smooth tests since we do it in the smooth
	if do_zero and test_type.lower() != 'smooth':
		if test_direction not in (UP, DOWN):
			raise ArgumentError(f'Specify --test-direction as UP or DOWN to zero, not {test_direction}!')
		print(f'Zeroing z axis {test_direction}')
		meter.zero_z_axis(test_direction)

	if not test_type:
		sys.exit(0)

	test_params = dict(
		 feedrate                  = feedrate,
		 test_type                 = test_type,
		 test_direction            = test_direction,
		 test_loops                = test_loops,
		 test_num                  = test_num,
		 return_to_zero_after_test = return_to_zero_after_test,
		)

	if test_direction != STILL:
		data = []
		print(f'Going to do test {test_direction}')

		#For any smooth test we need to find the target displacement first, unless
		# it's already specified. For a careful test, run one if we're not looping.
		if((test_type.lower() == 'careful' and test_loops <= 0) or
			 (test_type.lower() == 'smooth'  and smooth_displacement == 0)):
			data = meter.careful_move_test(careful_inc,
																		 test_direction,
																		 n_samples=n_samples,
																		 return_to_zero=return_to_zero_after_test,
																		 min_displacement=min_down,
																		 stop_after=stop_after)
			smooth_displacement = data[-1].displacement
			print(f'Careful test found displacment {smooth_displacement}')

		#Run loops of careful or smooth
		if test_loops > 0:
			data.extend(
				meter.test_loop(
					z_inc=smooth_displacement if test_type.lower() == 'smooth' else careful_inc,
					repetitions=test_loops,
					start_direction=test_direction,
					test_no=test_num,
					smooth=test_type.lower() == 'smooth',
					n_samples=n_samples,
					stop_after=stop_after,
					max_down=max_down,
					max_up=max_up,
				)
			)

		elif test_type.lower() == 'smooth':
			data.extend(
				meter.smooth_move_test(
					target_displacement=smooth_displacement,
					direction=test_direction,
					return_to_zero=return_to_zero_after_test,
					feedrate=feedrate,
				)
			)

		if test_type.lower() == 'smooth':
			test_params.update(smooth_displacement=smooth_displacement)
		else:
			test_params.update(n_samples=n_samples, careful_inc=careful_inc,
											stop_after=stop_after)

		if outfile:
			fn = results_to_json(test_params, data, outfile)
			print(f'Saved data to {fn}')


run(main)
