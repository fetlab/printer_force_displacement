#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyserial",
#     "rich",
#     "clize",
# ]
# ///

from ender_fdm import FDMeter, Direction, UP, DOWN, STILL, DEFAULT_FEEDRATE, MAX_FEEDRATE, results_to_json
from clize import run, ArgumentError, parameters, parser
from pathlib import Path
import sys
import queue
import json
import multiprocessing
from GUI_classes.DataJson import TestConfig

def main(msg_queue: multiprocessing.Queue, cmd_queue: multiprocessing.Queue, cfg: TestConfig):
	print("Starting test with config")
	# initialize FDMeter with parameters from cfg
	meter = FDMeter(
			printer_port        = cfg.printer_port,
			force_gauge_port    = None if cfg.exit_after_first_z_move else cfg.force_gauge_port,
			printer_baud        = cfg.printer_baud,
			force_gauge_baud    = cfg.force_gauge_baud,
			printer_timeout     = cfg.printer_timeout,
			force_gauge_timeout = cfg.force_gauge_timeout,
			z_coarse_inc        = cfg.zero_coarse_inc,
			z_fine_inc          = cfg.zero_fine_inc,
	)
	meter._debug_gcode = cfg.debug_gcode

	if cfg.force_info:
		i = 0
		while i < cfg.force_info if cfg.force_info > 0 else float('inf'):
			print(f"Force: {meter.get_force()}, direction: {meter.force.direction}")
			i += 1
		sys.exit(0)

	feedrate = min(cfg.feedrate, MAX_FEEDRATE)
	
	if cfg.first_move_z_up_by or cfg.first_move_z_down_by:
		direction = UP if cfg.first_move_z_up_by else DOWN
		amount = cfg.first_move_z_up_by or cfg.first_move_z_down_by
		if cfg.quicktest:
			meter.move_z(amount, direction, feedrate=feedrate, wait=False)
			while True:
				print(meter.get_tsforce())
		else:
			meter.move_z(amount, direction, feedrate=feedrate)

		if cfg.exit_after_first_z_move or cfg.quicktest:
			print('Moved z, exiting')
			sys.exit(0)
	# Tells the GUI that a new test is starting
	msg_queue.put("connected")
	z = meter.z_endstop()
	print(f'endstop {z}')
	# convert test direction to Direction
	converted_direction = Direction.arg2dir(cfg.test_direction)
	
	if converted_direction not in (UP, DOWN):
		raise ArgumentError(f'Specify --test-direction as UP or DOWN to zero, not {converted_direction}!')
	
	# do the zeroing
	if cfg.do_zero:
		print(f'Zeroing z axis {converted_direction}')
		meter.zero_z_axis(converted_direction)

	# if test type is not specified, exit
	if not cfg.test_type:
		sys.exit(0)

	test_params = dict(
		 feedrate                  = feedrate,
		 test_type                 = cfg.test_type,
		 test_direction            = cfg.test_direction,
		 test_loops                = cfg.test_loops,
		 test_num                  = cfg.test_num,
		 return_to_zero_after_test = cfg.return_to_zero_after_test,
		 testInvialided 			= False,
		)

	if converted_direction != STILL:
		data = []
		print(f'Going to do test {converted_direction}')

		if(cfg.test_type.lower() == 'careful' and cfg.test_loops <= 0): 

			data = meter.careful_move_test(
				queue=msg_queue,
				direction=converted_direction,
				cfg=cfg
			)

		elif(cfg.test_type.lower() == 'force-limit_test'):
			data = meter.push_until_test(
				queue=msg_queue,
				converted_direction=converted_direction,
				cfg=cfg
			)
		elif(cfg.test_type.lower() == 'displacement-limit_test'):
			data = meter.push_until_dist_test(
				queue=msg_queue,
				direction=converted_direction,
				cfg=cfg,
			)
		elif(cfg.test_type.lower() == 'custom'):
			data = meter.custom_move_test(
				msg_queue=msg_queue,
				cmd_queue=cmd_queue,
				direction=converted_direction,
				cfg=cfg)
		# Run loops of careful have not messed with this feature
		if cfg.test_loops > 0:
			update = meter.test_loop(
					z_inc= cfg.careful_inc,
					repetitions=cfg.test_loops,
					start_direction=cfg.test_direction,
					test_no=cfg.test_num,
					smooth=cfg.test_type.lower() == 'smooth',
					n_samples=cfg.n_samples,
					stop_after=cfg.stop_after,
					max_down=cfg.max_down,
					max_up=cfg.max_up,
				)
			data.extend(update)
			
		# have not understood why the test parameters needs to be updated here. why should they change?
		if cfg.test_type.lower() =='careful':
			test_params.update(n_samples=cfg.n_samples, careful_inc=cfg.careful_inc,
											stop_after=cfg.stop_after)
		
		elif cfg.test_type.lower() =='force-limit_test':
			test_params.update(n_samples=cfg.n_samples, careful_inc=cfg.careful_inc,
											stop_after=cfg.stop_after,
											force_threshold=cfg.force_threshold)
		elif cfg.test_type.lower() =='displacement-limit_test':
			test_params.update(n_samples=cfg.n_samples, careful_inc=cfg.careful_inc,
											stop_after=cfg.stop_after,
											displacement_threshold=cfg.displacement_threshold)
		elif cfg.test_type.lower() =='custom':
			test_params.update(n_samples=cfg.n_samples, careful_inc=cfg.careful_inc,
											max_force=cfg.max_force,
											max_displacement=cfg.max_displacement)
		if cfg.outfile:
			cfg.outfile = Path(cfg.outfile)
			fn = results_to_json(test_params, data, cfg.outfile)
			print(f'Saved data to {fn}')
		
	print('Done!')
	meter.close()
	msg_queue.put("done")






