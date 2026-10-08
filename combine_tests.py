#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "clize",
#     "pyserial",
#     "rich",
# ]
# ///
import json, clize
from pathlib import Path
from ender_fdm import results_to_json
from collections import defaultdict


def combine_all(files):
	stems = defaultdict(list)

	for fn in files:
		if '-' not in fn.stem:
			continue
		stems[fn.stem.split('-')[0]].append(fn)

	for stem, fns in stems.items():
		outfile = fns[0].parent / f'{stem}.json'
		combine(fns, outfile=outfile)


def combine(files, outfile):
	outdata = []
	params = {}
	testno = None
	testnums: set[int] = set()

	o = Path(outfile)
	if o.exists():
		print(f'Outfile {o} exists, pick a new one')

	for file in files:
		testno = None
		p = Path(file)

		if '-' in p.stem:
			name, numstr = p.stem.rsplit('-', maxsplit=1)
			try: testno = int(numstr)
			except ValueError: pass

		with open(p) as f:
			data = json.load(f)

		if params and data['test_params'] != params:
			raise ValueError(f'test_params in {p} not the same as previously read params:\n'
										f'{set(params.items()) ^ set(data["test_params"].items())}')
		params = data['test_params']

		if (testno := data['test_results'][0]['testno']) < 0:
			try:
				testno = max(testnums) + 1
			except ValueError:
				testno = 0

		if testno is None or testno in testnums:
			raise ValueError(f'{testno=}, {testnums=}')
		testnums.add(testno)

		for rec in data['test_results']:
			rec['testno'] = testno

		outdata.extend(data['test_results'])

	results_to_json(params, outdata, o)


def main(*files, outfile:'o'=''):
	"""Combine test run files.

	:param files: The files to combine into outfile, or a single folder in which
								to combine files by prefix.
	:param outfile: The output file. If omitted will be automatically determined.
	"""
	files = list(map(Path, files))
	if len(files) > 1:
		if any(f.is_dir() for f in files):
			raise ValueError("Specify exactly one directory")
		if outfile:
			combine(files, outfile)
		else:
			combine_all(files)
	else:
		if not files[0].is_dir():
			raise ValueError("Only name given which is not a directory")
		combine_all(files[0].glob('*.json'))

	

if __name__ == "__main__":
	clize.run(main)
