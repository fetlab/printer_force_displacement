import json, clize
from pathlib import Path
from ender_fdm import results_to_json

def main(*files, outfile: 'o'):
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
			raise ValueError(f'test_params in {p} not the same as previously read params!')
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

if __name__ == "__main__":
	clize.run(main)
