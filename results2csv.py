#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "clize",
#     "pandas",
# ]
# ///
import pandas as pd
import json
from pathlib import Path
from typing import Iterable

ZERO_THRESH = 0.003

def read_test_json(file:Path) -> tuple[pd.Series, list[pd.DataFrame]]:
	"""Read the JSON file for a single model's test results. Return test_params,
	[test_results,...] where each test_result is a group
	of a single test direction."""
	with open(file) as f:
		data = json.load(f)
	df = pd.DataFrame.from_records(data['test_results'])
	df.attrs = {'name': file.stem}
	grouper = 'direction' if len(df.direction.unique()) > 1 else 'testno'
	return pd.Series(data['test_params'], name=file.stem), [
			g for _,g in df.groupby((df[grouper] != df[grouper].shift()).cumsum())]


def standardize(df):
	"""Standardize raw test data. In raw data:

		"DOWN" test data:
			- displacement starts at 0 and becoems negative as the
				Z-axis moves down.
			- force is negative as the probe pushes down

		"UP" test data:
			- displacement starts at 0 and becomes positive as the Z-axis
				moves up
			- force is positive as the probe pulls up

	Standardizing does:

		"DOWN" test data:
			- displacement sign is flipped so it starts at 0 and
				becomes positive as the Z-axis moves down
			- force is positive as the probe pushes down

		"UP" test data:
			- displacement data is reversed so it starts at max_displacement
				and ends at 0
			- force is negative as the probe pulls up
	"""
	df = df.copy()

	#Flip direction and force
	direc = df.iloc[0].direction

	if direc == 'DOWN':
		df.loc[:, 'displacement'] *= -1
		df.loc[:, 'force'] *= -1
	elif direc == 'UP':
		df.loc[:, 'displacement'] = (df.displacement - df.displacement.max()).abs()
		df.loc[:, 'force'] *= -1
	else:
		raise ValueError(f"Unknown direction {direc}")
	return df


def proc_df(df):
	"""Process a test results dataframe which has been standardized by a call to
	`standardize()`."""
	direc = df.iloc[0].direction

	#Flip the sign of force temporarily so the same algorithm works for both up and down
	if direc == "UP": df.loc[:,'force'] *= -1

	#First check if any rows in the first 10 have exactly 0; if so, drop up until
	# that row.
	if (f10 := df.iloc[:10].force.eq(0)).any():
		#Drop up to the last 0 in the first 10 rows
		df = df.iloc[df.index.get_loc(f10.iloc[::-1].idxmax()):]

	#Otherwise, drop up to 10 leading rows with force <= 0, except for the last one,
	# by finding the first row with force > 0.
	else:
		# print(f'{df.attrs["name"]}: drop to first > 0')
		fi = df.iloc[:10].force.gt(0).argmax()
		if fi > 1: df = df.iloc[fi-1:]

	if df.force.min() <= ZERO_THRESH:
		#Drop everything after the first zero crossing that happens after the max
		# value of the first half of the data

		#Find the location of the max value in the first half of the data
		maxidx = df.force[:len(df.force)//2].argmax()

		#Find the first zero crossing after the max and drop data after it
		tail = df.iloc[maxidx:]
		below_zero = tail.force.le(ZERO_THRESH).diff()
		if below_zero.any():
			idx = below_zero.argmax()
			to_drop = tail.iloc[idx+1:].index
			df = df.drop(to_drop)

	#Shift displacements to ensure they start at 0
	df.loc[:,'displacement'] -= df.displacement.min()

	#Unflip the force
	if direc == "UP": df.loc[:,'force'] *= -1

	#Convert the force from Kgf to N
	df.loc[:,'force'] *= 9.806650

	return df


def load_results(filenames:Iterable[Path],
								 modelinfo:Path,
								 drop_tests=[],) -> tuple[pd.DataFrame, dict[str,pd.DataFrame]]:
	"""Load results files, process, and return a DataFrame of test and model params (model names as columns) and a dict of {name: data}. Return only params for which there are test results."""
	params, data = {}, {}
	filenames = filenames or Path('results').glob('*.json')

	for file in filenames:
		p, dfs = read_test_json(file)
		params[file.stem] = p
		data[file.stem] = pd.concat([proc_df(standardize(df)) for df in dfs if len(df) > 10])

	if isinstance(modelinfo, pd.DataFrame) or modelinfo.exists():
		models:pd.DataFrame = modelinfo2df(modelinfo) if isinstance(modelinfo, Path) else modelinfo
		for name in list(params.keys()):
			if name not in models.index:
				print(f'Test results for "{name}" but no model info, not loading')
				if name in data: del(data[name])
				del params[name]
				continue
			mname = name.split('-')[0]
			params[name] = pd.concat(
				[params[name], models.loc[mname]],
				axis=0,
				keys=['test', 'model']
			)

		print(f"Models defined but without tests: {', '.join(sorted(set(models.index) - set(params.keys())))}")

	rparams = pd.DataFrame(params).T
	return rparams, data#, drop_stuff(data)


def main(results_folder:Path, output_csv:Path) -> None:
	"""
	:param results_folder: The folder where your result files in json format are located
	:param output_csv:     The CSV file to write to
	"""
	out = []
	defined_models = pd.read_csv('defined_models.csv', index_col='Name')
	params, data = load_results(Path(results_folder).glob('*.json'), defined_models)
	for model, results in data.items():
		out.append(results.assign(model=model))
	pd.concat(out)[['model', 'testno', 'displacement', 'force', 'timestamp']].to_csv(output_csv)


if __name__ == "__main__":
	from clize import run
	run(main)
