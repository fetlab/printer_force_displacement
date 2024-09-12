import pandas as pd
import numpy as np
import json, csv
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import seaborn as sns
from pathlib import Path
import mplcursors
from ender_fdm import force_gauge, UP, DOWN, STILL
from typing import Iterable

ZERO_THRESH = 0.003

# MODEL_INFO_PATH = '../../../../Dropbox/Apps/Overleaf/A Tunable Bistable Mechanism/Table_model_names.tex'
MODEL_INFO_PATH = '../../../../Dropbox/Apps/Overleaf/A Tunable Bistable Mechanism/Table_model_names_version 2.tex'

FLIPPER_LEN = 16.7
SHUTTLE_THICKNESS = 6

# Special cases for manual data munging
def drop_stuff(data):
	#Drop BR test 2/down
	data['BR'] = data['BR'].loc[~(
		(data['BR'].direction == "DOWN") &
		(data['BR'].testno	  == 2))]

	#Drop BR due to incorrect model
	del data['BR']

	return data


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



def old_proc_df(df:pd.DataFrame, test_params:pd.Series):
	"""Process a test result dataframe for a single model. Pass results and test
	parameters."""
	df = df.copy()

	#Guess displacment based on params
	if test_params['test_type'] == 'smooth':
		df.displacement = np.linspace(0, test_params.smooth_displacement, len(df))

	direc = df.iloc[0].direction

	#Drop rows from the end which indicate snapthrough;
	#  for DOWN, that's force >= 0,
	#  for UP, that's force <= 0
	#To get from the end, we reverse the df, then use force.values.argmax to find the
	#  index of the first row where force is > or < 0.
	df = df[::-1]
	df = df.iloc[df.force.ge(0).values.argmax() if direc == 'DOWN' else df.force.le(0).values.argmin():]
	df = df[::-1]

	dft0 = df.copy()

	#Different method: look at all the rows after the minimum/maximum value, then drop
	# everything after a zero crossing
	tail = df.loc[df.force.argmin() if direc == 'DOWN' else df.force.argmax():]
	idx = tail.force.le(0).diff().argmax()
	to_drop = tail.iloc[idx:].index
	df = df.drop(tail.iloc[idx:].index)
	if len(df) == 0:
		raise ValueError(test_params)

	#Invert displacement so it's positive
	df.loc[:,'displacement'] *= -1

	#Drop leading rows with 0 force, except for the last one
	# fi = (df.force.abs().values > 0).argmax()
	# if fi > 1: df = df.iloc[fi-1:]

	dft1 = df.copy()

	if drops := DROP_TO_END.get(test_params.name):
		for d in drops:
			if df.iloc[0].testno == d['testno'] and df.iloc[0].direction == d['direction']:
				print(f'Drop {test_params.name}[{d["loc"]}:]')
				df = df.drop(df.loc[d['loc']:].index)

	if drops := DROP_TO_START.get(test_params.name):
		for d in drops:
			if df.iloc[0].testno == d['testno'] and df.iloc[0].direction == d['direction']:
				print(f'Drop {test_params.name}[:{d["loc"]}]')
				df = df.drop(df.loc[:d['loc']].index)


	#Make sure displacement starts at 0
	df.loc[:, 'displacement'] -= df.displacement.min()

	#Make both force readings positive
	df.loc[df.direction == 'DOWN', 'force'] *= -1

	return df


def loadproc(file:Path) -> pd.DataFrame:
	"""Load and process a test result."""
	p, dfs = read_test_json(file)
	return pd.concat([proc_df(df, p) for df in dfs])


def multi_sub(col:pd.Series, *subs:tuple[str,str]):
	"""Given a Series (`col`), do multiple search/replaces."""
	for search, replace in subs:
		col = col.str.replace(search, replace, regex=True)
	return col



def modelinfo2df(modelinfo:Path) -> dict:
	"""Translate a latex table to model info"""
	if modelinfo.suffix == '.tex':
		from astropy.table import Table
		tab = Table.read(modelinfo).to_pandas()
		for col in tab.columns:
			tab[col] = multi_sub(tab[col],
								 (r'\\textbf{([^}]+)}', r'\1'),
								 (r'(?:\s*mm)', ''),
								 ('\?', ''),
								 ('%.*$', ''),
								 (r'\\*', ''),
			 )#.str.strip()
			try:
				tab[col] = pd.to_numeric(tab[col])
			except ValueError:
				pass
		tab['Symmetric'] = tab.Symmetric.map(lambda v: v == 'checkmark')
		tab['w_b_ratio'] = (2*FLIPPER_LEN + SHUTTLE_THICKNESS) / tab['Base Width']
		tab['displacement_calc'] = 2 * FLIPPER_LEN * np.sin(np.radians(tab.Angle))
		return tab.set_index('Names')


def load_raw_results(filenames:Iterable[Path]=[],
										 modelinfo:Path=Path(MODEL_INFO_PATH)) -> tuple[pd.DataFrame, dict[str,list[pd.DataFrame]]]:
	filenames = filenames or Path('results').glob('*.json')
	params, data = {}, {}
	for file in filenames:
		p, dfs = read_test_json(file)
		params[file.stem] = p
		data[file.stem] = dfs

	if modelinfo.exists():
		models = modelinfo2df(modelinfo)
		for name in params:
			mname = name.split('-')[0]
			params[name] = pd.concat([params[name], models.loc[mname]], keys=['test', 'model'])

	return pd.DataFrame(params).T, data


def load_results(filenames:Iterable[Path]=[],
								 modelinfo:Path=Path(MODEL_INFO_PATH),
								 drop_tests=[],) -> tuple[pd.DataFrame, dict[str,pd.DataFrame]]:
	"""Load results files, process, and return a DataFrame of test and model params (model names as columns) and a dict of {name: data}. Return only params for which there are test results."""
	params, data = {}, {}
	filenames = filenames or Path('results').glob('*.json')

	for file in filenames:
		p, dfs = read_test_json(file)
		params[file.stem] = p
		data[file.stem] = pd.concat([proc_df(standardize(df)) for df in dfs if len(df) > 10])

	if modelinfo.exists():
		models = modelinfo2df(modelinfo)
		for name in params:
			mname = name.split('-')[0]
			params[name] = pd.concat([params[name], models.loc[mname]], keys=['test', 'model'])

		print(f"Models defined but without tests: {', '.join(sorted(set(models.index) - set(params.keys())))}")

	return pd.DataFrame(params).T, data#, drop_stuff(data)


def param2modelstr(name:str, params:pd.DataFrame):
	m = params.loc[name].model
	return fr"{name}: {m['Arm Type'][0]}{m.Angle}°{m['Arm Eff Len']}$l\times${m['In-Plane Th']}$t\times${m['Out-P Th']}$d$"


def remove_duplicate_legends(ax):
	handles, labels = ax.get_legend_handles_labels()
	labels = {k:v for k,v in zip(labels, handles)}
	ax.legend(labels.values(), labels.keys(), loc='best')


def snsplot(data:dict[str, list[pd.DataFrame]], params:pd.DataFrame,
						model_lines:dict, directions=('UP','DOWN'), figargs={}, **kwargs):
	if 'ax' in kwargs:
		ax = kwargs.pop('ax')
		fig = ax.get_figure()
	else:
		fig, ax = plt.subplots(**figargs)
	for name in data:
		df = data[name]
		for direction in directions:
			sns.lineplot(data=df[df.direction == direction], x='displacement',
								y='force', ax=ax, label=param2modelstr(name, params), **model_lines[name], **kwargs)

	#Remove duplicate legend entries
	if kwargs.get('legend', True):
		remove_duplicate_legends(ax)
	return fig


def multiplot(data:dict[str, list[pd.DataFrame]], params:pd.DataFrame, model_lines:dict, directions=('UP', 'DOWN'), figargs={}, **kwargs):
	if 'ax' in kwargs:
		ax = kwargs.pop('ax')
		fig = ax.get_figure()
	else:
		fig, ax = plt.subplots(**figargs)
	for name, df in data.items():
		c = ax._get_lines.get_next_color()
		for direction in directions:
			d = df[df.direction == direction]
			for _, g in d.groupby('testno'):
				ax.plot(g.displacement, g.force, label=name, color=c, alpha=.5 if (len(directions) > 1 and direction == 'UP') else 1, **kwargs)

	#Remove duplicate legend entries
	remove_duplicate_legends(ax)
	return fig


def plots(data:dict[str, list[pd.DataFrame]], params:pd.DataFrame, model_lines:dict, ptype='sns', subset=[], skip=[], inctest=None, **kwargs):
	if subset:	data = {name: data[name] for name in data if name in subset}
	if skip :	data = {name: data[name] for name in data if name not in skip}
	if inctest: data = {name: df for name,df in data.items() if inctest(name, df)}

	#Sort by parameters
	sortkeys, ascending = zip(*(
		('Arm Type', False),
		('Symmetric', False),
		('Angle', True),
		('Arm Eff Len', True),
		('In-Plane Th', True),
		('Out-P Th', True),
	))
	data = {name: data[name] for name in params.model.sort_values(list(sortkeys), ascending=ascending).index if name in data}

	if ptype == 'sns':
		return snsplot(data, params, model_lines, **kwargs)
	if ptype == 'mpl':
		return multiplot(data, params, model_lines, **kwargs)
	raise ValueError(f"Unmatched plot type: {ptype=}")


def plot_tests(df, direction="DOWN"):
	name = df.attrs['name']
	fig, axs = plt.subplots(3, 2, figsize=(7, 10), sharex=True, sharey=True)
	dirs = {'DOWN':'↓', 'UP':'↑'}
	for i,ax in zip(df.testno.unique(), axs.flat):
		plotdata = df.loc[((df.direction == direction) & (df.testno == i))]
		if len(plotdata) > 0:
			plotdata.plot('displacement', 'force', ax=ax, label=f"{name}{i}{dirs[direction]}")




#Source: https://stackoverflow.com/a/67005578/49663
def symmetrical_colormap(cmap_settings, new_name = None ):
	"""This function take a colormap and create a new one, as the concatenation
	of itself by a symmetrical fold.

	Example:
	cmap_settings = ('Blues', None)  # provide int instead of None to "discretize/bin" the colormap
	mymap = symmetrical_colormap(cmap_settings=cmap_settings, new_name=None)
	"""
	# get the colormap
	cmap = plt.cm.get_cmap(*cmap_settings)
	if not new_name:
		new_name = "sym_"+cmap_settings[0]	# ex: 'sym_Blues'

	# this defined the roughness of the colormap, 128 fine
	n = 128

	# get the list of color from colormap
	colors_r = cmap(np.linspace(0, 1, n))	 # take the standard colormap # 'right-part'
	colors_l = colors_r[::-1]				 # take the first list of color and flip the order # "left-part"

	# combine them and build a new colormap
	colors = np.vstack((colors_l, colors_r))
	mymap = mcolors.LinearSegmentedColormap.from_list(new_name, colors)

	return mymap
