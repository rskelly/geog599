#!/usr/bin/env python3

import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np
import math 
import os
import re
from operator import itemgetter
import sys

traj = []
pos = []

dir = '/home/rob/Documents/git/msc/experiments/controller/build/'

for f in [x for x in os.listdir(dir) if x.startswith('traj_')]:
	traj.append(os.path.join(dir, f))

for f in [x for x in os.listdir(dir) if x.startswith('pos_')]:
	pos.append(os.path.join(dir, f))

def _sort(a, b):
	if a[0] == b[0]:
		return a[1] < b[1]
	else:
		return a[0] < b[0]

def plot(traj, pos, skip = False):

	m = re.match(r"(.*?)_(.*?)_(.*?).csv", traj)
	smooth = m.group(2)
	weight = m.group(3)
	print(smooth, weight)

	ss = str(smooth).replace('.', '_')
	ww = str(weight).replace('.', '_')

	output = []

	if not skip:

		i = 0
		j = 0
		data = {}
		with open(traj, 'rU') as f:
			line = f.readline()
			while line:
				try:
					line = line.strip().split(',')
					data[line[0]] = list(map(float, line[1:-1]))
					i += 1
					if i > 0 and i % 4 == 0:
						outfile = 'output/graph_{j}_{s}_weight_{w}.png'.format(s = ss, w = ww, j = j)
						output.append(plot2(outfile, smooth, weight, **data))
						j += 1
						sys.exit(1)
				except Exception as e:
					print(e)
				line = f.readline()

	return output


def plot2(outfile, smooth, weight, y, alt, vel, acc):

	print(list(zip(y, alt)))
	print(outfile, smooth, weight)

	fig = plt.figure(figsize=(8, 3.5))
	ax1 = fig.add_subplot(111)

	#l_hull, = ax1.plot(x, y, 'ro', ms=1)
	l_spl, = ax1.plot(y, alt, 'g', lw=1)

	ax1.set_ylabel('Elevation (m)')
	ax1.set_xlabel('Distance (m)')
	ax1.set_title("Smoothing Cubic Spline (p = {s}, w = {w})".format(s = smooth, w = weight))
	for tl in ax1.get_yticklabels():
	    tl.set_color('g')

	ax2 = ax1.twinx()
	ax2.axhline(0, color='#cccccc')
	l_vel, = ax2.plot(y, vel, 'm', lw=1)
	for tl in ax2.get_yticklabels():
	    tl.set_color('m')
	ymax = max(map(abs, ax2.get_ylim()))
	ax2.set_ylim((-ymax, ymax))

	ax3 = ax1.twinx()
	l_acc, = ax3.plot(y, acc, 'b', lw=1)
	ax3.set_ylabel('Velocity (m/s); Acceleration (m/s²)')
	for tl in ax3.get_yticklabels():
	    tl.set_color('b')
	ymax = max(map(abs, ax3.get_ylim()))
	ax3.set_ylim((-ymax, ymax))

	plt.legend([l_spl, l_vel, l_acc], ['Altitude (m)', 'Vertical Velocity (m/s)', 'Vertical Acceleration (m/s²)'])
	plt.savefig(outfile, bbox_inches='tight', format='png', dpi=96)
	#plt.show()
	plt.close()

	return (weight, smooth, outfile)

try:
	os.makedirs('output')
except Exception as e: 
	pass

for i in range(len(traj)):
	outfiles = plot(traj[i], pos[i], False)

# outfiles.sort(key = itemgetter(0, 1))

# with open('index.html', 'w') as o:

# 	o.write('<html><body><table>')

# 	i = 0
# 	for smooth, weight, file in outfiles:
# 		if i % 2 == 0:
# 			o.write('<tr>')
# 		o.write('<td><img src="{f}" /></td>'.format(f = file))
# 		if i % 2 != 0:
# 			o.write('</tr>')
# 		i += 1

# 	o.write('</table></body></html>')
