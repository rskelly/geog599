#!/usr/bin/env python3

import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np
import math 
import os
import re
from operator import itemgetter


data = []

dir = '/home/rob/Documents/git/msc/experiments/controller/build/'
for f in [x for x in os.listdir(dir) if x.endswith('.csv')]:
	data.append(os.path.join(dir, f))

def _sort(a, b):
	if a[0] == b[0]:
		return a[1] < b[1]
	else:
		return a[0] < b[0]

def plot(file, skip = False):
	x = []
	y = []
	y0 = []
	y1 = []
	y2 = []

	m = re.match(r"(.*?)_(.*?)_(.*?).csv", file)
	smooth = m.group(2)
	weight = m.group(3)
	print(smooth, weight)

	ss = str(smooth).replace('.', '_')
	ww = str(weight).replace('.', '_')

	outfile = 'figures/graph_{s}_weight_{w}.png'.format(s = ss, w = ww)

	if not skip:

		with open(file, 'rU') as f:
			line = f.readline()
			while line:
				try:
					_x, _y, _y0, _y1, _y2 = list(map(float, line.strip().split(',')))
					x.append(_x)
					y.append(_y)
					y0.append(_y0)
					y1.append(_y1)
					y2.append(_y2)
				except: pass
				line = f.readline()

		x = x[2:-2]
		y = y[2:-2]
		y0 = y0[2:-2]
		y1 = y1[2:-2]
		y2 = y2[2:-2]

		fig = plt.figure(figsize=(8, 3.5))
		ax1 = fig.add_subplot(111)

		l_hull, = ax1.plot(x, y, 'ro', ms=1)
		l_spl, = ax1.plot(x, y0, 'g', lw=1)

		ax1.set_ylabel('Elevation (m)')
		ax1.set_xlabel('Distance (m)')
		ax1.set_title("Smoothing Cubic Spline (p = {s}, w = {w})".format(s = smooth, w = weight))
		for tl in ax1.get_yticklabels():
		    tl.set_color('g')

		ax2 = ax1.twinx()
		ax2.axhline(0, color='#cccccc')
		l_vel, = ax2.plot(x, y1, 'm', lw=1)
		for tl in ax2.get_yticklabels():
		    tl.set_color('m')
		ymax = max(map(abs, ax2.get_ylim()))
		ax2.set_ylim((-ymax, ymax))

		ax3 = ax1.twinx()
		l_acc, = ax3.plot(x, y2, 'b', lw=1)
		ax3.set_ylabel('Velocity (m/s); Acceleration (m/s²)')
		for tl in ax3.get_yticklabels():
		    tl.set_color('b')
		ymax = max(map(abs, ax3.get_ylim()))
		ax3.set_ylim((-ymax, ymax))

		plt.legend([l_hull, l_spl, l_vel, l_acc], ['Concave Hull Vertices (⍺ = 10m)', 'Altitude (m)', 'Vertical Velocity (m/s)', 'Vertical Acceleration (m/s²)'])
		plt.savefig(outfile, bbox_inches='tight', format='png', dpi=96)
		#plt.show()
		plt.close()

	return (weight, smooth, outfile)

try:
	os.makedirs('figures')
except Exception as e: 
	pass

outfiles = []

for file in data:
	outfiles.append(plot(file, False))

outfiles.sort(key = itemgetter(0, 1))

with open('index.html', 'w') as o:

	o.write('<html><body><table>')

	i = 0
	for smooth, weight, file in outfiles:
		if i % 2 == 0:
			o.write('<tr>')
		o.write('<td><img src="{f}" /></td>'.format(f = file))
		if i % 2 != 0:
			o.write('</tr>')
		i += 1

	o.write('</table></body></html>')
