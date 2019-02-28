#!/usr/bin/env python3

import os
import sys
import matplotlib.pyplot as plt
import numpy as np


def plot(y, z, i):
	fig = plt.figure(figsize=(8, 3.5))
	ax1 = fig.add_subplot(111)

	l_surf, = ax1.plot(y, z, 'ro', ms=1)

	#ax1.set_ylabel('Elevation (m)')
	#ax1.set_xlabel('Distance (m)')
	#ax1.set_title("Smoothing Cubic Spline (p = {s}, w = {w})".format(s = smooth, w = weight))
	#for tl in ax1.get_yticklabels():
	#    tl.set_color('g')

	#ax2 = ax1.twinx()
	#ax2.axhline(0, color='#cccccc')
	#l_vel, = ax2.plot(x, y1, 'm', lw=1)
	#for tl in ax2.get_yticklabels():
	#    tl.set_color('m')
	#ymax = max(map(abs, ax2.get_ylim()))
	#ax2.set_ylim((-ymax, ymax))

	#plt.legend([l_hull, l_spl, l_vel, l_acc], ['Concave Hull Vertices (⍺ = 10m)', 'Altitude (m)', 'Vertical Velocity (m/s)', 'Vertical Acceleration (m/s²)'])
	plt.savefig('files/plot_{i}.png'.format(i = i), bbox_inches='tight', format='png', dpi=96)
	#plt.show()
	plt.close()

i = 1
with open(sys.argv[1], 'r') as f:
	while True:
		y = list(map(float, f.readline().strip()[:-2].split(',')))
		z = list(map(float, f.readline().strip()[:-2].split(',')))
		print(y)
		print(z)
		plot(y, z, i)
		i += 1
		break

