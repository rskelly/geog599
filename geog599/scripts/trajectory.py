#!/usr/bin/env python3

from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
import numpy as np
import sys
import csv
import os

''' 
Extracts a 'surface' from a point cloud using a modified convex hull algorithm.
Then uses the UnivariateSpline function to estimate a trajectory by fitting
a smooth spline to the surface points.

1) Must check the maximum vertical distance of an outlier from the trajectory.
2) Must analyze the RMS difference between this and a terrain representation.
3) Must constain according to the maximum value of the second derivative, which corresponds to thrust.
'''

def cross(p1, p2, p):
	'''
	Determine what side of the line joining p0 and p1, p is on.
	'''
	return (p1[1] - p[1]) * (p2[2] - p[2]) - (p1[2] - p[2]) * (p2[1] - p[1])

def lengthY(p1, p2):
	'''
	Calculate the squared distance between two points in the y.
	'''	
	return (p1[1] - p2[1]) ** 2.

def hull(pts, dist):
	'''
	Constructs a convex hull with the exception that the dist
	parameter limits the length of a segment in the y dimension.
	'''
	hull = list(pts[:2])

	# If there's an alpha do a degenerate hull. It could have been one loop but we save a bit of
	# time and visual complexity this way.
	for i in range(2, len(pts)):
		while len(hull) >= 2 and cross(hull[-2], hull[-1], pts[i]) >= 0 and lengthY(hull[-2], pts[i]) <= (dist * dist):
			hull.pop()
		hull.append(pts[i])

	hull = np.array(hull)
	#hull.sort(order = 'y')
	return hull

def load_points(filename):
	'''
	Loads a 2d point cloud ndarray.
	'''
	with open(filename, 'r') as f:
		lines = list(map(float, [item for row in f.read().strip().split('\n')[1:] for item in list(map(str.strip, row.split(',')))]))
	coords = np.array(lines).reshape(-1,3)
	coords = coords[coords[:,1].argsort()] # Sort on second column.
	return coords

def trajectory(coords, smooth, weight):
	'''
	Develop a smooth spline on the given coordinates with smooth and weight.
	Return the spline object (Univariate spline).
	'''
	x = coords[:,1]
	y = coords[:,2]
	return UnivariateSpline(x, y, k = 3, s = smooth)



def plot(outfile, smooth, weight, coords, hull, x, alt, vel, acc):

	fig = plt.figure(figsize=(12, 4))
	ax1 = fig.add_subplot(111)

	l_pts, = ax1.plot(coords[...,1], coords[...,2], 'o', color='#cccccc', ms=1)
	l_hull, = ax1.plot(hull[...,1], hull[...,2], 'ro', ms=1)
	l_spl, = ax1.plot(x, alt, 'g', lw=1)

	ax1.set_ylabel('Elevation (m)')
	ax1.set_xlabel('Distance (m)')
	ax1.set_title("Smoothing Cubic Spline (p = {s}, w = {w})".format(s = smooth, w = weight))
	for tl in ax1.get_yticklabels():
	    tl.set_color('g')

	ax2 = ax1.twinx()
	ax2.axhline(0, color='#cccccc')
	l_vel, = ax2.plot(x, vel, 'm', lw=1)
	for tl in ax2.get_yticklabels():
	    tl.set_color('m')
	ymax = max(map(abs, ax2.get_ylim()))
	ax2.set_ylim((-ymax, ymax))

	ax3 = ax1.twinx()
	l_acc, = ax3.plot(x, acc, 'b', lw=1)
	ax3.set_ylabel('Velocity (m/s); Acceleration (m/s²)')
	for tl in ax3.get_yticklabels():
	    tl.set_color('b')
	ymax = max(map(abs, ax3.get_ylim()))
	ax3.set_ylim((-ymax, ymax))

	plt.legend([l_spl, l_vel, l_acc], ['Altitude (m)', 'Vertical Velocity (m/s)', 'Vertical Acceleration (m/s²)'])
	plt.savefig(outfile, bbox_inches='tight', format='png', dpi=300)
	#plt.show()
	plt.close()

	return (weight, smooth, outfile)



smooths = [0, 1., 2., 3., 4., 5.]
weights = [1.]

def run(filename, outdir):
	
	try:
		os.makedirs(outdir)
	except: pass

	tpl = os.path.splitext(os.path.basename(filename))[0] + '_{s}_{w}.png'
	clip = 3

	for w in weights:
		for s in smooths:
			ww = str(w).replace('.', '_')
			ss = str(s).replace('.', '_')
			outfile = os.path.join(outdir, tpl.format(s = ss, w = ww))
			coords = load_points(filename)
			chull = hull(coords, 10.)
			spline = trajectory(chull, s, 0.)
			x = np.linspace(chull[0,1], chull[-1,1], 1000)
			alt = spline(x, 0)
			vel = spline(x, 1)
			acc = spline(x, 2)
			plot(outfile, s, w, coords, chull, x[clip:-clip], alt[clip:-clip], vel[clip:-clip], acc[clip:-clip]) # Clip the ends -- they tend to have outliers.

with open('profiles.csv', 'r') as f:
	db = csv.reader(f)
	head = next(db)
	for line in db:
		if line[0] == '1':
			run(line[7], 'plots')

