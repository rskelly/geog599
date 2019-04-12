#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
import numpy as np
import sys
import csv
import os
import math

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

def hull(pts, dist, clip):
	'''
	Constructs a convex hull with the exception that the dist
	parameter limits the length of a segment in the y dimension.
	'''
	hull = list(pts[:2])

	# If there's an alpha do a degenerate hull. It could have been one loop but we save a bit of
	# time and visual complexity this way.
	for i in range(2, len(pts)):
		while len(hull) >= 2 and cross(hull[-2], hull[-1], pts[i]) >= 0 and lengthY(hull[-2], pts[i]) <= dist ** 2:
			hull.pop()
		hull.append(pts[i])

	hull = hull[clip:-(clip + 1)]
	beg = []
	end = []
	c = 3
	for i in range(c):
		beg.append((hull[0][0], hull[0][1] - dist * (c - i), hull[0][2]))
		end.append((hull[-1][0], hull[-1][1] + dist * (i + 1), hull[-1][2]))
	hull = np.array(beg + hull + end)
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
	return UnivariateSpline(x, y, w = weight, k = 3, s = smooth)



def plot_profile(outfile, smooth, weight, alpha, coords, hull, x, alt, vel, acc):

	fig = plt.figure(figsize=(12, 4))
	ax1 = fig.add_subplot(111)

	l_pts, = ax1.plot(coords[...,1], coords[...,2], 'o', color='#cccccc', ms=1)
	l_hull, = ax1.plot(hull[...,1], hull[...,2], 'ro', ms=1)
	l_spl, = ax1.plot(x, alt, 'g', lw=1)

	ax1.set_ylabel('Elevation (m)')
	ax1.set_xlabel('Distance (m)')
	ax1.set_title("Smoothing Cubic Spline (smooth={s:.3f}, weight={w}, alpha={a})".format(s = smooth, w = weight, a = alpha))
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

	plt.legend([l_spl, l_vel, l_acc], ['Altitude (m)', 'Velocity (m/s)', 'Acceleration (m/s²)'])
	plt.savefig(outfile, bbox_inches='tight', format='png', dpi=300)
	#plt.show()
	plt.close()

	return (weight, smooth, outfile)


def p3angle(p1, p2, p3):
	'''
	Return the angle between 3 points. p2 is the middle point. Angle is 
	clampted to 0. <= a <= pi.
	'''
	_, x1, y1 = p1
	_, x2, y2 = p2
	_, x3, y3 = p3
	a = math.atan2(y1 - y2, x1 - x2)
	b = math.atan2(y3 - y2, x3 - x2)
	if a < 0.:
		a += math.pi * 2.
	if b < 0.:
		b += math.pi * 2.
	a = b - a if b > a else a - b
	return a

def weights(pts, exp = 0):
	'''
	Compute weights for the pointset based on the angle between
	triples of points. An acute angle has a high weight. A flat gets a low weight.
	'''
	# Get the std deviation of the points (assumes 3d).
	std = np.std(pts[...,1])
	
	w = [1. / std]
	for i in range(1, len(pts) - 1):
		a = p3angle(pts[i - 1], pts[i], pts[i + 1])
		w.append(math.pow(a / math.pi, exp) * std)
	w.append(1. / std)
	w = np.array(w)
	return w

		
def run(filename, outdir):
	'''
	Run the profile with angle-based weights.
	'''

	smooth_mult = [.25, .5, 1., 2., 4.]
	weight_exp = [0., 1., 2., 4.]
	alpha = [2., 5., 10., 25., 50.]
	hull_clip = 5
	x_clip = 5
	
	# Try to create the output directory.
	try:
		os.makedirs(outdir)
	except: pass

	# Remove files from output.
	for f in [x for x in os.listdir(outdir) if x.endswith('.png')]:
		os.unlink(os.path.join(outdir, f))

	# Template for plot output file.
	ptpl = os.path.splitext(os.path.basename(filename))[0] + '_{s}_{a}_{w}.png'

	# Load coordinates from point file.
	coords = load_points(filename)
	
	# Open a stats file. This will contain stats for all runs.
	with open(os.path.join(outdir, 'stats.csv'), 'w') as f:
		f.write('filename,alpha,smooth,weight,coord_n,hull_n,above_n,above_max,vel_max,vel_min,acc_max,acc_min,residual\n')
		
		for a in alpha:
			
			chull = hull(coords, a, hull_clip)
			minx = chull[0,1]
			maxx = chull[-1,1]
			x = np.linspace(minx, maxx, 1000)[x_clip:-(x_clip + 1)]	# Regular x-coords (actually, y)
			aa = str(a).replace('.', '_')
	
			for w in weight_exp:
				
				wts = weights(chull, w)
				ww = str(w).replace('.', '_')
	
				for s in smooth_mult:
	
					s = math.sqrt(2 * len(chull)) * s
					ss = '{s:.3f}'.format(s = s).replace('.', '_')
	
					spline = trajectory(chull, s, wts)
					
					alt = spline(x, 0)								# Altitude
					vel = spline(x, 1)								# Velocity
					acc = spline(x, 2)								# Acceleration
	
					plotfile = os.path.join(outdir, ptpl.format(s = ss, a = aa, w = ww))
	
					plot_profile(plotfile, s, w, a, coords, chull, x, alt, vel, acc) # Clip the ends -- they tend to have outliers.

					# Look at points above the spline.
					acoords = coords[coords[...,1] >= minx]
					acoords = acoords[acoords[...,1] <= maxx]
					above = spline(acoords[...,1], ext = 2)
	
					above = acoords[above < acoords[...,2]]
					above_max = np.max(above)
					above_n = len(above)
					
					vel = spline(acoords[...,1], nu = 1, ext = 2)
					vel_max = np.max(vel)
					vel_min = np.min(vel)
					
					acc = spline(acoords[...,1], nu = 2, ext = 2)
					acc_max = np.max(acc)
					acc_min = np.min(acc)
					
					residual = spline.get_residual()
					
					coord_n = len(coords)
					hull_n = len(chull)
					
					f.write(','.join(list(map(str, [filename, a, s, w, coord_n, hull_n, above_n, above_max, vel_max, vel_min, acc_max, acc_min, residual]))) + '\n')


def run_profiles():
	with open('profiles.csv', 'r') as f:
		db = csv.reader(f)
		next(db)
		for line in db:
			if line[0] == '1':
				print(line[7])
				run(line[7], 'plots')
				break


if __name__ == '__main__':

	run_profiles()

	# p1 = [0, 0, 0]
	# p2 = [0, 1, 0]
	# p3 = [0, 1, 1]
	# a = p3angle(p1, p2, p3)
	# print(a)

	# p1 = [0, 0, 0]
	# p2 = [0, 1, 0]
	# p3 = [0, 2, 0]
	# a = p3angle(p1, p2, p3)
	# print(a)	

	# p1 = [0, 1, 0]
	# p2 = [0, 0, 0]
	# p3 = [0, 0, 1]
	# a = p3angle(p1, p2, p3)
	# print(a)	