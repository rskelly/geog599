#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

from scipy.interpolate import UnivariateSpline
from scipy.optimize import fmin
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
	spline = UnivariateSpline(x, y, w = weight, k = 3, s = smooth)
	return spline



def plot_profile(outfile, smooth, weight, alpha, coords, hull, x, alt, vel, acc, wts):

	fig = plt.figure(figsize=(12, 4))
	ax1 = fig.add_subplot(111)

	l_pts = ax1.scatter(coords[...,1], coords[...,2], marker='.', s=1, color='#cccccc', zorder=1)
	l_hull = ax1.scatter(hull[...,1], hull[...,2], marker='o', s=1, c=wts, zorder=1000)
	l_spl, = ax1.plot(x, alt, 'g', lw=1, zorder=100)

	ax1.set_ylabel('Elevation (m)')
	ax1.set_xlabel('Distance (m)')
	ax1.set_title("Smoothing Cubic Spline (smooth={s:.6f}, weight={w}, alpha={a})".format(s = smooth, w = weight, a = alpha))
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
	ax3.tick_params(pad=30)

	plt.legend([l_spl, l_vel, l_acc, l_hull], ['Altitude (m)', 'Velocity (m/s)', 'Acceleration ($m/s^2$)', 'Concave Hull Vertex'])
	plt.savefig(outfile, bbox_inches='tight', format='png', dpi=300)
	#plt.show()
	plt.close()

	return (weight, smooth, outfile)

def plot_slope_weights(outfile):

	fig = plt.figure(figsize=(6, 4))
	ax1 = fig.add_subplot(111)

	a = np.linspace(0, math.pi / 2., 100)
	x = a / (math.pi / 2.)
	y025 = 1. -x ** .25
	y05 = 1. - x ** .5
	y1 = 1. - x
	y2 = 1. - x ** 2.
	y4 = 1. - x ** 4.
	
	l_y025, = ax1.plot(a, y025, '-c', lw=1)
	l_y05, = ax1.plot(a, y05, '-y', lw=1)
	l_y1, = ax1.plot(a, y1, '-r', lw=1)
	l_y2, = ax1.plot(a, y2, '-g', lw=1)
	l_y4, = ax1.plot(a, y4, '-b', lw=1)

	ax1.set_ylabel('Weight')
	ax1.set_xlabel('Angle (r)')
	ax1.set_title('Slope-Derived Weights')

	plt.legend([l_y025, l_y05, l_y1, l_y2, l_y4], ['$y^{0.25}$', '$y^{0.5}$', '$y^1$', '$y^2$', '$y^4$'])
	plt.savefig(outfile, bbox_inches='tight', format='png', dpi=300)
	#plt.show()
	plt.close()


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
	if a > math.pi:
		a -= math.pi
	return a

def angle_weights(pts, exp = 0):
	'''
	Compute weights for the pointset based on the angle between
	triples of points. An acute angle has a high weight. A flat gets a low weight.
	'''
	# Get the std deviation of the points (assumes 3d).
	std = np.std(pts[...,1])
	
	w = [std]
	for i in range(1, len(pts) - 1):
		a = p3angle(pts[i - 1], pts[i], pts[i + 1])
		p = 1. - (a / math.pi)
		if p == 0.:
			p += 0.000000001
		w.append(math.pow(p, exp) * std)
	w.append(std)
	w = np.array(w)
	return w

def slope(p1, p2):
	_, x1, y1 = p1
	_, x2, y2 = p2
	a = abs(math.atan2(y2 - y1, x2 - x1))
	if a > math.pi / 4.:
		a = abs(a - math.pi / 2.)
	return a
	

def slope_weights(pts, exp = 0):
	'''
	Compute weights for the pointset based on the angle between
	triples of points. An acute angle has a high weight. A flat gets a low weight.
	'''
	# Get the std deviation of the points (assumes 3d).
	std = np.std(pts[...,1])
	
	w = [1. / std]
	for i in range(1, len(pts) - 1):
		s1 = slope(pts[i - 1], pts[i])
		s2 = slope(pts[i], pts[i + 1])
		q = math.pow(1. - max(s1, s2) / (math.pi / 4.), exp)
		w.append(q / std)
	w.append(1. / std)
	w = np.array(w)
	return w

def minimize_residual(filename):
	'''
	'''

	# Load coordinates from point file.
	coords = load_points(filename)
	
	hull_clip = 5
	x_clip = 5
	
	def fn(x, *args):
		s, a = x
		if s <= 0. or a <= 0.:
			return math.inf
		chull = hull(args[0], a, hull_clip)
		weights = [1.] * len(chull)
		
		spline = trajectory(chull, s, weights)
		
		weights = weights[x_clip:-(x_clip + 1)]
		x = chull[...,1][x_clip:-(x_clip + 1)]
		y = chull[...,2][x_clip:-(x_clip + 1)]
		
		y0 = spline(x)
		y1 = spline(x, nu=1)
		y2 = spline(x, nu=2)
		resid = np.sum((weights * (y - y0)) ** 2)
		
		# Look at points above the spline.
		acoords = coords[coords[...,1] >= x[0]]
		acoords = acoords[acoords[...,1] <= x[-1]]
		az1 = spline(acoords[...,1], ext = 2)
		az2 = acoords[...,2]
		aboveidx = az1 < az2
		az1 = az1[aboveidx]
		az2 = az2[aboveidx]
		above = az2 - az1
		above_max = np.max(above)
		
		d = np.linalg.norm((np.max(y1), np.max(y2), resid, above_max))
		print(s, a, d, np.max(y1), np.max(y2), resid, above_max)
		return d
					
	fmin(fn, (1., 10.), args=(coords,))

def run(filename, outdir, stats, doplot = False, params = None):
	'''
	Run the profile with angle-based weights.
	'''
	if params:
		smooth = [params['smooth_mult']]
		weight_exp = [params['weight_exp']]
		alpha = [params['alpha']]
	else:
		smooth = None
		smooth_mult = np.geomspace(.000001, 100, 20)
		weight_exp = [0., 1., 2., 4.]
		alpha = [1., 2., 5., 10., 25., 50.]

	hull_clip = 5
	x_clip = 5
	
	# Template for plot output file.
	ptpl = os.path.splitext(os.path.basename(filename))[0] + '_{s}_{a}_{w}.png'

	# Load coordinates from point file.
	coords = load_points(filename)
	
	for a in alpha:
		
		try:
			chull = hull(coords, a, hull_clip)
			x = np.linspace(chull[0,1], chull[-1,1], 1000)[x_clip:-(x_clip + 1)]	# Regular x-coords (actually, y)
			minx = x[0]
			maxx = x[-1]
			aa = '{a:.2f}'.format(a = a).replace('.', '_')
		except Exception as e:
			print(e)
			stats.write('{f},{e}\n'.format(f = filename, e = e.__str__()))
			continue
			
		for w in weight_exp:

			try:			
				wts = slope_weights(chull, w)
				ww = '{w:.3f}'.format(w = w).replace('.', '_')
				m = len(chull)
			except Exception as e:
				print(e)
				stats.write('{f},{e}\n'.format(f = filename, e = e.__str__()))
				continue

			# Either use the give smooth parameter(s) or compute using the suggested range and multiplers.
			smooth = smooth if smooth else [x * y for y in smooth_mult for x in [m - math.sqrt(2 * m), m, m + math.sqrt(2 * m)]]

			for s in smooth:
				
				try:
					ss = '{s:.6f}'.format(s = s).replace('.', '_')
	
					spline = trajectory(chull, s, wts)
					
					alt = spline(x, 0)								# Altitude
					vel = spline(x, 1)								# Velocity
					acc = spline(x, 2)								# Acceleration
	
					if doplot:
						plotfile = os.path.join(outdir, ptpl.format(s = ss, a = aa, w = ww))
		
						plot_profile(plotfile, s, w, a, coords, chull, x, alt, vel, acc, wts) # Clip the ends -- they tend to have outliers.

					# Look at points above the spline.
					acoords = coords[coords[...,1] >= minx]
					acoords = acoords[acoords[...,1] <= maxx]
					acoords = acoords[x_clip:-(x_clip + 1)]
					az1 = spline(acoords[...,1], ext = 2)
					az2 = acoords[...,2]
					
					aboveidx = az1 < az2
					az1 = az1[aboveidx]
					az2 = az2[aboveidx]
					above = az2 - az1
					try:
						above_max = np.max(above)
					except Exception as e:
						print(e)
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
					
					if stats:
						stats.write(','.join(list(map(str, [filename, '', a, s, w, coord_n, hull_n, above_n, above_max, vel_max, vel_min, acc_max, acc_min, residual]))) + '\n')

				except Exception as e:
					print(e)
					stats.write('{f},{e}\n'.format(f = filename, e = e.__str__()))


def density(filename):

	miny = 99999.
	maxy = -99999.
	minz = 99999.
	maxz = -99999.
	count = 0
	with open(filename, 'r') as f:
		f.readline()
		for line in f.read().split():
			try:
				x, y, z = list(map(float, line.split(',')))
				if y < miny:
					miny = y
				if y > maxy:
					maxy = y
				if z < minz:
					minz = z
				if z > maxz:
					maxz = z
				count += 1
			except Exception as e:
				continue

	print(filename, count / (maxy - miny))

def run_profiles(outdir):
	
	params = {
		'/home/rob/Documents/msc/data/lidar/2m_swath/mt_doug_1_2m.txt': {
			'smooth_mult': 0.003220259068882,
			'weight_exp': 0.,
			'alpha': 10.
		},
		'/home/rob/Documents/msc/data/lidar/2m_swath/mt_doug_2_2m.txt': {
			'smooth_mult':  0.011467576706331,
			'weight_exp': 0.,
			'alpha': 10.
		},
		'/home/rob/Documents/msc/data/lidar/2m_swath/nrcan_4_2m.txt': {
			'smooth_mult': 0.002412535572338,
			'weight_exp': 0.,
			'alpha': 1.
		},
		'/home/rob/Documents/msc/data/lidar/2m_swath/swan_lk_1_2m.txt': {
			'smooth_mult': 0.002315,
			'weight_exp': 0.,
			'alpha': 10.,
		},
		'/home/rob/Documents/msc/data/lidar/2m_swath/swan_lk_2_2m.txt': {
			'smooth_mult': 0.004712245808863,
			'weight_exp': 1.,
			'alpha': 5.,
		},
		'/home/rob/Documents/msc/data/lidar/2m_swath/VITI_D168_BART_sess12_v1_1_2m.txt': {
			'smooth_mult': 0.002609890637721,
			'weight_exp': 0.,
			'alpha': 5.,
		},
		# '/home/rob/Documents/msc/data/lidar/2m_swath/VITI_D168_BART_sess12_v1_2_2m.txt': {
		# 	'smooth_mult': 7.56583359358737e-05,
		# 	'weight_exp': 1.,
		# 	'alpha': 5.,
		# }
	}

	# Try to create the output directory.
	try:
		os.makedirs(outdir)
	except: pass

	#plot_slope_weights(os.path.join(outdir, 'slope_weights.png'))
	
	# Remove files from output.
	#for f in [x for x in os.listdir(outdir) if x.endswith('.png')]:
	#	os.unlink(os.path.join(outdir, f))

	# Open a stats file. This will contain stats for all runs.
	stats = open(os.path.join(outdir, 'stats.csv'), 'w');
	stats.write('filename,error,alpha,smooth,weight,coord_n,hull_n,above_n,above_max,vel_max,vel_min,acc_max,acc_min,residual\n')

	mode = 'density' # plots, stats

	if mode == 'plots':
		with open('profiles.csv', 'r') as f:
			db = csv.reader(f)
			next(db)
			for line in db:
				if line[0] == '1':
					print(line[7])
					run(line[7], outdir, stats, False, None)
					#break

		stats.close()

	if mode == 'stats':
		for k, v in params.items():
			run(k, outdir, None, True, v)

	if mode == 'density':

		with open('profiles.csv', 'r') as f:
			db = csv.reader(f)
			next(db)
			for line in db:
				if line[0] == '1':
					print(line[7])
					density(line[7])

if __name__ == '__main__':

	outdir = 'plots'
	
	run_profiles(outdir)

	#plot_slope_weights(os.path.join(outdir, 'slope_weights.png'))
	
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