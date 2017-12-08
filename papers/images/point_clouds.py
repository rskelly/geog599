#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os
from math import sin, cos, pi

def plot(xs, ys, zs, title, xlim, ylim, zlim, filename):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(xs, ys, zs)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	ax.set_title(title)
	plt.xlim(0., xlim)
	plt.ylim(0., ylim)
	ax.set_zlim(-zlim, zlim)
	plt.savefig(filename)


def gauss(mean, sigma):
	if sigma <= 0.:
		return mean
	return np.random.normal(mean, sigma)

## Surface functions.
def plane(x, y, width, height):
	return 0.

def block(x, y, width, height):
	if x > width / 3 and x < 2 * width / 3 and y > height / 3 and y < 2 * height / 3:
		return 0.02
	else:
		return 0

# Scan functions. Takes a surface function for fn.
def grid(cols, rows, sigma, fn):
	xs = []
	ys = []
	zs = []
	for r in range(rows):
		for c in range(cols):
			xs.append(gauss(c, sigma))
			ys.append(gauss(r, sigma))
			zs.append(gauss(fn(c, r, cols, rows), sigma))
	return xs, ys, zs

def sinusoid(cols, rows, sigma, fn):
	xs = []
	ys = []
	zs = []
	for r in range(rows * 10):
		x = cols / 2. + cos(r / 10. * pi / 4.) * cols / 2.
		xs.append(gauss(x, sigma))
		ys.append(gauss(r / 10., sigma))
		zs.append(gauss(fn(x, r / 10., cols, rows), sigma))
	return xs, ys, zs

def line(cols, rows, sigma, fn):
	xs = []
	ys = []
	zs = []
	for r in range(rows * 10):
		xs.append(gauss(cols / 2, sigma))
		ys.append(gauss(r / 10., sigma))
		zs.append(gauss(fn(cols / 2, r / 10., cols, rows), sigma))
	return xs, ys, zs



if __name__ == '__main__':

	outdir = sys.argv[1]

	try:
		os.makedirs(outdir)
	except: pass

	matplotlib.rc('font', family='Abyssinica SIL')

	xs, ys, zs = grid(20, 20, .0, plane)
	plot(xs, ys, zs, u'Planar Grid - $\sigma0$', 20., 20., .1, os.path.join(outdir, 'plane_grid_0.svg'))
	xs, ys, zs = grid(20, 20, .001, plane)
	plot(xs, ys, zs, u'Planar Grid - $\sigma0.001$', 20., 20., .1, os.path.join(outdir, 'plane_grid_0001.svg'))
	xs, ys, zs = grid(20, 20, .01, plane)
	plot(xs, ys, zs, u'Planar Grid - $\sigma0.01$', 20., 20., .1, os.path.join(outdir, 'plane_grid_001.svg'))

	xs, ys, zs = sinusoid(20, 20, .0, plane)
	plot(xs, ys, zs, u'Sinusoidal Scan - $\sigma0$', 20., 20., .1, os.path.join(outdir, 'plane_sinus_0.svg'))
	xs, ys, zs = sinusoid(20, 20, .001, plane)
	plot(xs, ys, zs, u'Sinusoidal Scan - $\sigma0.001$', 20., 20., .1, os.path.join(outdir, 'plane_sinus_0001.svg'))
	xs, ys, zs = sinusoid(20, 20, .01, plane)
	plot(xs, ys, zs, u'Sinusoidal Scan - $\sigma0.01$', 20., 20., .1, os.path.join(outdir, 'plane_sinus_001.svg'))

	xs, ys, zs = line(20, 20, .0, plane)
	plot(xs, ys, zs, u'Linear Scan - $\sigma0$', 20., 20., .1, os.path.join(outdir, 'plane_linear_0.svg'))
	xs, ys, zs = line(20, 20, .001, plane)
	plot(xs, ys, zs, u'Linear Scan - $\sigma0.001$', 20., 20., .1, os.path.join(outdir, 'plane_linear_0001.svg'))
	xs, ys, zs = line(20, 20, .01, plane)
	plot(xs, ys, zs, u'Linear Scan - $\sigma0.01$', 20., 20., .1, os.path.join(outdir, 'plane_linear_001.svg'))


	xs, ys, zs = grid(20, 20, .0, block)
	plot(xs, ys, zs, u'Planar Grid with Obstacle - $\sigma0$', 20., 20., .1, os.path.join(outdir, 'block_grid_0.svg'))
	xs, ys, zs = grid(20, 20, .001, block)
	plot(xs, ys, zs, u'Planar Grid with Obstacle - $\sigma0.001$', 20., 20., .1, os.path.join(outdir, 'block_grid_0001.svg'))
	xs, ys, zs = grid(20, 20, .01, block)
	plot(xs, ys, zs, u'Planar Grid with Obstacle - $\sigma0.01$', 20., 20., .1, os.path.join(outdir, 'block_grid_001.svg'))

	xs, ys, zs = sinusoid(20, 20, .0, block)
	plot(xs, ys, zs, u'Sinusoidal Scan with Obstacle - $\sigma0$', 20., 20., .1, os.path.join(outdir, 'block_sinus_0.svg'))
	xs, ys, zs = sinusoid(20, 20, .001, block)
	plot(xs, ys, zs, u'Sinusoidal Scan with Obstacle - $\sigma0.001$', 20., 20., .1, os.path.join(outdir, 'block_sinus_0001.svg'))
	xs, ys, zs = sinusoid(20, 20, .01, block)
	plot(xs, ys, zs, u'Sinusoidal Scan with Obstacle - $\sigma0.01$', 20., 20., .1, os.path.join(outdir, 'block_sinus_001.svg'))

	xs, ys, zs = line(20, 20, .0, block)
	plot(xs, ys, zs, u'Linear Scan with Obstacle - $\sigma0$', 20., 20., .1, os.path.join(outdir, 'block_linear_0.svg'))
	xs, ys, zs = line(20, 20, .001, block)
	plot(xs, ys, zs, u'Linear Scan with Obstacle - $\sigma0.001$', 20., 20., .1, os.path.join(outdir, 'block_linear_0001.svg'))
	xs, ys, zs = line(20, 20, .01, block)
	plot(xs, ys, zs, u'Linear Scan with Obstacle - $\sigma0.01$', 20., 20., .1, os.path.join(outdir, 'block_linear_001.svg'))

