#!/usr/bin/env python


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os

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


def norm(mean, sigma):
	if sigma <= 0.:
		return mean
	return np.random.normal(mean, sigma)

def line(cols, rows, sigma = 0.):
	xs = []
	ys = []
	zs = []
	for r in range(rows):
		for c in range(cols):
			xs.append(norm(c, sigma))
			ys.append(norm(r, sigma))
			zs.append(norm(0., sigma))
	return xs, ys, zs

if __name__ == '__main__':

	outdir = sys.argv[1]

	try:
		os.makedirs(outdir)
	except: pass

	xs, ys, zs = line(20, 20, .0)
	plot(xs, ys, zs, 'Smooth Plane', 20., 20., .1, os.path.join(outdir, 'smooth_plane.svg'))
	xs, ys, zs = line(20, 20, .001)
	plot(xs, ys, zs, 'Sigma 0.001 Plane', 20., 20., .1, os.path.join(outdir, 'sigma_0001_plane.svg'))
	xs, ys, zs = line(20, 20, .01)
	plot(xs, ys, zs, 'Sigma 0.01 Plane', 20., 20., .1, os.path.join(outdir, 'sigma_001_plane.svg'))


