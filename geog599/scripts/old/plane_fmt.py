#!/usr/bin/env python3

# Formats 3d lidar text files into 2d y-z planar point clouds.

import sys
import math

# File, start coord, end coord,
files = [
	('nrcan_slice_1_8m.txt', (619970.248, 5611536.913), (620289.757, 5611592.180), 'nrcan_slice_1_8m_proc.txt')
]

def line_dist(p, p0, p1):
	x, y = p
	x0, y0 = p0
	x1, y1 = p1
	return abs((y1 - y0) * x - (x1 - x0) * y + x1 * y0 - y1 * x0) / math.sqrt((y1 - y0) ** 2 + (x1 - x0) ** 2)


def point_dist(p0, p1):
	return math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2)


def process_file(filename, p0, p1, altitude, angle, sweep, outfile):

	h = altitude / math.sin(angle * math.pi / 180.)
	w = (sweep * math.pi / 180.) * h
	print('altitude:', altitude, 'range:', h, 'width:', w)

	with open(filename, 'rU') as f:
		with open(outfile, 'w') as o:
			line = f.readline()
			while line:
				try:
					x, y, z = list(map(float, line.strip().split(',')))
					if line_dist((x, y), p0, p1) < w / 2.:
						y = point_dist((x, y), p0)
						o.write('{y},{z}\n'.format(y = y, z = z))
				except Exception as e: 
					print(e)
					break
				line = f.readline()


altitude = 10.
angle = 45.
sweep = 30.

for file, p0, p1, outfile in files:
	process_file(file, p0, p1, altitude, angle, sweep, outfile)
