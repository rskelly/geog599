#!/usr/bin/env python3

''' 
Extracts a slice of a lidar point cloud given a starting and ending point, a platform altitude, 
the rangefinder range and a scan angle. It is assumed that the altitude is held constant, so 
the slice width is also. Loads a las file, outputs a csv file.
'''

import sys
import math
import laspy
import numpy as np

def get_points(lasfile):
	'''
	Returns the points as an ndarray.
	'''
	print('Getting points from', lasfile)
	f = laspy.file.File(lasfile, mode='r')
	coords = np.vstack((f.x, f.y, f.z)).transpose()
	print('Got', len(coords), 'points')
	return coords

def p3angle(p1, p2, p3):
	'''
	Return the angle between 3 points. p2 is the middle point.
	'''
	x1, y1 = p1
	x2, y2 = p2
	x3, y3 = p3
	ax = x1 - x2
	ay = y1 - y2
	bx = x3 - x2
	by = y3 - y2
	return math.atan2(ay, ax) - math.atan2(by, bx)


def plane_filter(coords, dist, v1, v2, ox, oy, oz, ex, ey, ez):
	'''
	Filters the ndarray for points that are within the given distance of the plane.
	The plane is defined by two orthogonal vectors, v1 and v2.
	https://stackoverflow.com/questions/14632769/how-to-get-orthogonal-distances-of-vectors-from-plane-in-numpy-scipy
	'''
	print('Filtering to plane at range', dist)
	# Normal to plane.
	nhat = np.cross(v1, v2)
	nhat = nhat / np.sqrt(np.dot(nhat, nhat))
	out = []
	m = math.pi / 2.
	for x, y, z in list(coords):
		try:
			ds = abs(nhat[0] * (x - ox) + nhat[1] * (y - oy) + nhat[2] * (z - oz)) / math.sqrt(nhat[0] ** 2. + nhat[1] ** 2. + nhat[2] ** 2.)
			if ds < dist:
				ro = abs(p3angle((x, y), (ox, oy), (ex, ey)))
				re = abs(p3angle((x, y), (ex, ey), (ox, oy)))
				if ro <= m and re <= m:
					out.append((x, y, z))
		except Exception as e:
			print(e)
			print(x, y, z, ox, oy, oz, ex, ey, ez)
			sys.exit(1)
	return np.array(out)

def define_plane(start, end):
	'''
	Defines a plane as a tuple of two vectors, one from start to end (normalized)
	and one orthogonal to that, pointing straight up.
	Start and end vectors are np arrays.
	'''
	print('Defining plane')
	# Zero out the altitude changes.
	start[2] = 0.
	end[2] = 0.
	# Normalize the first vector.
	v1 = start - end
	v1 = v1 / np.sqrt(np.dot(v1, v1))
	# Second one just points up.
	v2 = np.array([0., 0., 1.])
	return v1, v2

def point_trans(start):
	'''
	The 2d distance from start to p.
	'''
	sx, sy = start
	return lambda p: (0., math.sqrt((p[0] - sx) ** 2. + (p[1] - sy) ** 2.), p[2])

def run(lasfile, swath, start, end, outfile):
	print('Running', lasfile)
	sx, sy = start
	ex, ey = end
	coords = get_points(lasfile)
	v1, v2 = define_plane(np.array([sx, sy, 0.]), np.array([ex, ey, 0.]))
	coords = plane_filter(coords, swath / 2., v1, v2, sx, sy, 0. , ex, ey, 0.)
	coords = np.apply_along_axis(point_trans((sx, sy)), 1, coords)

	with open(outfile, 'w') as o:
		o.write('x,y,z\n')

		for pt in list(coords):
			o.write(','.join(list(map(str, pt))) + '\n')


import csv

with open('profiles.csv', 'r') as f:
	db = csv.reader(f)
	head = next(db)
	for line in db:
		if line[0] == '1':
			run(line[1], float(line[2]), (float(line[3]), float(line[4])), (float(line[5]), float(line[6])), line[7])

