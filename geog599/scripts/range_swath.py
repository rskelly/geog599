#!/usr/bin/env python3

'''
Computes the angle of the laser and the horizontal range given the vehicle altitude and absolute range.
'''

import math 

alt = [10., 15., 20., 25., 30., 35., 40., 45., 50.]
sweep = [0.5, 1., 1.5, 2., 2.5, 5., 10., 15., 20., 30.]

range = 100.

print('altitude,range,angle_rad,angle_deg,swath')

for a in alt:
	for s in sweep:
		sweepr = s * math.pi / 180.
		swath = sweepr * range;

		print(','.join(list(map(str, (a, range, sweepr, s, swath)))))

