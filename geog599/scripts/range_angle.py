#!/usr/bin/env python3

'''
Computes the angle of the laser and the horizontal range given the vehicle altitude and absolute range.
'''

import math 

alt = [10., 15., 20., 25., 30., 35., 40., 45., 50.]

range = 100.

print('altitude,range,angle_rad,angle_deg,distance')

for a in alt:

	angle = math.asin(a / range)
	dist = a / math.tan(angle)
	angled = angle * 180. / math.pi

	print(','.join(list(map(str, (a, range, angle, angled, dist)))))

