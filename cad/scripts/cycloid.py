#!/usr/bin/env python3

from math import cos, sin, asin, sqrt, pi
import matplotlib.pyplot as plt

'''
Generate a cycloid cam for a lever follower, given the parameters:
- the desired sweep of the follower axis in radians,
- the displacement of the follower axis relative to the cam axis,
- the cam's base circle radius.

The follwer is tangent to the surface of the cam and will not always contact its surface
at the radius, relative to the cam.

1) At each rotation step, compute the projection of the circle as a radius. 
2) Given that the follower is tangent to the cam, find the rotational offset at which it will
   contact the cam to achieve the radius and give the radius and angle at that point.

'''

# The radius of the base circle.
c_rad = 20.

# The desired sweep of the follower in degrees.
f_sweep = 45.

# Displacement of the follower pivot from the cam axis in y.
f_dy = 10.

# Displacement of the follower pivot from the cam axis in x.
f_dx = -60.


def circ_project(t, dr):
	'''
	Project the point on a circle to a cartesian position given
	t, the rotation along the baseline, and r, the radius of the circle.
	At zero radians, the point on the circle is at the bottom, and rotation is
	clockwise.
	Return x and y.
	'''
	return (t * dr, dr - cos(t) * dr)

def to_rad(t):
	'''
	Convert degrees to radians.
	'''
	return t * pi / 180.


def rad_offset(f_dx, f_dy, c_r):
	'''
	f_dx, f_dy The offset of the follower pivot relative to the cam axis.
	c_r The desired radius at the point of contact with the follower.
	'''
	d90 = to_rad(90.)
	d360 = to_rad(360.)

	h = sqrt(f_dx * f_dx + f_dy * f_dy)
	# law of sines
	a = asin(c_r * sin(d90) / h)
	# get the complement of the offset angle from the quad's angles
	c = d90 - (d360 - d90 - a - d90)

	return c


def run(c_rad, f_sweep, f_dx, f_dy):
	
	f_sweep = to_rad(f_sweep)

	# Sweep transformed into a change in radius. 
	f_s_d = 20.

	xx = [0.]
	yy = [0.]

	for t in range(0, 360, 1):
		# To radians
		t0 = to_rad(t)
		# Project the circle
		x, y = circ_project(t0, f_s_d)
		# Get the angle offset for this radius at this point in the rotation.
		#t1 = rad_offset(f_dx, f_dy, c_rad + y)
		t1 = 0.
		#print(t, t0 + t1, c_rad + y)
		xx.append(cos(t0 + t1) * (c_rad + y))
		yy.append(sin(t0 + t1) * (c_rad + y))
		#xx.append(x)
		#yy.append(y)

	ax0 = plt.subplot()
	#ax0.set_ylim([-60., 60.])
	#ax0.set_xlim([-60., 60.])
	#ax0.set_aspect(1.)
	ax0.plot(xx, yy)

	plt.show()

run(c_rad, f_sweep, f_dx, f_dy)