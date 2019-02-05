#!/usr/bin/env python3

import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np
import math 

# Return the squared distance between the points, but only in x.
def lengthY(p0, p1):
	return (p0[0] - p1[0]) ** 2

def cross(p0, p1, p):
	return (p0[0] - p[0]) * (p1[1] - p[1]) - (p0[1] - p[1]) * (p1[0] - p[0])


def hull(pts, alpha = 10.):
	if len(pts) < 3:
		return []
	alpha *= alpha;
	hull = [pts[0], pts[1]]
	for i in range(2, len(pts)):
		# The _length call limits the range of the search for a convex point; causes an alpha-like surface.
		l = lengthY(hull[-2], pts[i])
		c = cross(hull[-2], hull[-1], pts[i])
		while len(hull) > 2 and c >= 0 and (alpha <= 0 or l <= alpha):
			hull.pop()
			l = lengthY(hull[-2], pts[i])
			c = cross(hull[-2], hull[-1], pts[i])

		hull.append(pts[i])
	return hull

def ptsort(a):
	return a[0]

def load(filename):

	tpts = []
	minx = 99999999.
	maxx = -99999999.
	miny = 99999999.
	maxy = -99999999.
	minz = 99999999.
	maxz = -99999999.
	with open(filename, 'rU') as f:
		f.readline()
		line = f.readline()
		while line:
			try:
				x, y, z, c = list(map(float, line.strip().split(',')))
				if c == 2.0:
					if x < minx:
						minx = x
					if x > maxx:
						maxx = x
					if y < miny:
						miny = y
					if y > maxy:
						maxy = y
					if z < minz: minz = z
					if z > maxz: maxz = z
					tpts.append((x, y, z))
			except Exception as e: 
				print(e)
			line = f.readline()

	pts = []
	for x, y, z in tpts:
		d = math.sqrt((x - minx) ** 2 + (y - miny) ** 2)
		pts.append((d, z))
	
	pts = sorted(pts, key = ptsort)
	xx = [x[0] for x in pts]
	yy = [y[1] for y in pts]
	hul = hull(pts)
	hx = [x[0] for x in hul]
	hy = [y[1] for y in hul]

	return (xx, yy, hx, hy)


x, y, hx, hy = load('mt_doug_3.txt')

w = [0.1] * len(hx)

fig = plt.figure(figsize=(8, 4))
ax1 = fig.add_subplot(111)

spl = UnivariateSpline(hx, hy, k=3, s = 10, w = w)
spl.set_smoothing_factor(0.8)
d1 = spl.derivative(n=1)
d2 = spl.derivative(n=2)
kx = spl.get_knots()
xs = np.linspace(hx[0], hx[-1], 1000.)


l_pts, = ax1.plot(x, y, color='#cccccc', marker='o', linestyle='None', ms=1)
l_hull, = ax1.plot(hx, hy, 'ro', ms=1)
l_spl, = ax1.plot(xs, spl(xs), 'g', lw=1)
l_skn, =ax1.plot(kx, spl(kx), 'go', ms=3)
ax1.set_ylabel('Elevation (m)')
ax1.set_xlabel('Distance (m)')
for tl in ax1.get_yticklabels():
    tl.set_color('g')

ax2 = ax1.twinx()
ax2.axhline(0, color='#cccccc')
l_vel, = ax2.plot(xs, d1(xs), 'm', lw=1)
for tl in ax2.get_yticklabels():
    tl.set_color('m')
ymax = max(map(abs, ax2.get_ylim()))
ax2.set_ylim((-ymax, ymax))

ax3 = ax1.twinx()
l_acc, = ax3.plot(xs, d2(xs), 'b', lw=1)
ax3.set_ylabel('Velocity (m/s); Acceleration (m/s²)')
for tl in ax3.get_yticklabels():
    tl.set_color('b')
ymax = max(map(abs, ax3.get_ylim()))
ax3.set_ylim((-ymax, ymax))

plt.legend([l_pts, l_hull, l_spl, l_vel, l_acc], ['Point Cloud', 'Concave Hull Vertices (⍺ = 10m)', 'Altitude (m)', 'Vertical Velocity (m/s)', 'Vertical Acceleration (m/s²)'])

#plt.savefig('../figures/splines_derivs.png', bbox_inches='tight', format='png', dpi=96)
plt.show()

