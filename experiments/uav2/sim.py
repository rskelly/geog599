#!/usr/bin/env python3

from math import sqrt, sin, cos, pi
import time

g = 9.80665
rho = 1.2250 # @15C
C = 1.05
A = 0.5

def drag(rho, v, C, A):
	'''
	Calculate drag force.
	rho - density of air.
	v - velocity.
	C - drag coefficient.
	A - cross-sectional area.
	'''
	return 0.5 * rho * v * v * C * A

def mag(vec):
	return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])

class V:

	max_thrust = 10.
	min_thrust = 0.
	max_avelocity = 3.
	min_avelocity = -3.
	mass = 10.

	def __init__(self):
		self.thrust = 0
		self.lvelocity = [0., 0., 0.]
		self.avelocity = [0., 0., 0.]
		self.rotation = [0., 0., 0.]
		self.position = [0., 0., 0.]
		
	def rotate_by(self, x, y, z):
		'''
		Rotate by the given amount in x, y and z.
		'''
		r = self.rotation
		r[0] += x
		r[1] += y
		r[2] += z
		self.calculate()

	def adj_thrust(self, delta):
		'''
		Change the current thrust force by delta.
		Will be limited by max_thrust and min_thrust.
		'''
		self.thrust = min(V.max_thrust, max(V.min_thrust, self.thrust + delta))
		self.calculate()

	def calculate(self):
		t = self.thrust
		r = self.rotation
		lv = self.lvelocity

		lv[0] = (t * cos(pi / 2. - r[1]) / V.mass) - drag(rho, lv[0], C, A)
		lv[2] = (t * sin(pi / 2. - r[1]) / V.mass) - drag(rho, lv[2], C, A) - g


	def update(self, interval):
		'''
		Update the dynamic state of the vehicle. Interval is given 
		in seconds.
		'''
		t = self.thrust
		r = self.rotation
		lv = self.lvelocity
		p = self.position

		p[0] += lv[0] * interval
		p[2] = max(0., p[2] + lv[2] * interval)

		print('linear velocity', lv)
		print('position', p)

if __name__ == '__main__':

	v = V()
	v.adj_thrust(.1)
	#v.rotate_by(0., .1, 0.)	
	while True:
		v.adj_thrust(.1)
		v.update(1.)
		time.sleep(1.)



