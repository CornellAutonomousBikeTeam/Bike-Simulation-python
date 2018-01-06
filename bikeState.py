import numpy as np
from constants import *
""" This module contains a class to represent the bike's state """


class Bike(object):

	def __init__(self, xB, yB, phi, psi, delta, w_r, v):
		""" State representation in Matlab """

		#xB, yB initial position coordinates of bicycle
		self.xB = xB
		self.yB = yB
		self.phi = phi #lean
		self.psi = psi #direction (aka thetaB)
		self.delta = delta # Steer Angle!!!!!!
		self.w_r = w_r #lean rate
		self.v = v  #speed
		self.turning_r = 2


	@property
	def vector(self):
		b = np.array([math.cos(self.psi), math.sin(self.psi)])
		return b/np.linalg.norm(b)

	def update(self, bike):
		self.xB = bike.xB
		self.yB = bike.yB
		self.phi = bike.phi
		self.psi = bike.psi
		self.delta = bike.delta
		self.w_r = bike.w_r
		self.v = bike.v
		self.turning_r = bike.turning_r


	def rhs(self, steerD):
		""" Equivalent to rhs in Matlab. Modifies the state object to turn it into the next state """

		deltaD = steerD

		u = K1 * self.phi + K2 * self.w_r + K3 * (self.delta - deltaD)

		if u > 10:
			u = 10
		elif u < -10:
			u = -10

		xdot = self.v * np.cos(self.psi)
		ydot = self.v * np.sin(self.psi)
		phi_dot = self.w_r
		psi_dot = (self.v/L)*(np.tan(self.delta)/np.cos(self.phi))
		delta_dot = u # ideal steer rate
		v_dot = 0
		# wr_dot = (((-(self.v)**2)*self.delta) - B * self.v * u + G * L * self.phi)/(H*L)
		wr_dot = (1/H)*(G*np.sin(self.phi) - np.tan(self.delta)*((self.v**2)/L + B*v_dot/L + np.tan(self.phi)*((B/L)*self.v*phi_dot - (H/(L**2)*(self.v**2)*np.tan(self.delta))))-B*self.v*delta_dot/(L*np.cos(self.delta)**2))
		# Returns u which is the motor command and the zdot vector in the form of a list
		return (u, [xdot, ydot, phi_dot, psi_dot, delta_dot, wr_dot, v_dot])




		