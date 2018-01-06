import bikeState
import numpy as np
""" This module contains a class to represent the Map Model and all its functions """

class Map_Model(object):


	def __init__(self, bike, waypoints, obstacles, paths = []):
		""" Initializes the Map Model """
		self.bike = bike
		self.paths = self.init_paths(waypoints)
		self.waypoints = waypoints
		self.obstacles = obstacles

	def init_paths(self, waypoints):
		""" Initializes paths fron input waypoints """
		paths = []
		if len(waypoints) < 2:
			return paths
		else:
			for i in range(1, len(waypoints)):
				paths.append((waypoints[i-1], waypoints[i]))
			return paths

	def add_path(self, p1, p2):
		""" Adds a new path from point p1 to point p2 at the end of the path list """
		self.paths.append([p1,p2])
		self.waypoints[0].append(p1[0])
		self.waypoints[0].append(p2[0])
		self.waypoints[1].append(p1[1])
		self.waypoints[1].append(p2[1])


	def add_point(self, p):
		""" Adds a new point p to the list of waypoints. If it is not the first point added in 
		the waypoints list, then a path is also added from the last point in waypoints to the 
		new point p that we add """
		if (len(self.paths) != 0):
			previous_point = self.paths[-1][1]
			self.paths.append([previous_point, p])
		elif (len(self.waypoints[0])==1):
			#If the first point had been added we create the first path from the first point to p
			self.paths.append([(self.waypoints[0][0], self.waypoints[1][0]), p])
		self.waypoints[0].append(p[0])
		self.waypoints[1].append(p[1])


	def close_path(self):
		""" Adds a path from the last point to the first point in the waypoints list """
		self.add_point(self.paths[0][0])


	def draw_circle(self, center, r, n_points, degrees = 2*np.pi):
		""" Draws a circle with given characteristics """
		deg_inc = float(degrees)/n_points
		theta = deg_inc
		p0 = np.array(center) + np.array([r, 0])
		p1 = np.array(center) + np.array([r*np.cos(theta), r*np.sin(theta)])
		self.add_path(p0, p1)
		for i in range(2, n_points+1):
			next_point = np.array(center) + np.array([r*np.cos(i*theta), r*np.sin(i*theta)])
			self.add_point(next_point)


