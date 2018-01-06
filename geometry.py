import numpy as np
import math
""" This module contains all the vector/math functions """

def unit_vector(p1, p2):
	""" Returns: the unit vector given points [p1] and [p2] """
	v =  np.array([p2[0]-p1[0], p2[1]-p1[1]])
	return v/np.linalg.norm(v)


def dist2(p1, p2):
	""" Returns: the square of the distance between [p1] and [p2] """
	return (np.square(p1[0]-p2[0]) + np.square(p1[1]-p2[1]))


def distance(p1, p2):
	""" Returns: the distance between points [p1] and [p2] """
	return np.sqrt(dist2(p1, p2))	


def get_sign(v):
	""" Returns: the sign of the number [v] """
	if v == 0:
		return 0
	else:
		return v/np.abs(v)

def nearest_point_on_path(path, point):
	""" Returns: the nearest point on the [path] to the [point] """
	point1 = path[0]
	point2 = path[1]
	dist = distance(point1, point2)
	d2 = dist2(point1, point2)
	if (d2 != 0):
		# print 'D2 IS ', d2
		# print "HELLO"
		# print "point is ", point
		# print "point1 is ", point1
		summ = ((point[0] - point1[0])*(point2[0]-point1[0]) + (point[1] - point1[1]) * (point2[1] - point1[1]))
		# print "summmm is ", summ
		t = summ/d2
		# print "D2 AFTER IS", d2
		# print "t IS !!!!!!!!!", t
	else:
		t = 0
	if (t < 0):
		return point1
	elif (t > 1):
		return point2
	elif (t == 0):
		return point
	else:
		x = point1[0] + t * (point2[0]-point1[0])
		y = point1[1] + t * (point2[1]-point1[1])
		return (x, y)


def angle_from_path(bike_direction, p1, p2):
	""" [angle_from_path] returns the angle that the bicycle has to turn to 
	be perpendicular to the line defined by [p1] and [p2] as well as if
	it has to turn clockwise or counterclockwise """
	bike_vector = (math.cos(bike_direction), math.sin(bike_direction))
	path_vector = (p2[0]-p1[0], p2[1]-p1[1])
	dot_product = bike_vector[0]*path_vector[0] + bike_vector[1]*path_vector[1]
	angle = get_sign(dot_product)*math.acos(dot_product/distance(p1, p2))
	return angle


def distance_from_path(point, target_path):
	""" [distance_from_path] calculates the distance from [point] to [target_path] """
	# if target_path is None:
	#	target_path = self.target_path
	p1 = np.array(target_path[0])
	v = unit_vector(target_path[0], target_path[1])
	if v[0] == 0:
		sign = get_sign(v[1])
		return	sign*(p1[0] - point[0])
	v_perp = np.array([v[1], -1*v[0]])
	bike_coords = np.array(point)
	r = p1 - bike_coords
	dist = np.sum(v_perp*r)
	return dist

def line_slope(line):
	""" Returns: slope of [line] """
	return ((line[1][1]-line[0][1])/(line[1][0]-line[0][0]))

def slope_intercept(p1,p2):
	"""Returns [slope, y-intercept] of line given by points p1, p2"""
	m = float(p1[1]-p2[1]) / (p1[0]-p2[0])
	b = p1[1] - m*p1[0]
	return (m,b)

def angle_between_vectors(v1, v2):
	dot = v1[0]*v2[0] + v1[1]*v2[1]
	abs1 = math.sqrt(v1[0]**2 + v1[1]**2)
	abs2 = math.sqrt(v2[0]**2 + v2[1]**2)
	return math.acos(dot/(abs1*abs2))

def angle_between_two_lines(line1, line2):
	v1 = unit_vector(line1[0], line1[1])
	v2 = unit_vector(line2[0], line2[1])
	return angle_between_vectors(v1, v2)

def dot_product(v1, v2):
	return v1[0]*v2[0]+v1[1]*v2[1]

def threeD_unit_vector(p1, p2):
	""" Returns: the unit vector given points [p1] and [p2] """
	v =  np.array([p2[0]-p1[0], p2[1]-p1[1], 0])
	# If it is the zero vector then just return that vector
	if (np.linalg.norm(v)) == 0:
		return v
	else:
		return v/np.linalg.norm(v)

def is_between(p1,p2,p3):
	"""Precondition: points p1, p2, p3 form a line.
	Return true if p3 is between p1 and p2 on the line"""
	min_x, max_x = p1[0], p2[0]
	if min_x > max_x:

		# If p1 and p2 are oriented the other way, swap 'em
		min_x, max_x = max_x, min_x
	return (min_x <= p3[0]) and (p3[0] <= max_x)

def intersect_circle_path(path, r, c):
	"""Returns points of intersection between path and circle with radius r and center c"""
	p1 = path[0]
	p2 = path[1]
	xc = c[0]
	yc = c[1]
	m = slope_intercept(p1, p2)[0]
	b = slope_intercept(p1, p2)[1]
	x_roots = np.roots([m**2+1, 2*m*b-2*m*yc-2*xc, xc**2+b**2-2*yc*b+yc**2-r**2])
	ip1 = (x_roots[0], m*x_roots[0]+b)
	ip2 = (x_roots[1], m*x_roots[1]+b)
	return (ip1, ip2)
	

