import requests
import json
import urllib
import math
import numpy as np
from itertools import groupby

""" Run server """

def convert(latitude, longitude):
	""" Mercator map projection 
	http://stackoverflow.com/questions/14329691/covert-latitude-longitude-point-to-a-pixels-x-y-on-mercator-projection """

	mapWidth    = 1000
	mapHeight   = 1000

	x = (longitude+180)*(mapWidth/360.0)

	latRad = latitude*math.pi/180

	mercN = math.log(math.tan((math.pi/4)+(latRad/2)))

	y = (mapHeight/2)-(mapWidth*mercN/(2*math.pi))

	# Hardcoded the scaling of the simulation, y - coordinate is flipped to appear properly
	# return (x*(10000)%10, mapWidth - y*(10000)%10)
	return (x*100, (mapWidth - y)*100)

def convert2(latitude, longitude):
	"""http://stackoverflow.com/questions/1019997/convert-lat-longs-to-x-y-co-ordinates"""
	mapWidth    = 1000
	mapHeight   = 1000

	y = ((-1 * latitude) + 90) * (mapHeight / 180.0);
	x = (longitude + 180) * (mapWidth / 360.0);

	return (x, y)
	
# Also this --> http://stackoverflow.com/questions/1019997/convert-lat-longs-to-x-y-co-ordinates

def math_convert(latitude, longitude):
	rad_earth = 6371000
	# Hardcoded origin
	origin = (np.deg2rad(42.44814), np.deg2rad(-76.48489))
	latitude = np.deg2rad(latitude)
	longitude = np.deg2rad(longitude)
	a = (np.sin((latitude-origin[0])/2.0))**2 + np.cos(latitude)*np.cos(origin[0])*(np.sin((longitude-origin[1])/2.0))**2
	theta = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
	d = theta * rad_earth
	bearing = bearing_from_origin(origin, latitude, longitude)
	# print "HERE", d
	return (d*np.sin(bearing), d*np.cos(bearing))

def bearing_from_origin(origin, latitude, longitude):
	y = np.sin(longitude-origin[1]) * np.cos(latitude)
	x = np.cos(origin[0]) * np.sin(latitude) - np.sin(origin[0])*np.cos(latitude)*np.cos(longitude-origin[1])
	#Bearing in radians
	bearing = np.arctan2(y, x)
	#Bearing in degrees
	bearing_deg = np.rad2deg(bearing)
	# print "BEARING", bearing_deg
	return bearing



def parse_json(presets=False):
	#Gets url object and makes it into string
	# url = urllib.urlopen("http://localhost:8000/secretdatatransfer")


	if (not presets):
		url = urllib.urlopen("https://abserver-168813.appspot.com/getwaypoints") #NEW
		string = url.read()

		#Make into dictionary
		dictionary = json.loads(string)
		legs = dictionary["legs"] #points list

		points = []
		points2 = []

		#Accumulates list of point tuples
		for i in legs:
			for j in (i["points"]):
				latitude = j['lat']
				longitude = j['lng']
				xy = math_convert(latitude, longitude)
				xy2 = convert2(latitude, longitude)
				points.append(xy)
				points2.append(xy2)
				
		# Remove consecutive duplicate waypoints
		points = [x[0] for x in groupby(points)]

		# Scaling Issue of points. Need to ZOOM IN because discrepancies are very small
		print points
		# print points2
		
		return points

	else:
		point1 = math_convert(42.4450492859, -76.4836349487)
		point2 = math_convert(42.4442214966, -76.4835510254)
		return [point1, point2] 




# parse_json()

#Gets latitude of first point
# print legs[0]["points"][0]['lat']



# print convert(-76.4852, 42.45026)
# print convert(-76.48506, 42.450390000000006)

# LEG: A separate leg will be present for each waypoint or destination specified.
# Each leg consists of a series of steps

#latitude and longitude conversion to x, y coordinates
# dx = (lon2-lon1)*40000*math.cos((lat1+lat2)*math.pi/360)/360
# dy = (lat1-lat2)*40000/360



# int x =  (int) ((MAP_WIDTH/360.0) * (180 + lon));
# int y =  (int) ((MAP_HEIGHT/180.0) * (90 - lat));


#Print string
# print len(legs)
