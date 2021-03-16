
from shapely.geometry import Polygon
import math
import numpy as np
import random
from shapely.geometry import Point
# from shapely.geometry import Polygon

def Ticking(world,frame):
	# Tick
	world.tick()
	# Get world snapshot   
	world_snapshot = world.get_snapshot()
	ts             = world_snapshot.timestamp
	if frame is not None:
		if ts.frame_count != frame + 1:
			logging.warning('frame skip!')
	frame          = ts.frame_count
	return ts, frame


def wraptopi(x):
	if x > np.pi:
		x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
	elif x < -np.pi:
		x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
	return x

def calculateDistance(x1,y1,x2,y2):
	dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return dist


def AgentColourToRGB(agentColour):

	if agentColour == "red":
		RGB_colour = "255,0,0"

	elif agentColour == "yellow":
		RGB_colour = "255,200,0"

	elif agentColour == "blue":
		RGB_colour = "0,0,255"

	elif agentColour == "white":
		RGB_colour = "255,255,255"

	elif agentColour == "black":
		RGB_colour = "0,0,0"

	else:
		RGB_colour = "255,0,0" # Default to red

	return RGB_colour


def generate_random_number_in_polygon(polygon, number=1):
	list_of_points = []
	minx, miny, maxx, maxy = polygon.bounds
	counter = 0
	while counter < number:
		random_x = random.uniform(minx, maxx) 
		random_y = random.uniform(miny, maxy)
		pnt = Point(random_x, random_y)
		if polygon.contains(pnt):
			list_of_points.append(list(pnt.coords))
			counter += 1
	return list_of_points