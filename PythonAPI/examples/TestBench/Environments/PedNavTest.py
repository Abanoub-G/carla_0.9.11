# from AVs_internal.AV_UoB import autonomous_vehicle
from Agents.pedestrian import pedestrian
import random
import numpy as np

from shapely.geometry import Point
from shapely.geometry import Polygon



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


	

class PedNavTest():
	def __init__(self):
		self.ped1 = pedestrian()

		self.spawn_height = 1

	def set(self): 
		ped_speed       = 3

		# Setting Ped 1
		self.ped1_spawn_point = [-17.0,-48.3]
		self.ped1_dest_point  = [-17.7,-34.8] # on pavement
		self.ped1_dest_point  = [-4.2,-51.3] # on road
		self.ped1.spawn(self.ped1_spawn_point[0],self.ped1_spawn_point[1], self.spawn_height, 0)
		self.ped1.set_speed(ped_speed) 
		self.ped1.set_destination(self.ped1_dest_point[0],self.ped1_dest_point[1],0) 


	def step(self):
		self.ped1.step() 
		pass

	def destroy(self):
		self.ped1.destroy()
		pass
