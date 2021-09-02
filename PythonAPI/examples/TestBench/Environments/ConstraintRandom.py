# from AVs_internal.AV_UoB import autonomous_vehicle
from Agents.pedestrian import pedestrian
from Agents.vehicle import vehicle
from path_planner_suite import map_section, Generate_Numeric_Map
import random
import numpy as np

from shapely.geometry import Point
from shapely.geometry import Polygon


class ConstraintRandom():
	def __init__(self):
		self.create_polygons_map()

		self.occupancy_grid, self.map_x_coord, self.map_y_coord = Generate_Numeric_Map(self.list_of_sections)


		self.pave1         = Polygon([[31.0,-33.1],[37.2,-36.3],[36.2,-38.1],[30.0,-34.9]])
		self.pave2         = Polygon([[25.1,-41.3],[32.6,-45.3],[31.6,-47.1],[24.1,-43.1]]) 
		self.v1_spawn_area = Polygon([[41.8,-44.1],[42.3,-43.2],[48.0,-46.2],[47.5,-47.1]])
		self.AV_spawn_area = Polygon([[10.1,-30.4],[9.5,-31.4],[16.9,-35.1],[17.5,-34.2]])

		self.ped1 = pedestrian()
		self.ped2 = pedestrian()
		self.vehicle1 = vehicle(self.occupancy_grid, self.map_x_coord, self.map_y_coord)
		self.AV = autonomous_vehicle(self.occupancy_grid, self.map_x_coord, self.map_y_coord)

		self.spawn_height = 1

	def set(self): 
		ped_speed       = random.uniform(0.5, 2.0)
		vehicle_speed   = random.uniform(5, 30)

		# Setting Ped 1
		self.ped1_spawn_point = generate_random_number_in_polygon(self.pave1)
		self.ped1_dest_point = generate_random_number_in_polygon(self.pave2)
		self.ped1.spawn(self.ped1_spawn_point[0][0][0],self.ped1_spawn_point[0][0][1], self.spawn_height, 0)
		self.ped1.set_speed(ped_speed) 
		self.ped1.set_destination(self.ped1_dest_point[0][0][0],self.ped1_dest_point[0][0][1],0) 

		# Setting Ped 2
		self.ped2_spawn_point = generate_random_number_in_polygon(self.pave2)
		self.ped2_dest_point = generate_random_number_in_polygon(self.pave1)
		self.ped2.spawn(self.ped2_spawn_point[0][0][0],self.ped2_spawn_point[0][0][1], self.spawn_height, 0)
		self.ped2.set_speed(ped_speed) 
		self.ped2.set_destination(self.ped2_dest_point[0][0][0],self.ped2_dest_point[0][0][1],0) 

		# Setting Vehicle 1
		self.vehicle1_spawn_point = generate_random_number_in_polygon(self.v1_spawn_area)
		self.vehicle1_dest_point = [[[14.8,-29.6]]]
		self.vehicle1.spawn(self.vehicle1_spawn_point[0][0][0],self.vehicle1_spawn_point[0][0][1], self.spawn_height, 155)
		self.vehicle1.set_speed(vehicle_speed)
		self.vehicle1.set_destination(self.vehicle1_dest_point[0][0][0],self.vehicle1_dest_point[0][0][1],0) 

		# Setting AV
		self.AV_spawn_point = generate_random_number_in_polygon(self.AV_spawn_area)
		self.AV_dest_point = [[[47.9,-50.9]]]
		self.AV.spawn(self.AV_spawn_point[0][0][0],self.AV_spawn_point[0][0][1], self.spawn_height, -25)
		self.AV.set_speed(vehicle_speed)
		self.AV.set_destination(self.AV_dest_point[0][0][0],self.AV_dest_point[0][0][1],0) 

		

		
	def reset(self):
		self.destroy()
		self.set()

	def step(self):
		self.ped1.step() 
		self.ped2.step() 
		self.vehicle1.step()
		self.AV.step()
		pass

	def destroy(self):
		self.ped1.destroy()
		self.ped2.destroy()
		self.vehicle1.destroy()
		self.AV.destroy()
		pass

	def create_polygons_map(self):
		boundaries_list  = [[[12.6,-23.5],[11.6,-25.3],[50.9,-46.1],[51.9,-44.3]],
							[[11.6,-25.3],[9.6,-28.6],[48.9,-49.4],[50.9,-46.1]],
							[[9.6,-28.6],[7.8,-32.2],[47.2,-53.0],[48.9,-49.4]],
							[[7.8,-32.2],[6.7,-34.0],[46.0,-54.8],[47.2,-53.0]]]

		Polygons_travel_direction = np.array([0,9,9,0])

		self.list_of_sections = []
		self.list_of_polygons = []

		for i in range(len(Polygons_travel_direction)):
			Sec = map_section(ID = i, travel_direction = Polygons_travel_direction[i], boundaries = boundaries_list[i])
			self.list_of_sections.append(Sec) 
			self.list_of_polygons.append(Sec.polygon)

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
