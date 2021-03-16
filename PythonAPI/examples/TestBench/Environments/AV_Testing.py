# from AVs_internal.AV_UoB import autonomous_vehicle
from path_planner_suite import map_section, Generate_Numeric_Map
import random
import numpy as np

from shapely.geometry import Point
from shapely.geometry import Polygon


class AV_Testing():
	def __init__(self):
		self.create_polygons_map()

		self.occupancy_grid, self.map_x_coord, self.map_y_coord = Generate_Numeric_Map(self.list_of_sections)


		self.pave1         = Polygon([[31.0,-33.1],[37.2,-36.3],[36.2,-38.1],[30.0,-34.9]])
		self.pave2         = Polygon([[25.1,-41.3],[32.6,-45.3],[31.6,-47.1],[24.1,-43.1]]) 
		self.v1_spawn_area = Polygon([[41.8,-44.1],[42.3,-43.2],[48.0,-46.2],[47.5,-47.1]])

		self.AV = autonomous_vehicle(self.occupancy_grid, self.map_x_coord, self.map_y_coord)

		self.spawn_height = 1

	def set(self): 
		vehicle_speed   = random.uniform(5, 5)

		# Setting AV
		self.AV_spawn_point = generate_random_number_in_polygon(self.v1_spawn_area)
		self.AV_dest_point = [[[14.8,-29.6]]]
		self.AV.spawn(self.AV_spawn_point[0][0][0],self.AV_spawn_point[0][0][1], self.spawn_height, 155)
		self.AV.set_speed(vehicle_speed)
		self.AV.set_destination(self.AV_dest_point[0][0][0],self.AV_dest_point[0][0][1],0) 

		
	def reset(self):
		self.destroy()
		self.set()

	def step(self):
		self.AV.step()
		pass

	def destroy(self):
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


class AV2_Testing():
	def __init__(self):

		self.pave1         = Polygon([[-10.0,-71.4],[-9.6,-83.4],[-7.2,-83.3],[-7.5,-71.3]])
		self.Lane1         = Polygon([[-7.8,-61.4],[-6.8,-95.2],[-3.4,-94.9],[-4.4,-61.4]])
		self.Lane2         = Polygon([[-4.4,-61.4],[-3.4,-94.9],[0.3,-94.6],[-0.7,-61.1]])
		self.pave2         = Polygon([[-0.4,-71.0],[-0.2,-83.1],[3.1,-82.9],[2.9,-70.8]]) 
		self.crossing      = Polygon([[-0.4,-74.2],[-0.3,-78.8],[-7.3,-79.0],[-7.3,-74.6]])

		self.AV = autonomous_vehicle(self.occupancy_grid, self.map_x_coord, self.map_y_coord)

		self.spawn_height = 1

	def set(self): 
		vehicle_speed   = random.uniform(5, 5)

		# Setting AV
		self.AV_spawn_point = [[[-6.2,-63.6]]]
		self.AV_dest_point  = [[[-5.3,-92.8]]]
		self.AV.spawn(self.AV_spawn_point[0][0][0],self.AV_spawn_point[0][0][1], self.spawn_height, 155)
		self.AV.set_speed(vehicle_speed)
		self.AV.set_destination(self.AV_dest_point[0][0][0],self.AV_dest_point[0][0][1],0) 

		
	def reset(self):
		self.destroy()
		self.set()

	def step(self):
		self.AV.step()
		pass

	def destroy(self):
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