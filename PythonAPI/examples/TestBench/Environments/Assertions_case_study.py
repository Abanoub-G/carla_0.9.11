import random
import numpy as np

from Agents.pedestrian import pedestrian
from Agents.vehicle import vehicle
from TB_common_functions import log, calculateDistance
# from Agents.traffic_light import traffic_light

def interpolate_path(path_tuple, increments_resolution):
	
	path_tuple_interpolated = []
	
	path_tuple_interpolated.append(path_tuple[0])
	
	for i in range(len(path_tuple)-1):
		
		diff = abs(np.subtract(path_tuple[i], path_tuple[i+1]))
		dist = np.hypot(diff[0],diff[1])
		
		if dist > increments_resolution:

			num_points_remainder = dist%increments_resolution
			num_points = int(np.ceil((dist - num_points_remainder)/increments_resolution)) + 1
			
			if num_points_remainder > 0:
				num_points = num_points + 1 
				
			x = np.linspace(path_tuple[i][0], path_tuple[i+1][0], num_points)
			x = x[1:]
			x = [round(num, 1) for num in x]

			y = np.linspace(path_tuple[i][1], path_tuple[i+1][1], num_points)
			y = y[1:]
			y = [round(num, 1) for num in y]

			interolated_points = list(zip(x, y))
			path_tuple_interpolated = path_tuple_interpolated + interolated_points
			# print(path_tuple_interpolated)
			
	return(path_tuple_interpolated)

	

class Assertions_case_study():
	def __init__(self,world):
		self.actors_list = []
		self.veh1 = vehicle(1)
		self.veh2 = vehicle(2)
		self.veh3 = vehicle(3)
		self.actors_list.append(self.veh1)
		self.actors_list.append(self.veh2)
		self.actors_list.append(self.veh3)
		self.spawn_height = 8
		self.tests_ended = False

		self.world = world

		logging_time_increment = 0.1
		self.tests_logs = log(logging_time_increment) 

	def set(self): 
		# spawn(self,x,y,z,yaw):
		# Setting Veh 1
		# self.veh1_path = [[-6.44,-42.19],[-2.94,-61.59],[-6.44,-79.05]]
		self.veh1_path = [(-368.2,209.8),(-342.6,197.1),(-334.4,197.1),(-317.9,189.1),(-314.5,183.3),(-280.1,166.3)]
		self.veh2_path = [[-328.4,190.3]]
		self.veh3_path = [(-242.3,151.6),(-365.6,212.7)]

		# interpolate path(s)
		interpolation_resolution_min = 1
		self.veh1_path_interpolated  = interpolate_path(self.veh1_path, interpolation_resolution_min)
		self.veh2_path_interpolated  = interpolate_path(self.veh2_path, interpolation_resolution_min)
		self.veh3_path_interpolated  = interpolate_path(self.veh3_path, interpolation_resolution_min)

		self.veh1.set_path(self.veh1_path_interpolated)
		self.veh2.set_path(self.veh3_path_interpolated)
		self.veh3.set_path(self.veh3_path_interpolated)

		self.veh1.spawn(self.veh1_path[0][0],self.veh1_path[0][1], self.spawn_height, -25)
		self.veh2.spawn(self.veh2_path[0][0],self.veh2_path[0][1], self.spawn_height, -25)
		self.veh3.spawn(self.veh3_path[0][0],self.veh3_path[0][1], self.spawn_height, 155)

		# self.veh1_speed = 5
		# self.veh2_speed = 0
		# self.veh3_speed = 5


	def step(self):
		self.veh1.step() 
		self.veh2.step() 
		self.veh3.step()

		self.veh2.stop = True



		# Log 
		t   = self.world.get_snapshot().timestamp.elapsed_seconds
		fps = 1 / (self.world.get_snapshot().timestamp.frame_count)
		self.tests_logs.append(0,0,self.actors_list,t,fps)

		pass

	def destroy(self):
		self.veh1.destroy()
		self.veh2.destroy()
		self.veh3.destroy()
		self.tests_logs.write_file("AssertionCheckingCaseStudyLogs.txt")
		pass
