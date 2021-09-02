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


# =================================================================================================================================================
#                                        Case study to provide log files for game transaltion to OpenScenario files
#                      
# =================================================================================================================================================

class GameRun():
	def __init__(self,world):
		self.spawn_height = 2
		self.tests_ended = False

		self.world = world

		self.logging_time_increment = 0.1
		self.tests_logs = log(self.logging_time_increment) 
		# self.i = 0

	def set(self): 
		self.timeout_thresh = 30
		self.actors_list = []
		self.veh1 = vehicle(1) # vehicle turning right 
		self.veh2 = vehicle(2) # vehicle turning right  
		# self.ped1 = pedestrian(2,"adult") # pedestrian crossing zebra line
		# self.ped1.set_speed(1.5)
		self.actors_list.append(self.veh1)
		self.actors_list.append(self.veh2)
		# self.actors_list.append(self.ped1)
		self.test_ended = False
		self.tests_logs = log(self.logging_time_increment) 
		# spawn(self,x,y,z,yaw):
		# Setting Veh 1
		self.veh1_path = [[-5.26,-98.13],[-4.36,-124.83], [-6.16,-132.83], [-12.96,-135.23], [-39.36,-135.43]]
		# self.veh2_path = [[16.63,-134.53],[-65.76,-136.13]] # safe
		self.veh2_path = [[30.63,-134.23],[-65.76,-136.13]] # crash
		# self.veh2_path = [[-26.26,-7.95],[-16.04,-16.99], [-8.64,-25.19], [-6.84,-35.19], [-5.44,-74.89],[-4.44,-127.99], [-8.84,-134.99], [-31.54,-135.39]] # crash
		# self.ped1_path = [[-22.24,-62.49], [-24.64,-81.39], [-17.44,-100.79], [-49.84,-97.19]]
		

		# # Setting Ped 1
		# if self.i == 0:
		# 	# Creating several paths for contious runs
		# 	ped1_path_series_start = [-283.90, 176.50] # Start of Trajectory series for pedestrian with varying starting point
		# 	ped1_path_series_end = [-295.80,182.10] # End of Trajectory series for pedestrian with varying starting point
		# 	step = 2
		
		# 	x = [-ped1_path_series_start[0], -ped1_path_series_end[0]]
		# 	print("x[0] = ",x[0])
		# 	print("x[1] = ",x[1])
		# 	print("step = ",step)
		# 	x_new = np.arange(x[0],x[1],step)
		# 	print("x_new = ",x_new)
		# 	y = [ped1_path_series_start[1], ped1_path_series_end[1]]
		# 	self.y_new = np.interp(x_new, x, y)
		# 	self.y_new = np.append(self.y_new,ped1_path_series_end[1])
		# 	self.x_new = x_new * -1
		# 	self.x_new = np.append(self.x_new,ped1_path_series_end[0])

		
		# self.ped1_path = [(self.x_new[self.i],self.y_new[self.i]),(-295.80,182.10), (-300.50,172.50), (-288.30,165.70)] 
		# # self.ped1_path = [[-279.80,170.10], [-310.50,185.40]] 
		# self.i = self.i + 1
		# if self.i == len(self.x_new):
		# 	self.tests_ended = True
		# print("i",self.i)
		# print("self.x_new",self.x_new)
		# print("length of x_new",len(self.x_new))
		

		# interpolate path(s)
		interpolation_resolution_min = 1
		self.veh1_path_interpolated  = interpolate_path(self.veh1_path, interpolation_resolution_min)
		self.veh2_path_interpolated  = interpolate_path(self.veh2_path, interpolation_resolution_min)
		# self.ped1_path_interpolated  = interpolate_path(self.ped1_path, interpolation_resolution_min)
		# self.veh3_path_interpolated  = interpolate_path(self.veh3_path, interpolation_resolution_min)

		self.veh1.set_path(self.veh1_path_interpolated)
		self.veh2.set_path(self.veh2_path_interpolated)
		# self.ped1.set_path(self.ped1_path_interpolated)
		# self.veh3.set_path(self.veh3_path_interpolated)

		self.veh1.spawn(self.veh1_path[0][0],self.veh1_path[0][1], self.spawn_height, 240)
		self.veh2.spawn(self.veh2_path[0][0],self.veh2_path[0][1], self.spawn_height, 240)
		# self.ped1.spawn(self.ped1_path[0][0],self.ped1_path[0][1], self.spawn_height, 0)
		# self.veh3.spawn(self.veh3_path[0][0],self.veh3_path[0][1], self.spawn_height, 155)

		# self.veh1_speed = 5
		# self.veh2_speed = 0
		# self.veh3_speed = 5


	def step(self):
		self.veh1.step()
		self.veh2.step() 
		# self.ped1.step() 
		# self.veh3.step()



		# Log 
		t   = self.world.get_snapshot().timestamp.elapsed_seconds
		fps = 1 / (self.world.get_snapshot().timestamp.frame_count)
		self.tests_logs.append(0,0,self.actors_list,t,fps)
		if self.tests_logs.time_array[-1] >= self.timeout_thresh:
			self.test_ended = True
			self.tests_ended = True



		pass

	def destroy(self):
		self.veh1.destroy()
		self.veh2.destroy()
		# self.ped1.destroy()
		# self.veh3.destroy()
		# self.tests_logs.write_file("GameRunTown3_intersection_safe.txt")
		self.tests_logs.write_file("GameRunTown3_intersection_dangerous.txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_NearMiss.txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_Crash.txt")
		pass