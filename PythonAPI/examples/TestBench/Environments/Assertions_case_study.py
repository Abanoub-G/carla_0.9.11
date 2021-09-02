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
#                                                                      Overtaking
# =================================================================================================================================================

class Assertions_case_study_overtaking():
	def __init__(self,world):
		# self.actors_list = []
		# self.veh1 = vehicle(1)
		# self.veh2 = vehicle(2)
		# self.veh3 = vehicle(3)
		# self.actors_list.append(self.veh1)
		# self.actors_list.append(self.veh2)
		# self.actors_list.append(self.veh3)
		self.spawn_height = 8
		self.tests_ended = False

		self.world = world

		self.logging_time_increment = 0.1
		self.tests_logs = log(self.logging_time_increment) 
		self.i = 0

	def set(self): 
		self.actors_list = []
		self.veh1 = vehicle(1)
		self.veh2 = vehicle(2)
		self.veh3 = vehicle(3)
		self.actors_list.append(self.veh1)
		self.actors_list.append(self.veh2)
		self.actors_list.append(self.veh3)
		self.test_ended = False
		self.tests_logs = log(self.logging_time_increment) 
		# spawn(self,x,y,z,yaw):
		# Setting Veh 1
		# self.veh1_path = [[-6.44,-42.19],[-2.94,-61.59],[-6.44,-79.05]]
		self.veh1_path = [(-368.2,209.8),(-342.6,197.1),(-334.4,197.1),(-317.9,189.1),(-314.5,183.3),(-280.1,166.3)]
		self.veh2_path = [[-328.4,190.3]]
		# self.veh3_path = [(-242.3,151.6),(-365.6,212.7)] # Safe Trajectory
		# self.veh3_path = [(-258.6,159.4),(-365.6,212.7)] # Near miss Trajectory
		# self.veh3_path = [(-279.0,169.4),(-365.6,212.7)] # Crash Trajectory
		# self.veh3_path = [(-302.0,181.4),(-365.6,212.7)] # Ending Interolation Trajectory
		if self.i == 0:
			# Creating several paths for contious runs
			veh3_path_series_start = [-242.3, 151.6] # Start of Trajectory series for vehicle with varying starting point
			veh3_path_series_end = [-322.6, 191.2] # End of Trajectory series for vehicle with varying starting point
			step = 5
			self.timeout_thresh = 32
			x = [-veh3_path_series_start[0], -veh3_path_series_end[0]]
			x_new = np.arange(x[0],x[1],step)
			y = [veh3_path_series_start[1], veh3_path_series_end[1]]
			self.y_new = np.interp(x_new, x, y)
			self.y_new = np.append(self.y_new,veh3_path_series_end[1])
			self.x_new = x_new * -1
			self.x_new = np.append(self.x_new,veh3_path_series_end[0])

		
		self.veh3_path = [(self.x_new[self.i],self.y_new[self.i]),(-365.6,212.7)] 
		self.i = self.i + 1
		if self.i == len(self.x_new):
			self.tests_ended = True


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
		if self.tests_logs.time_array[-1] >= self.timeout_thresh:
			self.test_ended = True



		pass

	def destroy(self):
		self.veh1.destroy()
		self.veh2.destroy()
		self.veh3.destroy()
		self.tests_logs.write_file("Overtaking_TestID"+str(self.i)+".txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_NearMiss.txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_Crash.txt")
		pass

# =================================================================================================================================================
#                                                                      Left turn at intersection
# =================================================================================================================================================

class Assertions_case_study_left_turn_at_intersection():
	def __init__(self,world):
		self.spawn_height = 8
		self.tests_ended = False

		self.world = world

		self.logging_time_increment = 0.1
		self.tests_logs = log(self.logging_time_increment) 
		self.i = 0

	def set(self): 
		self.actors_list = []
		self.veh1 = vehicle(1) # vehicle going straight 
		self.veh2 = vehicle(2) # vehicle turning left
		self.actors_list.append(self.veh1)
		self.actors_list.append(self.veh2)
		self.test_ended = False
		self.tests_logs = log(self.logging_time_increment) 
		# spawn(self,x,y,z,yaw):
		# Setting Veh 1
		# self.veh1_path = [[-6.44,-42.19],[-2.94,-61.59],[-6.44,-79.05]]
		# self.veh1_path = [(-368.2,209.8),(-342.6,197.1),(-334.4,197.1),(-317.9,189.1),(-314.5,183.3),(-280.1,166.3)]
		self.veh2_path = [[-288.7,144.1], [-285.20,152.10], [-281.60,159.20],[-278.90,163.50], [-275.70,164.40], [-243.60,148.80]]
		# self.veh3_path = [(-242.3,151.6),(-365.6,212.7)] # Safe Trajectory
		# self.veh3_path = [(-258.6,159.4),(-365.6,212.7)] # Near miss Trajectory
		# self.veh3_path = [(-279.0,169.4),(-365.6,212.7)] # Crash Trajectory
		# self.veh3_path = [(-302.0,181.4),(-365.6,212.7)] # Ending Interolation Trajectory
		if self.i == 0:
			# Creating several paths for contious runs
			veh1_path_series_start = [-310.30, 181.50] # Start of Trajectory series for vehicle with varying starting point
			veh1_path_series_end = [-286.10, 170.00] # End of Trajectory series for vehicle with varying starting point
			step = 2
			self.timeout_thresh = 10
			x = [veh1_path_series_start[0], veh1_path_series_end[0]]
			print("x[0] = ",x[0])
			print("x[1] = ",x[1])
			print("step = ",step)
			x_new = np.arange(x[0],x[1],step)
			y = [veh1_path_series_start[1], veh1_path_series_end[1]]
			self.y_new = np.interp(x_new, x, y)
			self.y_new = np.append(self.y_new,veh1_path_series_end[1])
			self.x_new = x_new #* -1
			self.x_new = np.append(self.x_new,veh1_path_series_end[0])

		
		self.veh1_path = [(self.x_new[self.i],self.y_new[self.i]),(-254.30, 153.70)] 
		self.i = self.i + 1
		if self.i == len(self.x_new):
			self.tests_ended = True
		print("i",self.i)
		print("self.x_new",self.x_new)
		print("length of x_new",len(self.x_new))
		

		# interpolate path(s)
		interpolation_resolution_min = 1
		self.veh1_path_interpolated  = interpolate_path(self.veh1_path, interpolation_resolution_min)
		self.veh2_path_interpolated  = interpolate_path(self.veh2_path, interpolation_resolution_min)
		# self.veh3_path_interpolated  = interpolate_path(self.veh3_path, interpolation_resolution_min)

		self.veh1.set_path(self.veh1_path_interpolated)
		self.veh2.set_path(self.veh2_path_interpolated)
		# self.veh3.set_path(self.veh3_path_interpolated)

		self.veh1.spawn(self.veh1_path[0][0],self.veh1_path[0][1], self.spawn_height, -25)
		self.veh2.spawn(self.veh2_path[0][0],self.veh2_path[0][1], self.spawn_height, 65)
		# self.veh3.spawn(self.veh3_path[0][0],self.veh3_path[0][1], self.spawn_height, 155)

		# self.veh1_speed = 5
		# self.veh2_speed = 0
		# self.veh3_speed = 5


	def step(self):
		self.veh1.step() 
		self.veh2.step() 
		# self.veh3.step()

		self.veh2.stop = False



		# Log 
		t   = self.world.get_snapshot().timestamp.elapsed_seconds
		fps = 1 / (self.world.get_snapshot().timestamp.frame_count)
		self.tests_logs.append(0,0,self.actors_list,t,fps)
		if self.tests_logs.time_array[-1] >= self.timeout_thresh:
			self.test_ended = True



		pass

	def destroy(self):
		self.veh1.destroy()
		self.veh2.destroy()
		# self.veh3.destroy()
		self.tests_logs.write_file("Left_Turn_At_Intersection_TestID"+str(self.i)+".txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_NearMiss.txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_Crash.txt")
		pass

# =================================================================================================================================================
#                                                                      Right turn at intersection, pedestrian crossing 
#                                                  (pedestrian replaced by vehicle due to map needs upadating to match carla udates)
# =================================================================================================================================================

class Assertions_case_study_right_turn_at_intersection_pedestrian_crossing():
	def __init__(self,world):
		self.spawn_height = 8
		self.tests_ended = False

		self.world = world

		self.logging_time_increment = 0.1
		self.tests_logs = log(self.logging_time_increment) 
		self.i = 0

	def set(self): 
		self.actors_list = []
		self.veh1 = vehicle(1) # vehicle turning right  
		self.ped1 = vehicle(2)#pedestrian(2,"adult") # pedestrian crossing zebra line
		self.ped1.vehicle_speed = 1.5
		self.actors_list.append(self.veh1)
		self.actors_list.append(self.ped1)
		self.test_ended = False
		self.tests_logs = log(self.logging_time_increment) 
		# spawn(self,x,y,z,yaw):
		# Setting Veh 1
		self.veh1_path = [[-284.20,153.90], [-279.50,162.90], [-278.40,166.50], [-279.80,170.10],[-323.50,192.20]]

		# Setting Ped 1
		if self.i == 0:
			# Creating several paths for contious runs
			ped1_path_series_start = [-283.90, 176.50] # Start of Trajectory series for pedestrian with varying starting point
			ped1_path_series_end = [-295.80,182.10] # End of Trajectory series for pedestrian with varying starting point
			step = 2
			self.timeout_thresh = 30
			x = [-ped1_path_series_start[0], -ped1_path_series_end[0]]
			print("x[0] = ",x[0])
			print("x[1] = ",x[1])
			print("step = ",step)
			x_new = np.arange(x[0],x[1],step)
			print("x_new = ",x_new)
			y = [ped1_path_series_start[1], ped1_path_series_end[1]]
			self.y_new = np.interp(x_new, x, y)
			self.y_new = np.append(self.y_new,ped1_path_series_end[1])
			self.x_new = x_new * -1
			self.x_new = np.append(self.x_new,ped1_path_series_end[0])

		
		self.ped1_path = [(self.x_new[self.i],self.y_new[self.i]),(-295.80,182.10), (-300.50,172.50), (-288.30,165.70)] 
		# self.ped1_path = [[-279.80,170.10], [-310.50,185.40]] 
		self.i = self.i + 1
		if self.i == len(self.x_new):
			self.tests_ended = True
		print("i",self.i)
		print("self.x_new",self.x_new)
		print("length of x_new",len(self.x_new))
		

		# interpolate path(s)
		interpolation_resolution_min = 1
		self.veh1_path_interpolated  = interpolate_path(self.veh1_path, interpolation_resolution_min)
		self.ped1_path_interpolated  = interpolate_path(self.ped1_path, interpolation_resolution_min)
		# self.veh3_path_interpolated  = interpolate_path(self.veh3_path, interpolation_resolution_min)

		self.veh1.set_path(self.veh1_path_interpolated)
		self.ped1.set_path(self.ped1_path_interpolated)
		# self.veh3.set_path(self.veh3_path_interpolated)

		self.veh1.spawn(self.veh1_path[0][0],self.veh1_path[0][1], self.spawn_height, 65)
		self.ped1.spawn(self.ped1_path[0][0],self.ped1_path[0][1], self.spawn_height, -115)
		# self.veh3.spawn(self.veh3_path[0][0],self.veh3_path[0][1], self.spawn_height, 155)

		# self.veh1_speed = 5
		# self.veh2_speed = 0
		# self.veh3_speed = 5


	def step(self):
		self.veh1.step() 
		self.ped1.step() 
		# self.veh3.step()



		# Log 
		t   = self.world.get_snapshot().timestamp.elapsed_seconds
		fps = 1 / (self.world.get_snapshot().timestamp.frame_count)
		self.tests_logs.append(0,0,self.actors_list,t,fps)
		if self.tests_logs.time_array[-1] >= self.timeout_thresh:
			self.test_ended = True



		pass

	def destroy(self):
		self.veh1.destroy()
		self.ped1.destroy()
		# self.veh3.destroy()
		self.tests_logs.write_file("Right_turn_at_intersection_pedestrian_crossing_TestID"+str(self.i)+".txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_NearMiss.txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_Crash.txt")
		pass