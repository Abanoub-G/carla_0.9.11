import os
import math
import numpy as np
import random
from shapely.geometry import Point
from shapely.geometry import Polygon

def Ticking(world,frame):
	# Tick
	world.tick()
	# Get world snapshot   
	world_snapshot = world.get_snapshot()
	ts             = world_snapshot.timestamp
	if frame is not None:
		if ts.frame_count != frame + 1:
			print('frame skip!')
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

class log():
	def __init__(self, logging_time_increment):
		self.testNo_array         = []
		self.repeatNo_array       = []
		self.agentNo_array        = []
		self.agentID_array        = []
		self.agentType_array      = []
		self.agentTypeNo_array    = []
		self.x_array              = []
		self.y_array              = []
		self.z_array              = []
		self.yaw_array            = []
		self.vel_x_array          = []
		self.vel_y_array          = []
		self.vel_z_array          = []
		self.speed_array          = []
		self.time_array           = []
		self.sim_time_array       = []
		self.fps_array            = []
		

		self.logging_time_increment = logging_time_increment

	def append(self,testNo,repeatNo,agents_list,sim_time,fps):

		# Check if new test
		if (len(self.time_array) == 0) or (testNo > self.testNo_array[-1]) or (repeatNo != self.repeatNo_array[-1]):
			new_test = True
		else:
			new_test = False
		
		if new_test:
			diff = np.inf
			time = 0
			new_test = False
		else:
			# Calculate test time from sim time
			diff =  sim_time - self.sim_time_array[-1]
			time =  self.time_array[-1] + diff
			 


		if diff >= self.logging_time_increment:
			for agent in agents_list:
				self.testNo_array.append(testNo)
				self.repeatNo_array.append(repeatNo)
				self.time_array.append(time)
				self.sim_time_array.append(sim_time) 
				self.fps_array.append(fps)
				try:
					self.agentID_array.append(agent.agentID)
					self.agentType_array.append(agent.agentType)
					self.agentNo_array.append(agent.agentNo)
					self.agentTypeNo_array.append(agent.agentTypeNo)
					self.x_array.append(agent.current_x)
					self.y_array.append(agent.current_y)
					self.z_array.append(agent.current_z)
					self.yaw_array.append(agent.current_yaw)
					self.vel_x_array.append(agent.current_velocity[0])
					self.vel_y_array.append(agent.current_velocity[1])
					self.vel_z_array.append(agent.current_velocity[2])
					self.speed_array.append(agent.current_speed)
				except:
					# if doesn't have these then it is probably the AV
					self.agentID_array.append(agent.id)
					self.agentType_array.append(agent.type_id)
					self.agentNo_array.append(0)
					self.agentTypeNo_array.append(0)
					self.x_array.append(agent.get_transform().location.x)
					self.y_array.append(agent.get_transform().location.y)
					self.z_array.append(agent.get_transform().location.z)
					self.yaw_array.append(agent.get_transform().rotation.yaw)
					self.vel_x_array.append(agent.get_velocity().x)
					self.vel_y_array.append(agent.get_velocity().y)
					self.vel_z_array.append(agent.get_velocity().z)
					current_velocity = np.array([agent.get_velocity().x, agent.get_velocity().y, agent.get_velocity().z]) 
					self.speed_array.append(np.sqrt(current_velocity.dot(current_velocity)))
					
					# print("AV_x = ",agent.get_transform().location.x)
					# print("AV_y = ",agent.get_transform().location.y)
					
				


	def write_file(self, file_name):
		# Folder "results" if not already there
		output_folder = "tests_logs"
		if not os.path.exists(output_folder):
			os.makedirs(output_folder)

		file_path = os.path.join(output_folder, file_name)
		with open(file_path, 'w') as log_file: 
			log_file.write('testNo ,repeatNo, agentNo, agentID, agentType, agentTypeNo, x, y, z, yaw, vel_x, vel_y, vel_z, speed, time, sim_time, fps\n')
			for i in range(len(self.testNo_array)):
				log_file.write('%d, %d, %d, %d, %s, %d, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f\n' %\
					(self.testNo_array[i],self.repeatNo_array[i], self.agentNo_array[i], self.agentID_array[i], self.agentType_array[i],\
					self.agentTypeNo_array[i], self.x_array[i], self.y_array[i], self.z_array[i], self.yaw_array[i],self.vel_x_array[i],self.vel_y_array[i],self.vel_z_array[i], self.speed_array[i], self.time_array[i], self.sim_time_array[i], self.fps_array[i]))
		print('Log file SUCCESSFULLY generated!')

