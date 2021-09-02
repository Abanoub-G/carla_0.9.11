import glob
import os
import sys
import numpy as np
import math

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Script level imports
sys.path.append('../../carla')

import logging
import carla 
import time

#=============================================
#===Connecting to Simulator
#=============================================
HOST = '127.0.0.1'
PORT = 2000

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

client = carla.Client(HOST, PORT)
client.set_timeout(2.0)

print("PROGRESS: Connecting to Simulator, please wait for me! Will notify you when I am done :)")

world = client.get_world()
# settings = world.get_settings()

# settings.fixed_delta_seconds = 0.05
# settings.synchronous_mode = False
# world.apply_settings(settings)

# @todo cannot import these directly.
SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor

print("PROGRESS: Connected to Simulator! :)")


#=============================================
#=== Classes and functions
#=============================================

def wraptopi(x):
	if x > np.pi:
		x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
	elif x < -np.pi:
		x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
	return x


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




class agent():
	def __init__(self,agentID,agentType,agentNo,agentTypeNo,current_x,current_y,current_z,current_yaw,current_velocity,current_speed):
		self.agentID          = agentID
		self.agentType        = agentType
		self.agentNo          = agentNo
		self.agentTypeNo      = agentTypeNo
		self.current_x        = current_x
		self.current_y        = current_y
		self.current_z        = current_z
		self.current_yaw      = current_yaw
		self.current_velocity = current_velocity
		self.current_speed    = current_speed




#=============================================
#=== Main
#=============================================

try:
	# set logging increment
	logging_time_increment = 0.1

	# setting time threshold for logging script to stop
	script_timeout_thresh = 40

	# initialise logging class
	tests_logs = log(logging_time_increment) 

	# start time of script
	start_time = time.time()

	while True:
		t   = world.get_snapshot().timestamp.elapsed_seconds
		fps = 1 / (world.get_snapshot().timestamp.frame_count)

		# get all actors 
		all_actors = world.get_actors()
		
		# create a list to append the agents of interest in the simulator 
		agents_list = []

		# filter the vehicles from actors and use extract their info to append to the agents list
		for vehicle in all_actors.filter('vehicle.mercedes-benz.coupe'):
			print(vehicle)
			agentID          = vehicle.id
			agentType        = vehicle.type_id
			agentNo          = vehicle.id
			agentTypeNo      = 3 
			current_x        = vehicle.get_transform().location.x
			current_y        = vehicle.get_transform().location.y
			current_z        = vehicle.get_transform().location.z
			current_yaw      = wraptopi(math.radians(vehicle.get_transform().rotation.yaw))
			agent_velocity   = vehicle.get_velocity() 
			current_velocity = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])
			current_speed    = np.sqrt(current_velocity.dot(current_velocity))

			agents_list.append(agent(agentID,agentType,agentNo,agentTypeNo,current_x,current_y,current_z,current_yaw,current_velocity,current_speed))

		# filter the pedestrians from actors and use extract their info to append to the agents list
		for pedestrian in all_actors.filter('walker.pedestrian.0002'):
			print(pedestrian)
			agentID          = pedestrian.id
			agentType        = pedestrian.type_id
			agentNo          = pedestrian.id
			agentTypeNo      = 1 
			current_x        = pedestrian.get_transform().location.x
			current_y        = pedestrian.get_transform().location.y
			current_z        = pedestrian.get_transform().location.z
			current_yaw      = wraptopi(math.radians(pedestrian.get_transform().rotation.yaw))
			agent_velocity   = pedestrian.get_velocity() 
			current_velocity = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])
			current_speed    = np.sqrt(current_velocity.dot(current_velocity))

			agents_list.append(agent(agentID,agentType,agentNo,agentTypeNo,current_x,current_y,current_z,current_yaw,current_velocity,current_speed))
		

		
		# append actors to logs
		tests_logs.append(0,0,agents_list,t,fps)

		# calcualte script time left
		end_time = time.time()
		diff_time = end_time - start_time

		# print script time left
		print("Script time =", diff_time ,"/", script_timeout_thresh )
		print("---")

		# break loop if script time has surpassed threshold
		if diff_time >= script_timeout_thresh:
			break
		




finally:
	# Save file: change name for new files or old files will be overwritten 
	tests_logs.write_file("RecreatedRun.txt")
	