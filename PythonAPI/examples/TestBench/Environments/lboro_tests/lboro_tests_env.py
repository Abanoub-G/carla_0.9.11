import numpy as np
import pandas as pd
from shapely.geometry import Polygon
from shapely import affinity
import requests 
import json
import time
from .loading_tests import get_hazard, create_grid
import subprocess
import os
import sys
import glob

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

sys.path.append('../../')
from TB_common_functions import log, calculateDistance
from Agents.pedestrian import pedestrian
from Agents.vehicle import vehicle





class static_and_dyanmic_tests():
	def __init__(self,world,static_dynamic_flag):

		self.world = world
		self.static_dynamic_flag = static_dynamic_flag

		file_name =  "lboro_tests/Capri_tests.xlsx"
		sheet     =  "Static hazard tests"

		self.df                = pd.read_excel(io=file_name, sheet_name=sheet)
		self.total_No_tests    = len(self.df)
		self.total_No_repeats  = 1

		# Insert AV waypoints
		self.AV_x_list_orig = [0.45,1.45]           # for straight path
		self.AV_y_list_orig = [-37.59,-83.09]       # for straight path  
		self.AV_x_list_orig = [0.45,0.95,-4.05]      # for curved path 
		self.AV_y_list_orig = [-37.59,-60.0,-65.0]   # for curved path 

		
		self.AV_pos          = [self.AV_x_list_orig[0],self.AV_y_list_orig[0]]
		self.AV_yaw          = -np.pi/2
		self.AV_yaw_deg      = self.AV_yaw * 180/np.pi
		# self.AV_spawn_location = [self.AV_pos[0],self.AV_pos[1],0.5,self.AV_yaw]
		self.AV_path_file_name  = '"path.csv"'
		self.AV_length       = 3.667
		self.AV_width        = 1.605
		dim                  = np.array([self.AV_width,self.AV_length]) 
		self.AV_pol_origin   = Polygon([[-dim[0]/2, dim[1]/2],\
						[dim[0]/2, dim[1]/2],\
						[dim[0]/2, -dim[1]/2],\
						[-dim[0]/2,-dim[1]/2],\
						[-dim[0]/2, dim[1]/2]])

		# Set variables
		self.testNo   = 1
		self.repeatNo = 1
		self.tests_ended = False
		self.set_test_flag = True
		logging_time_increment = 0.1
		self.spawn_height = 1

		# Set tests logging
		self.tests_logs = log(logging_time_increment) 


		# Communication variables with AV
		self.URL_start = 'http://localhost:5000/start_sim'
		self.URL_end = 'http://localhost:5000/terminate_sim'
		self.URL_data = 'http://localhost:5000/sim_data'
		self.headers = {'Content-type': 'application/json',}
		self.dissable_AV_sensors = 0
		self.data = '{"simulation_id": "SIM101", \
		 "gps_log_file": %s, \
		 "spawn_location": [%3.3f,%3.3f,%3.3f,%3.3f],\
		 "dissable_sensors": 1}'%(self.AV_path_file_name,self.AV_pos[0],self.AV_pos[1],0.5,self.AV_yaw_deg)
		 # "dissable_sensor_laser_front": 1,\
		 # "dissable_sensor_laser_left": 1,\
		 # "dissable_sensor_laser_right": 1,\
		 # "dissable_sensor_laser_rear": 1,\
		 # "dissable_sensor_radar_front": 1,\
		 # "dissable_sensor_radar_rear": 1}'%(self.AV_path_file_name,self.AV_pos[0],self.AV_pos[1],0.5,self.AV_yaw_deg)


		self.grid_origin =  np.array([0.95, -60])
		self.grid_orientation = -np.pi/2 +0.01

	def generate_AV_waypoints(self):

		z      = -0.057
		num_points = 200#20
		offset_from_rect_grid = -5

		# Find wich path to generate waypoints for.
		if self.pod_manoeuvre == "Turning left":
			self.AV_x_list_orig = [0.45,0.95,-4.05]      # for curved path 
			self.AV_y_list_orig = [-37.59,-60.0,-65.0]   # for curved path 
			x_list = np.linspace(self.AV_x_list_orig[0], self.AV_x_list_orig[1], num=num_points)  # for curved path
			y_list = np.interp(x_list, self.AV_x_list_orig[0:2], self.AV_y_list_orig[0:2])        # for curved path

			x_list2 = np.linspace(self.AV_x_list_orig[1], self.AV_x_list_orig[2], num=num_points)  # for curved path
			for x in x_list2:
				y = -60-np.sqrt(offset_from_rect_grid**2-(x+4.05)**2)
				y_list = np.append(y_list,y)
				x_list = np.append(x_list,x)

		else:
			self.AV_x_list_orig = [0.45,1.45]           # for straight path
			self.AV_y_list_orig = [-37.59,-83.09]       # for straight path  
			x_list = np.linspace(self.AV_x_list_orig[0], self.AV_x_list_orig[-1], num=num_points) # for straight path
			y_list = np.interp(x_list, self.AV_x_list_orig, self.AV_y_list_orig)                  # for straight path




		lat_list = []
		lon_list = []

		# Convert to geo coordinates
		current_map = self.world.get_map()

		for i in range(len(x_list)):
			geolocation = current_map.transform_to_geolocation(carla.Location(x=float(x_list[i]), y=float(y_list[i]), z=float(z)))

			lat_list.append(geolocation.latitude)
			lon_list.append(geolocation.longitude)

		# Write file
		file_name = "path.csv"
		file_path = os.path.join(file_name)
		with open(file_path, 'w') as log_file: 
			for i in range(len(lat_list)):
				log_file.write('%3.14f, %3.14f\n' %\
					(lat_list[i],lon_list[i]))
		print('AV Waypoints file SUCCESSFULLY generated!')


	
	def set_static_hazard(self,hazard_pos,hazard_orientation,hazardNo,hazard_type):
		if hazard_type == "adult" or hazard_type == "kid":
			agent = pedestrian(hazardNo,hazard_type)
			agent.spawn(hazard_pos[0],hazard_pos[1], self.spawn_height, hazard_orientation)
			ped_speed       = 0
			agent.set_speed(ped_speed)

		if hazard_type == "pushchair":
			pass
		if hazard_type == "trunki":
			pass
		if hazard_type == "bicycle":
			pass
		print("HAZARD SET! :-)")

		return agent


	def set_static_test(self,testNo): 
		# Get test details
		self.testNo = 1#105#105#1#105#132#111#105#
		test_details = self.df[self.df.TestNo  == self.testNo]
		print("TestNo = ",self.testNo)
		hazard_pos_on_grid = test_details["Hazard specific location"].values
		print("hazard_pos_on_grid = ",hazard_pos_on_grid)

		self.pod_manoeuvre = test_details["Pod manoeuvre"].values[0]
		# print("pod_manoeuvre = ",pod_manoeuvre)
		self.draw_test_grid(self.pod_manoeuvre)

		# Get and set hazard 
		hazard_pos,hazard_orientation,hazard_type = get_hazard(test_details,self.pod_manoeuvre,self.grid_origin,self.grid_orientation)
		hazardNo = 1
		print("agent hazard_pos", hazard_pos)
		self.agent = self.set_static_hazard(hazard_pos,hazard_orientation,hazardNo,hazard_type)
		# self.world.debug.draw_string(carla.Location(x=float(hazard_pos[0]), y=float(hazard_pos[1]), z=float(self.spawn_height/2)), 'O', draw_shadow=False,
		# 		color=carla.Color(r=0, g=255, b=0), life_time=120.0,persistent_lines=True)

		# Generate waypoints for AV
		self.generate_AV_waypoints()
		
		self.AV    = self.spawn_AV()



		# Create actors list (including AV)
		self.actors_list = []
		self.actors_list.append(self.agent)
		self.actors_list.append(self.AV)
		# print("hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
		# print(self.world.get_actors().filter('vehicle.*'))
		# self.actors_list.append(self.AV)
		

	def spawn_AV(self):
		# Establish connection to AV and spawn
		self.command1 = subprocess.Popen(['python3', 'ACSsim_4.1.pyc', '2'])
		# print("FIRST WAIT")
		time.sleep(5)
		
		response = requests.post(url = self.URL_start, headers = self.headers, data = self.data)
		# self.command2 = subprocess.Popen(['python3', 'test_ACS.py'])
		# print("SECOND WAIT")
		# time.sleep(5)
		
		while True: 
			# print("UoB waiting")
			# response = requests.post(url = self.URL_start, headers = self.headers, data = self.data)
			self.world.tick()# important to tick after spawning, otherwise actor details not reachable
			vehicles_list = self.world.get_actors().filter('vehicle.*')
			# everything_list = self.world.get_actors()
			
			# print(vehicles_list)#self.world.get_actors().filter('vehicle.*'))
			if len(vehicles_list) >= 1:
				AV = vehicles_list[0]
				# everything_list = self.world.get_actors()
				# print(everything_list)
				break

		return AV

	def destroy_AV(self):
		sensors_list = self.world.get_actors().filter('sensor.*')
		for sensor in sensors_list:
			sensor.destroy()
		self.AV.destroy()


			
		# 	print("TICK = ",i)
		# 	print(self.world.get_actors())
			



	def step(self):
		if self.set_test_flag:
			if self.static_dynamic_flag == "static":
				self.set_static_test(self.testNo)
			elif self.static_dynamic_flag == "dynamic":
				self.set_dynamic_test(self.testNo)
			self.set_test_flag = False
			self.timeout = time.time() + 3   # 3 sec from now

		# Step actors or update info (if applicable)
		# print(self.world.get_actors().filter('vehicle.*'))
		self.agent.Update_state_info()
		# pos = self.agent.get_pos()
		# print("hazard_pos = ",hazard_pos)
		# print("pos = ",pos) 

		# Log 
		t   = self.world.get_snapshot().timestamp.elapsed_seconds
		fps = 1 / (self.world.get_snapshot().timestamp.frame_count)
		self.tests_logs.append(self.testNo,self.repeatNo,self.actors_list,t,fps)

		AV_current_velocity = np.array([self.AV.get_velocity().x, self.AV.get_velocity().y, self.AV.get_velocity().z]) 
		AV_speed = np.sqrt(AV_current_velocity.dot(AV_current_velocity))
		# print("logs = ",self.tests_logs)

		# Calculate distance of AV from destination
		AV_dist_to_destination = calculateDistance(self.AV.get_transform().location.x,self.AV.get_transform().location.y,self.AV_x_list_orig[-1],self.AV_y_list_orig[-1])

		# Calculate distance of AV from spawn point
		AV_dist_from_spawn = calculateDistance(self.AV.get_transform().location.x,self.AV.get_transform().location.y,self.AV_x_list_orig[0],self.AV_y_list_orig[0])


		
		# # Check if AV is close to destination i.e. arrived
		# print("AV_dist_to_destination",AV_dist_to_destination)
		# print("AV_dist_from_spawn",AV_dist_from_spawn)
		# print("AV_speed",AV_speed)
		# time.time() > self.timeout
		# print("AV_speed = ",AV_speed)
		if (AV_dist_to_destination <= 1) or (AV_dist_from_spawn >= 5 and AV_speed == 0.0):

			print("I'm in 1")
			
			print("I'm in 2")
			# Check if all repeats ended 
			if self.repeatNo >= self.total_No_repeats:
				# Check if all tests ended 
				if self.testNo >= self.total_No_tests:
					print("I'm in end")
					self.tests_ended = True
					# self.end_tests()
					return 
				else:
					print("End of test no = ",self.testNo)
					# Destroy actors 
					self.destroy_actors()
				self.testNo  += 1
				self.repeatNo = 1

			else:
				self.repeatNo += 1

			self.set_test_flag = True 

		# except:
		# 	print("Test no %d is lacking something, please check the hazard type is available",%self.testNo)
		# 	self.testNo  += 1
		# 	self.set_test_flag = True





	def destroy_actors(self):
		self.agent.destroy()
		# response = requests.post(url = self.URL_end)#, headers = self.headers, data = self.data)
		
		self.command1.kill()
		self.destroy_AV()
		# self.AV.destroy()
		
		# while True:
		# 	self.world.tick()

		# for i in range(100): 
		# 	self.world.tick()# important to tick after spawning, otherwise actor details not reachable
		# 	vehicles_list = self.world.get_actors().filter('vehicle.*')
		# 	# print(vehicles_list)#self.world.get_actors().filter('vehicle.*'))
		# 	if len(vehicles_list) < 1:
		# 		return

	def end_tests(self):
		self.tests_logs.write_file("lboro_static_tests.txt")
		self.destroy_actors()

	def draw_test_grid(self,pod_manoeuvre):
		grid_life_time = 50 # in seconds
		draw_height    = self.spawn_height/100
		if pod_manoeuvre == "Turning left":
			draw_rectangular_grid = False
			draw_polar_grid       = True
		else:
			draw_rectangular_grid = True
			draw_polar_grid       = False

		#====================================
		# Rectangular grid
		#====================================
		# draw_rectangular_grid = False
		# draw_polar_grid       = True
		if draw_rectangular_grid:
			l0 = 0.4
			l1 = 1
			unit_vector      = np.array([np.cos(self.grid_orientation),np.sin(self.grid_orientation)])
			unit_vector_orth = np.array([np.cos(self.grid_orientation+np.pi/2),np.sin(self.grid_orientation+np.pi/2)])
			
			# Centre line
			p_start          = self.grid_origin
			p_end            = l1*5*unit_vector + self.grid_origin
			begin = carla.Location(x=float(p_start[0]), y=float(p_start[1]), z=float(draw_height))
			end   = carla.Location(x=float(p_end[0]), y=float(p_end[1]), z=float(draw_height))
			self.world.debug.draw_line(begin, end, thickness=0.05, color=carla.Color(r=255, g=0, b=0), life_time=grid_life_time)

			# Drawing vertical lines
			for i in range(5):
				for k in [1,-1]:
					if i <= 2:
						p_start          = unit_vector_orth*l0*i*k + self.grid_origin
						p_end            = unit_vector_orth*l0*i*k + (l1*5*unit_vector + self.grid_origin)
					else:
						p_start          = unit_vector_orth*l1*(i-2)*k + unit_vector_orth*l0*2*k + self.grid_origin
						p_end            = unit_vector_orth*l1*(i-2)*k + unit_vector_orth*l0*2*k + (l1*5*unit_vector + self.grid_origin)


					begin = carla.Location(x=float(p_start[0]), y=float(p_start[1]), z=float(draw_height))
					end   = carla.Location(x=float(p_end[0]), y=float(p_end[1]), z=float(draw_height))
					self.world.debug.draw_line(begin, end, thickness=0.05, color=carla.Color(r=5, g=0, b=0,a=255), life_time=grid_life_time)

			# Drawing horizontal lines
			for i in range(6):
				p_start          = unit_vector*l1*i + (self.grid_origin + (l0*2 + l1*2)*unit_vector_orth)
				p_end            = unit_vector*l1*i + (self.grid_origin - (l0*2 + l1*2)*unit_vector_orth)


				begin = carla.Location(x=float(p_start[0]), y=float(p_start[1]), z=float(draw_height))
				end   = carla.Location(x=float(p_end[0]), y=float(p_end[1]), z=float(draw_height))
				self.world.debug.draw_line(begin, end, thickness=0.05, color=carla.Color(r=5, g=0, b=0, a=255), life_time=grid_life_time)

		# if pod_manoeuvre == "Turning left":
			#====================================
			# Polar grid 
			#====================================
			# polar_grid_origin = self.grid_origin
			# l0 = 0.4
			# l1 = 1
			# unit_vector      = np.array([np.cos(self.grid_orientation),np.sin(self.grid_orientation)])
			# unit_vector_orth = np.array([np.cos(self.grid_orientation+np.pi/2),np.sin(self.grid_orientation+np.pi/2)])
		if draw_polar_grid:
			polar_grid_len = 7.8
			angle_inc = np.pi/14
			grid_angle_start = 0
			grid_angle_end   = np.pi/2
			for angle in np.arange(grid_angle_start,grid_angle_end, angle_inc):
				angle = angle + self.grid_orientation
				
				# plotting the lines
				offset_from_rect_grid = -5#0#5 #cm
				begin = [offset_from_rect_grid,0] + self.grid_origin
				begin = carla.Location(x=float(begin[0]), y=float(begin[1]), z=float(draw_height))
				end   = [polar_grid_len*np.cos(angle)+offset_from_rect_grid,polar_grid_len*np.sin(angle)] + self.grid_origin
				end   = carla.Location(x=float(end[0]), y=float(end[1]), z=float(draw_height))
				self.world.debug.draw_line(begin, end, thickness=0.05, color=carla.Color(r=5, g=0, b=0,a=255), life_time=grid_life_time)
				# plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='red');

				# plotting the circles
				for r in [2.2,3.2,4.2,5,5.8,6.8,7.8]:
					begin = [r*np.cos(angle)+offset_from_rect_grid,r*np.sin(angle)] + self.grid_origin
					begin = carla.Location(x=float(begin[0]), y=float(begin[1]), z=float(draw_height))
					end   = [r*np.cos(angle+angle_inc)+offset_from_rect_grid,r*np.sin(angle+angle_inc)] + self.grid_origin
					end   = carla.Location(x=float(end[0]), y=float(end[1]), z=float(draw_height))
					self.world.debug.draw_line(begin, end, thickness=0.05, color=carla.Color(r=5, g=0, b=0,a=255), life_time=grid_life_time)
					# plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='red');

			begin = [offset_from_rect_grid,0] + self.grid_origin
			begin = carla.Location(x=float(begin[0]), y=float(begin[1]), z=float(draw_height))
			end   = [polar_grid_len*np.cos(angle+angle_inc)+offset_from_rect_grid,polar_grid_len*np.sin(angle+angle_inc)] + self.grid_origin
			end   = carla.Location(x=float(end[0]), y=float(end[1]), z=float(draw_height))
			self.world.debug.draw_line(begin, end, thickness=0.05, color=carla.Color(r=5, g=0, b=0,a=255), life_time=grid_life_time)
			# plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='red');
						


