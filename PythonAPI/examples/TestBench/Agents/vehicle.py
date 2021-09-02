import carla
from __init__ import client, world
from TB_common_functions import wraptopi, calculateDistance, AgentColourToRGB
# from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt
from Agents.controller import long_lat_controller
# from controller import long_lat_controller

class vehicle():
	def __init__(self,agentNo):
		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		self.vehicle_blueprint = self.blueprint_library.filter("vehicle.mercedes-benz.coupe")[0]
		self.vehicle_blueprint.set_attribute('color', AgentColourToRGB("yellow"))

		self.agentNo = agentNo
		self.agentTypeNo = 3

		self.STOPPING_DISTANCE = 5
		self.close_to_destination = False
		self.red_traffic_light    = False
		self.obstacle             = False

		self.stop  = False

		self.ignorePath = True

		self.vehicle_speed = 5 # Setting a default speed

		self.path_index = 0
		self.desired_speed = 0
		self.longitudinal_error_previous = 0
		self.previous_t = 0

		self.controller = long_lat_controller()

		# self.SpawnActor = carla.command.SpawnActor

	def spawn(self,x,y,z,yaw):
		self.actor_list = []
		self.spawn_orientation = yaw
		self.transform         = carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(self.spawn_orientation)))
		
		self.agent           = self.world.spawn_actor(self.vehicle_blueprint, self.transform) 
		self.world.tick() # important to tick after spawning, otherwise actor details not reachable
		self.actor_list.append(self.agent)

		self.spawn_point_x   = x 
		self.spawn_point_y   = y 
		self.spawn_point_z   = z 
		self.spawn_point_yaw = yaw

		self.agentID    = self.agent.id
		self.agentType  = self.agent.type_id
		pass

	def set_path(self,path_coord_tuple):

		self.path = path_coord_tuple  # [x,y]

		print("Vehicle path set!")

	# def set_destination(self,x,y,z,occupancy_grid,map_x_coord,map_y_coord):
	# 	# Note: if you want to the vehicle to use A* algorithim to path plan you need 
	# 	# to provide an occupancy grid and the corresponsing x and y coordinates for 
	# 	# each square in the occupancy grid by filling map_x_coord and map_y_coord 
	# 	# matrices respectively. If you just want the vehicle to follow a predefined 
	# 	# path, use the function set_path instead.  
		
	# 	self.occupancy_grid = occupancy_grid  
	# 	self.map_x_coord    = map_x_coord
	# 	self.map_y_coord    = map_y_coord

	# 	self.dest_x = x 
	# 	self.dest_y = y 
	# 	self.dest_z = z 

	# 	self.PathPlan()

	# 	print("Vehicle destination set!")
		
	def step(self):
		self.world_snapshot = self.world.get_snapshot()

		self.Update_state_info()

		if self.stop == True:
			self.send_control(0, 0, 1, hand_brake=False, reverse=False)
		else:
			# self.controller()
			throttle, brake, steer = self.controller.step(self.path, self.vehicle_speed, self.current_x, self.current_y, self.current_yaw, self.current_speed)

			# throttle_output      = np.fmax(np.fmin(throttle, 1.0), 0)
			# brake_output         = np.fmax(np.fmin(brake, 1.0), 0)
			# steer_output         = np.fmax(np.fmin(steer, 1.0), -1.0)

			self.send_control(throttle, steer, brake, hand_brake=False, reverse=False)
		
		pass

	def set_speed(self,speed):
		self.vehicle_speed = speed
		pass

	def Update_state_info(self):
		agent_transform                 = self.agent.get_transform()
		agent_location                  = agent_transform.location  
		agent_rotation                  = agent_transform.rotation
		agent_velocity                  = self.agent.get_velocity()      # This is an object vector

		self.current_t                  = self.world_snapshot.timestamp.elapsed_seconds #ts.elapsed_seconds - start_of_simulation_timestamp
		self.current_x                  = agent_location.x
		self.current_y                  = agent_location.y
		self.current_z                  = agent_location.z
		self.current_yaw                = wraptopi(math.radians(agent_rotation.yaw))
		self.current_velocity                = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])      # This is an array as opposed to agent_velocity, which is an object
		self.current_speed              = np.sqrt(self.current_velocity.dot(self.current_velocity))


	def get_pos(self):
		self.Update_state_info()
		self.pos_array = [self.current_x, self.current_y, self.current_z, self.current_yaw]

		return self.pos_array

	def destroy(self):
		self.agent.destroy()
		pass

	# def PathPlan(self):
	# 	spawn_pos = [self.spawn_point_x,self.spawn_point_y]
	# 	destination_pos = [self.dest_x,self.dest_y] 
	# 	want_to_plot_path = False
	# 	want_to_plot = False
	# 	plot_every       = 10
		
	# 	xStart = spawn_pos_idx = find_nearest(self.map_x_coord, spawn_pos[0])[0][0]
	# 	yStart = spawn_pos_idy = find_nearest(self.map_y_coord, spawn_pos[1])[0][1]

	# 	xEnd = destination_pos_idx = find_nearest(self.map_x_coord, destination_pos[0])[0][0]
	# 	yEnd = destination_pos_idy =find_nearest(self.map_y_coord, destination_pos[1])[0][1]

	# 	start = (xStart, yStart)
	# 	end = (xEnd, yEnd)
	# 	cost = 1 # cost per movement

	# 	# Plot_map(AV_pos, Target_pos, map_objects, map_x_coord, map_y_coord)

	# 	path, path_mat = astar_search(start, end, cost, spawn_pos, destination_pos, self.occupancy_grid, self.map_x_coord, self.map_y_coord, want_to_plot,plot_every)

	# 	path_coord = []
	# 	for i in range(len(path)):
	# 		coord_tuple = (self.map_x_coord[path[i]],self.map_y_coord[path[i]])
	# 		path_coord.append(coord_tuple)
	# 	self.path = path_coord
	# 	self.path_shapely_line = LineString(self.path)
	# 	print(self.path_shapely_line)

	# 	if want_to_plot_path:
	# 		self.Plot_path(spawn_pos, destination_pos, self.occupancy_grid, self.map_x_coord, self.map_y_coord, path_coord)

	def send_control(self, throttle, steer, brake, hand_brake=False, reverse=False):
		self.control = carla.VehicleControl()

		# Clamp all values within their limits
		steer                    = np.fmax(np.fmin(steer, 1.0), -1.0)
		throttle                 = np.fmax(np.fmin(throttle, 1.0), -1.0)
		brake                    = np.fmax(np.fmin(brake, 1.0), -1.0)

		self.control.steer       = steer
		self.control.throttle    = throttle
		self.control.brake       = brake
		self.control.hand_brake  = hand_brake
		self.control.reverse     = reverse

		self.agent.apply_control(self.control)


	def Plot_path(self,AV_pos, Target_pos, map_objects, map_x_coord, map_y_coord, path_coord):
		for i in np.unique(map_objects):
			map_objects2 = np.array(map_objects)
			result = np.where(map_objects2 == i)
			x = map_x_coord[result[0],result[1]]
			y = map_y_coord[result[0],result[1]]

			if i == 1:
				chosenColour = "cornflowerblue"
				chosenLabel  = "North"
			elif i == 2:
				chosenColour = "royalblue"
				chosenLabel  = "South"
			elif i == 3:
				chosenColour = "darkmagenta"
				chosenLabel  = "East"
			elif i == 4:
				chosenColour = "magenta"
				chosenLabel  = "West"
			elif i == 5:
				chosenColour = "red"
				chosenLabel  = "North-East"
			elif i == 6:
				chosenColour = "pink"
				chosenLabel  = "North-West"
			elif i == 7:
				chosenColour = "green"
				chosenLabel  = "South-East"
			elif i == 8:
				chosenColour = "lime"
				chosenLabel  = "South-West"
			elif i == 9:
				chosenColour = "darkgoldenrod"
				chosenLabel  = "All Directions"
			elif i == 10:
				chosenColour = "lightgrey"
				chosenLabel  = "Curb"
			else:
				chosenColour = "whitesmoke"
				chosenLabel  = "Obstacle"

			plt.scatter(x, y, s=5, marker = "s", color = chosenColour,label=chosenLabel)
			
		x_val = [x[0] for x in path_coord]
		y_val = [x[1] for x in path_coord]
		
		plt.scatter(AV_pos[0],AV_pos[1], s=20, marker='x', color='black',label="Start")
		plt.scatter(Target_pos[0],Target_pos[1], s=20, marker='x', color='red',label="End")
		plt.plot(x_val,y_val,markersize=4,linestyle="--",color='white',label="path")
		plt.legend()
		# plt.draw()
		# plt.pause(3)
		plt.show()
