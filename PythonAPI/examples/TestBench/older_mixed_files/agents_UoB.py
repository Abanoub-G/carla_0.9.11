import carla
from __init__ import client, world
from environment_setup import wraptopi, calculateDistance, AgentColourToRGB
from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt


class pedestrian():
	def __init__(self):
		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		self.ped_blueprint     = self.blueprint_library.filter("walker.pedestrian.0001")[0]
		self.ped_blueprint.set_attribute('is_invincible', 'false')
		self.ped_controller_blueprint = self.blueprint_library.find('controller.ai.walker')

		self.SpawnActor        = carla.command.SpawnActor
		self.batch_W           = []
		self.batch_W_C         = []

		self.stop  = False

	def spawn(self,x,y,z,yaw):
		self.spawn_orientation = yaw
		self.transform         = carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(self.spawn_orientation)))
		
		self.batch_W.append(self.SpawnActor(self.ped_blueprint, self.transform)) #Note I am using batch because regular comand not working
		self.results_p = self.client.apply_batch_sync(self.batch_W, True)
		self.ped_id    = self.results_p[0].actor_id
		self.world.tick()# important to tick after spawning, otherwise actor details not reachable
		actors         = self.world.get_actors()
		self.ped       = actors.find(self.ped_id)


		self.batch_W_C.append(self.SpawnActor(self.ped_controller_blueprint , carla.Transform(), self.ped)) #Note I am using batch because regular comand not working
		self.results_c          = self.client.apply_batch_sync(self.batch_W_C, True)
		self.ped_controller_id  = self.results_c[0].actor_id
		self.world.tick() # important to tick after spawning, otherwise actor details not reachable
		actors                  = self.world.get_actors()
		self.ped_controller     = actors.find(self.ped_controller_id)
	
		self.ped_controller.start()
		

		self.agentID    = self.ped.id
		self.agentType  = self.ped.type_id

	def set_destination(self,x,y,z):
		self.dest_x = x 
		self.dest_y = y 
		self.dest_z = z 
		
	def step(self):
		self.world_snapshot = self.world.get_snapshot()

		self.Update_state_info()

		if self.stop == True:
			self.ped_controller.go_to_location(carla.Location(x=float(self.current_x), y=float(self.current_y), z=float(self.current_z)))
		else:
			self.ped_controller.go_to_location(carla.Location(x=float(self.dest_x), y=float(self.dest_y), z=float(self.dest_z)))
		

	def set_speed(self,speed):
		self.ped_controller.set_max_speed(speed)

	def Update_state_info(self):
		agent_transform                 = self.ped.get_transform()
		agent_location                  = agent_transform.location  
		agent_rotation                  = agent_transform.rotation
		agent_velocity                  = self.ped.get_velocity()      # This is an object vector

		self.current_t                  = self.world_snapshot.timestamp.elapsed_seconds #ts.elapsed_seconds - start_of_simulation_timestamp
		self.current_x                  = agent_location.x
		self.current_y                  = agent_location.y
		self.current_z                  = agent_location.z
		self.current_yaw                = wraptopi(math.radians(agent_rotation.yaw))
		current_velocity                = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])      # This is an array as opposed to agent_velocity, which is an object
		self.current_speed              = np.sqrt(current_velocity.dot(current_velocity))


	def get_pos(self):
		self.Update_state_info()
		self.pos_array = [self.current_x, self.current_y, self.current_z, self.current_yaw]

		return self.pos_array

	def destroy(self):
		self.ped_controller.stop()
		self.ped_controller.destroy()
		self.ped.destroy()

	





class vehicle():
	def __init__(self,occupancy_grid,map_x_coord,map_y_coord):
		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		self.vehicle_blueprint = self.blueprint_library.filter("vehicle.mercedes-benz.coupe")[0]
		self.vehicle_blueprint.set_attribute('color', AgentColourToRGB("yellow"))
		
		self.occupancy_grid = occupancy_grid  
		self.map_x_coord    = map_x_coord
		self.map_y_coord    = map_y_coord

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

		# self.SpawnActor = carla.command.SpawnActor

	def spawn(self,x,y,z,yaw):
		self.actor_list = []
		self.spawn_orientation = yaw
		self.transform         = carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(self.spawn_orientation)))
		
		self.vehicle           = self.world.spawn_actor(self.vehicle_blueprint, self.transform) 
		self.world.tick() # important to tick after spawning, otherwise actor details not reachable
		self.actor_list.append(self.vehicle)

		self.spawn_point_x   = x 
		self.spawn_point_y   = y 
		self.spawn_point_z   = z 
		self.spawn_point_yaw = yaw

		self.agentID    = self.vehicle.id
		self.agentType  = self.vehicle.type_id
		pass

	def set_destination(self,x,y,z):
		self.dest_x = x 
		self.dest_y = y 
		self.dest_z = z 

		self.PathPlan()

		print("Vehicle destination set!")
		
	def step(self):
		self.world_snapshot = self.world.get_snapshot()

		self.Update_state_info()

		if self.stop == True:
			self.send_control(0, 0, 1, hand_brake=False, reverse=False)
		else:
			self.controller()
		
		pass

	def set_speed(self,speed):
		self.vehicle_speed = speed
		pass

	def Update_state_info(self):
		agent_transform                 = self.vehicle.get_transform()
		agent_location                  = agent_transform.location  
		agent_rotation                  = agent_transform.rotation
		agent_velocity                  = self.vehicle.get_velocity()      # This is an object vector

		self.current_t                  = self.world_snapshot.timestamp.elapsed_seconds #ts.elapsed_seconds - start_of_simulation_timestamp
		self.current_x                  = agent_location.x
		self.current_y                  = agent_location.y
		self.current_z                  = agent_location.z
		self.current_yaw                = wraptopi(math.radians(agent_rotation.yaw))
		current_velocity                = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])      # This is an array as opposed to agent_velocity, which is an object
		self.current_speed              = np.sqrt(current_velocity.dot(current_velocity))


	def get_pos(self):
		self.Update_state_info()
		self.pos_array = [self.current_x, self.current_y, self.current_z, self.current_yaw]

		return self.pos_array

	def destroy(self):
		self.vehicle.destroy()
		pass

	def PathPlan(self):
		spawn_pos = [self.spawn_point_x,self.spawn_point_y]
		destination_pos = [self.dest_x,self.dest_y] 
		want_to_plot_path = False
		want_to_plot = False
		plot_every       = 10
		
		xStart = spawn_pos_idx = find_nearest(self.map_x_coord, spawn_pos[0])[0][0]
		yStart = spawn_pos_idy = find_nearest(self.map_y_coord, spawn_pos[1])[0][1]

		xEnd = destination_pos_idx = find_nearest(self.map_x_coord, destination_pos[0])[0][0]
		yEnd = destination_pos_idy =find_nearest(self.map_y_coord, destination_pos[1])[0][1]

		start = (xStart, yStart)
		end = (xEnd, yEnd)
		cost = 1 # cost per movement

		# Plot_map(AV_pos, Target_pos, map_objects, map_x_coord, map_y_coord)

		path, path_mat = astar_search(start, end, cost, spawn_pos, destination_pos, self.occupancy_grid, self.map_x_coord, self.map_y_coord, want_to_plot,plot_every)

		path_coord = []
		for i in range(len(path)):
			coord_tuple = (self.map_x_coord[path[i]],self.map_y_coord[path[i]])
			path_coord.append(coord_tuple)
		self.path = path_coord
		self.path_shapely_line = LineString(self.path)
		print(self.path_shapely_line)

		if want_to_plot_path:
			self.Plot_path(spawn_pos, destination_pos, self.occupancy_grid, self.map_x_coord, self.map_y_coord, path_coord)

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

		self.vehicle.apply_control(self.control)


	def controller(self):
		speed_increment = 0.1 #TODO remove this increment and make it follow the velocity trajcetory we've got in the table or have both as an option
		self.Target_speed    = self.vehicle_speed 

		if (self.current_speed >= 0.7 * self.desired_speed and self.desired_speed <= self.Target_speed):
			self.desired_speed = self.desired_speed + speed_increment

		if (self.current_speed >= 0.7 * self.desired_speed and self.desired_speed > self.Target_speed):
			self.desired_speed = self.desired_speed - speed_increment

		Kp_longitudinal               = 0.25
		Ki_longitudinal               = 0.03
		Kd_longitudinal               = 0.1
		K_feedforward                 = 0.1

		longitudinal_error            = self.desired_speed - self.current_speed
		integral_longitudinal_error   = longitudinal_error + self.longitudinal_error_previous
		derivative_longitudinal_error = longitudinal_error - self.longitudinal_error_previous

		dt                            = self.current_t - self.previous_t


		
		if dt == 0:
			PID_output = Kp_longitudinal * longitudinal_error + Ki_longitudinal * integral_longitudinal_error
		else:
			PID_output = Kp_longitudinal * longitudinal_error + Ki_longitudinal * integral_longitudinal_error + Kd_longitudinal * derivative_longitudinal_error / dt


		if PID_output == float('Inf'):
			PID_output = 0

		feedforward_control           = K_feedforward * self.desired_speed

		combined_control              = feedforward_control + PID_output

		# print("combined_control = ",combined_control)

		if combined_control >= 0:
			throttle_output = combined_control
			brake_output    = 0
		elif combined_control < 0:
			throttle_output = 0
			brake_output    = combined_control






		L               = 2    # Length of Vehicle
		l_d_min         = 2    # Setting a minimum lookahead distance
		l_d             = 0
		Kpp             = 0.5  # Lateral controller gain

		while True:
			if self.ignorePath:
				break

			if self.path_index < len(self.path):

				try:
					l_d = calculateDistance(self.path[self.path_index][0], self.path[self.path_index][1], self.current_x, self.current_y) 
				except IndexError:  
					print("Index doesn't exist!")
					break

				# print("l_d_min = ", l_d_min)
				# print("l_d = ", l_d)

				if l_d >= l_d_min:
					break

				self.path_index += 1
				self.path_index = int(self.path_index)
				# print("self.path_index = ",self.path_index)
				# print("self.path[self.path_index][0] = ",self.path[self.path_index][00])
				# print("self.path[self.path_index][1] = ",self.path[self.path_index][1])
			
			else:
				break

		try:
			if self.current_speed == 0:
				delta = 0
			else:
				if self.ignorePath:
					alpha_hat = np.arctan2(self.dest_y - self.current_y, self.dest_x - self.current_x)
				else:	
					alpha_hat = np.arctan2(self.path[self.path_index][1] - self.current_y, self.path[self.path_index][0] - self.current_x)
				
				alpha     = alpha_hat - self.current_yaw
				delta = np.arctan(2 * L * np.sin(alpha) / (Kpp * self.current_speed))

				# print("l_d = ",l_d)
				# print("self.path_index = ",self.path_index)
				# print("self.path[self.path_index][1] = ", self.path[self.path_index][1])
				# print("self.current_y = ", self.current_y)
				# print("self.path[self.path_index][0] = ", self.path[self.path_index][0])
				# print("self.current_y = ", self.current_y)
				# print("alpha_hat = ", alpha_hat)
				# print("alpha = ", alpha)

		except: 
			delta = 0


		steer_output = delta

		self.longitudinal_error_previous = longitudinal_error
		self.previous_t                  = self.current_t
		distance_from_destination        = np.sqrt((self.path[-1][0] - self.current_x)**2 + (self.path[-1][1] - self.current_y)**2)

		steer_output         = np.fmax(np.fmin(steer_output, 1.0), -1.0)
		throttle_output      = np.fmax(np.fmin(throttle_output, 1.0), 0)
		brake_output         = np.fmax(np.fmin(brake_output, 1.0), 0)
		
		if distance_from_destination <= self.STOPPING_DISTANCE:
			self.close_to_destination = True


		if self.obstacle:
			print("STOPPED!! OBSTACLE AHEAD!")
			self.stop = True

		elif self.red_traffic_light:
			print("STOPPED!! RED Traffic Light!")
			self.stop = True

		elif self.close_to_destination:
			print("VEHICLE ARRIVED!")
			self.stop = True
		
		else:
			self.stop = False


		if self.stop:
			cmd_throttle = 0
			cmd_steer    = 0
			cmd_brake    = 1
		else:
			cmd_throttle = throttle_output
			cmd_steer    = steer_output
			cmd_brake    = brake_output



		self.send_control(cmd_throttle, cmd_steer, cmd_brake, hand_brake=False, reverse=False)


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



# class traffic_light():