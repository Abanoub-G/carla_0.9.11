import carla
from __init__ import client, world
from TB_common_functions import wraptopi, calculateDistance, AgentColourToRGB
from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt

from controller import long_lat_controller
from frenet_optimal_trajectory import generate_target_course, frenet_optimal_planning

class autonomous_vehicle():
	def __init__(self,occupancy_grid,map_x_coord,map_y_coord):
		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		self.vehicle_blueprint = self.blueprint_library.filter("vehicle.mercedes-benz.coupe")[0]
		self.vehicle_blueprint.set_attribute('color', AgentColourToRGB("yellow"))

		self.controller = long_lat_controller()
		
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

		self.frenet_start_flag = True



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
			# self.send_control(self.throttle, 0, 1, hand_brake=False, reverse=False)
			self.get_to_destination()	
			# self.controller()
		
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

	def get_actors_xy_pos(self):
		flag = False
		actors = self.world.get_actors()
		actors_pos = np.array([[]])
		for actor in actors:
			if ("vehicle" in actor.type_id) or ("walker.pedestrian" in actor.type_id):
				
				if actor.id == self.vehicle.id:
					continue # don't take pos of AV as one of the actors

				actor_location = actor.get_transform().location
				
				if flag == False:
					actors_pos = np.array([[actor_location.x,actor_location.y]])
					flag = True
				else:
					actors_pos = np.append(actors_pos,[[actor_location.x,actor_location.y]],axis=0)
		print("actors_pos = ",actors_pos)
		return actors_pos


	def get_to_destination(self):

		# update variables for frenet 
		if self.frenet_start_flag:
			c_speed = 1  # current speed [m/s]
			c_d = 0 # current lateral position [m]
			c_d_d = 0.0  # current lateral speed [m/s]
			c_d_dd = 0.0  # current lateral acceleration [m/s]
			s0 = 0.0  # current course position

			self.frenet_start_flag = False
		else:
			min_dist = float("inf")
			print("len(self.path.s) = ",len(self.path.s))
			print("len(self.path.x) = ",len(self.path.x))

			for i in range(len(self.path.x)):
				dist_AV_to_traj_point = calculateDistance(self.path.x[i], self.path.y[i], self.current_x, self.current_y)
				if dist_AV_to_traj_point < min_dist:
					min_dist = dist_AV_to_traj_point
					index = i

			s0 = self.path.s[index]
			c_d = self.path.d[index]
			c_d_d = self.path.d_d[index]
			c_d_dd = self.path.d_dd[index]
			c_speed = self.current_speed
		
		# check c_speed < 1, frentet optimisor fails if c_speed<1
		if c_speed < 1:
			c_speed = 1


		# update actors positions
		ob = self.get_actors_xy_pos()
		# ob = np.array([[31.5, -38.1]
                       # ])
	
		
		# frenet_optimal_planning
		self.newpath = frenet_optimal_planning(self.csp, s0, c_speed,c_d, c_d_d, c_d_dd, ob)
		# print("newpath = ",self.newpath)
		if self.newpath == None:
			self.path = self.path
			print("path = ",self.path)
			print("newpath = ",self.newpath)
			print("c_speed = ",c_speed)
			print("s0 = ",s0)
			print("c_d = ",c_d)
			print("c_d_d = ",c_d_d)
			print("c_d_dd = ",c_d_dd)

		else:
			self.path = self.newpath

		
		area = 20.0

		# print("path = ",self.path)
		plt.cla()
		# for stopping simulation with the esc key.
		plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
		plt.plot(self.tx, self.ty)
		if ob.size:
			plt.plot(ob[:, 0], ob[:, 1], "xk")
		plt.plot(self.path.x[1:], self.path.y[1:], "-or")
		plt.plot(self.path.x[1], self.path.y[1], "vc")
		plt.xlim(self.path.x[1] - area, self.path.x[1] + area)
		plt.ylim(self.path.y[1] - area, self.path.y[1] + area)
		plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
		plt.grid(True)
		plt.pause(0.0001)

		# conrtoller step
		throttle, brake, steer = self.controller.step(self.path, self.current_x, self.current_y, self.current_yaw, self.current_speed)

		throttle_output      = np.fmax(np.fmin(throttle, 1.0), 0)
		brake_output         = np.fmax(np.fmin(brake, 1.0), 0)
		steer_output         = np.fmax(np.fmin(steer, 1.0), -1.0)

		self.send_control(throttle_output, steer_output, brake_output, hand_brake=False, reverse=False)


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
		self.main_path = path_coord
		self.path_shapely_line = LineString(self.main_path)
		print(self.path_shapely_line)

		# wx = [x[0] for x in path_coord]
		# wy = [x[1] for x in path_coord]
		wx = [self.spawn_point_x, 29.4, 41.5]
		wy = [self.spawn_point_y, -41.2,-47.4]

		self.tx, self.ty, self.tyaw, self.tc, self.csp = generate_target_course(wx, wy)

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