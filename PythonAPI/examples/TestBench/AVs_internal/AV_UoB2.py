import carla
from __init__ import client, world
from environment_setup import wraptopi, calculateDistance, AgentColourToRGB
from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt

from controller import long_lat_controller
from frenet_optimal_trajectory import generate_target_course, frenet_optimal_planning

class autonomous_vehicle():
	def __init__(self,wx,wy):
		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		self.vehicle_blueprint = self.blueprint_library.filter("vehicle.mercedes-benz.coupe")[0]
		self.vehicle_blueprint.set_attribute('color', AgentColourToRGB("yellow"))

		self.controller = long_lat_controller()
		
		self.close_to_destination = False
		self.red_traffic_light    = False
		self.obstacle             = False

		self.stop  = False

		self.ignorePath = True

		self.vehicle_speed = 5 # Setting a default speed

		self.desired_speed = 0
		self.longitudinal_error_previous = 0



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
		

