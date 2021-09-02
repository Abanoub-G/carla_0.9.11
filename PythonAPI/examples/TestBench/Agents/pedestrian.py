import carla
from __init__ import client, world
from TB_common_functions import wraptopi, calculateDistance, AgentColourToRGB
# from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt

def calculateDistance(x1,y1,x2,y2):
	dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return dist


class pedestrian():
	def __init__(self,agentNo,option):
		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		if option == "adult":
			self.ped_blueprint     = self.blueprint_library.filter("walker.pedestrian.0002")[0]#2
			self.agentTypeNo = 1
		if option == "kid":
			self.ped_blueprint     = self.blueprint_library.filter("walker.pedestrian.0010")[0]#10
			self.agentTypeNo = 2
		self.ped_blueprint.set_attribute('is_invincible', 'false')
		self.ped_controller_blueprint = self.blueprint_library.find('controller.ai.walker')

		self.SpawnActor        = carla.command.SpawnActor
		self.batch_W           = []
		self.batch_W_C         = []

		self.stop  = False
		self.agentNo = agentNo

		self.index = int(0)
		self.l_d_min           = 2    # Setting a minimum lookahead distance
		self.stopping_distance = 2    #5
		

	def spawn(self,x,y,z,yaw):
		self.spawn_orientation = yaw
		self.transform         = carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(self.spawn_orientation)))
		
		self.batch_W.append(self.SpawnActor(self.ped_blueprint, self.transform)) #Note I am using batch because regular comand not working
		self.results_p = self.client.apply_batch_sync(self.batch_W, True)
		self.ped_id    = self.results_p[0].actor_id
		self.world.tick()# important to tick after spawning, otherwise actor details not reachable
		actors         = self.world.get_actors()
		self.agent       = actors.find(self.ped_id)


		self.batch_W_C.append(self.SpawnActor(self.ped_controller_blueprint , carla.Transform(), self.agent)) #Note I am using batch because regular comand not working
		self.results_c          = self.client.apply_batch_sync(self.batch_W_C, True)
		self.ped_controller_id  = self.results_c[0].actor_id
		self.world.tick() # important to tick after spawning, otherwise actor details not reachable
		actors                  = self.world.get_actors()
		self.ped_controller     = actors.find(self.ped_controller_id)
	
		self.ped_controller.start()
		

		self.agentID    = self.agent.id
		self.agentType  = self.agent.type_id

	def set_destination(self,x,y,z):
		self.dest_x = x 
		self.dest_y = y 
		self.dest_z = z 

	def set_path(self,path_coord_tuple):

		self.path = path_coord_tuple  # [x,y]

		print("Pedestrian path set!")


	def find_lookahead_point(self):
		# self.index = int(0)
		l_end = 0 # distance to end of path
		l_d   = 0 # look ahead distance


		# calculate distance to end of path
		l_end = calculateDistance(self.path[-1][0], self.path[-1][1], self.current_x, self.current_y)
		# l_end = calculateDistance(self.path.x[-1], self.path.y[-1], self.current_x, self.current_y)

		while True:

			# check current pos is not too close to end 
			if l_end < self.l_d_min:
				self.index = -1
				break
				
			# calc look ahead point
			l_d = calculateDistance(self.path[self.index][0], self.path[self.index][1], self.current_x, self.current_y)
			# l_d = calculateDistance(self.path.x[self.index], self.path.y[self.index], self.current_x, self.current_y)

			if l_d >= self.l_d_min:
				break

			# check reached look ahead distance
			if l_d >= self.l_d_min:
				break

			self.index += int(1)

	def controller_step(self):
		try:
			# find the closest point in the path tuple
			self.find_lookahead_point()
			self.dest_x = self.path[self.index][0] 
			self.dest_y = self.path[self.index][1]
			self.dest_z = self.current_z

			distance_from_destination = np.sqrt((self.path[-1][0] - self.current_x)**2 + (self.path[-1][1] - self.current_y)**2)

			if distance_from_destination <= self.stopping_distance:
				print("PEDESTRIAN ARRIVED!")
				self.stop = True
		except:
			print("No path defined for pedestrian, agentNo = ",self.agentNo)
			pass

		# send command to go to location
		if self.stop == True:
			self.ped_controller.go_to_location(carla.Location(x=float(self.current_x), y=float(self.current_y), z=float(self.current_z)))
		else:
			self.ped_controller.go_to_location(carla.Location(x=float(self.dest_x), y=float(self.dest_y), z=float(self.dest_z)))
		
	def step(self):
		self.Update_state_info()
		self.ped_controller.set_max_speed(self.speed)
		
		self.controller_step()

		# self.ped_controller.go_to_location(carla.Location(x=float(self.dest_x), y=float(self.dest_y), z=float(self.dest_z)))
		

	def set_speed(self,speed):
		self.speed = speed
		

	def Update_state_info(self):
		self.world_snapshot = self.world.get_snapshot()
		agent_transform                 = self.agent.get_transform()
		agent_location                  = agent_transform.location  
		agent_rotation                  = agent_transform.rotation
		agent_velocity                  = self.agent.get_velocity()      # This is an object vector

		self.current_t                  = self.world_snapshot.timestamp.elapsed_seconds #ts.elapsed_seconds - start_of_simulation_timestamp
		self.current_x                  = agent_location.x
		self.current_y                  = agent_location.y
		self.current_z                  = agent_location.z
		self.current_yaw                = wraptopi(math.radians(agent_rotation.yaw))
		self.current_velocity           = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])      # This is an array as opposed to agent_velocity, which is an object
		self.current_speed              = np.sqrt(self.current_velocity.dot(self.current_velocity))


	def get_pos(self):
		self.Update_state_info()
		self.pos_array = [self.current_x, self.current_y, self.current_z, self.current_yaw]

		return self.pos_array

	def destroy(self):
		self.ped_controller.stop()
		self.ped_controller.destroy()
		self.agent.destroy()

	