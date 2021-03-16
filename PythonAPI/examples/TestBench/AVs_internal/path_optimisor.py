import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splev, splrep, interp1d
from shapely.geometry import Polygon
from shapely import affinity
from descartes import PolygonPatch
from environment_setup import generate_random_number_in_polygon


class autonomous_vehicle():# NOTE: SPAWN AGENTS THEN AV!!!!!!!!!!
	def __init__(self,wp_x,wp_y,vel_mag,env):

		dim             = np.array([1.9,5]) 
		safety_offet    = 0.3
		safety_offetX2  = safety_offet * 2 # taking account of both sides
		dim_safety      = np.array([dim[0]+safety_offetX2,dim[1]+safety_offetX2])


		self.wp_x       =  self.wp_x_current  = self.wp_x_pred = wp_x
		self.wp_y       =  self.wp_y_current  = self.wp_y_pred = wp_y
		self.vel_mag    = vel_mag

		self.yaw        = np.arctan2(self.wp_x_current[1] - self.wp_x_current[0],self.wp_y_current[1] - self.wp_y_current[0]) 
		self.pos        = np.array([self.wp_x_current[0],self.wp_y_current[0]])

		self.vel_mag    = vel_mag 
		self.vel_vec    = np.array([self.vel_mag*np.cos(np.pi / 2 -self.yaw),self.vel_mag*np.sin(np.pi / 2 -self.yaw)])



		self.pol_origin = Polygon([[-dim[0]/2, dim[1]/2],\
				[dim[0]/2, dim[1]/2],\
				[dim[0]/2, -dim[1]/2],\
				[-dim[0]/2,-dim[1]/2],\
				[-dim[0]/2, dim[1]/2]])



		self.pol_safe_origin = Polygon([[-dim_safety[0]/2, dim_safety[1]/2],\
				[dim_safety[0]/2, dim_safety[1]/2],\
				[dim_safety[0]/2, -dim_safety[1]/2],\
				[-dim_safety[0]/2,-dim_safety[1]/2],\
				[-dim_safety[0]/2, dim_safety[1]/2]])

		self.pol      = self.pol_origin
		self.pol_safe = self.pol_safe_origin
		# self.pol_pred = self.pol_safe

		
		# self.trans     =  self.pos
		# self.rotate    = -self.yaw

		self.pos_next = self.pos
		# print(self.pos)
		self.yaw_next = -self.yaw
		

		self.env = env
		self.agents_env = self.env.ped_list#self.env.ped1
		
		self.update_polygons()

		self.time_increment = self.env.time_increment
		self.max_time_pred  = 2


		# Defining flags
		self.main_route_clear     = True
		self.following_main_route = True
		self.current_route_clear  = True
		self.stop                 = False

		self.plot_pred            = True
		self.reached_destination  = False
		self.dist_destination_stop = 1


		# Calculating offsets for the avoidance paths
		road_width = 5
		path_offset_res = 0.5

		delta_x = wp_x[-1] - wp_x[0] 
		delta_y = wp_y[-1] - wp_y[0]

		L = np.sqrt(delta_x**2 + delta_y**2)  

		adjustment_factor = path_offset_res / L 

		self.x_offset =  delta_y * adjustment_factor
		self.y_offset = -delta_x * adjustment_factor

		self.lookahead_distance = 0.5
		self.i = 0

		self.sensors_radius  = 10




	def calc_pred_dist(self,vel_mag,prediction_time):
		# Calculate prediction distance threshold dependant on speed
		return vel_mag * prediction_time
		

	def step(self):
		self.gather_agents_data()

		self.pol, self.pol_safe, self.pos = self.update_AV_data(self.pol_origin,self.pol_safe_origin,\
																self.pos,self.vel_mag,self.wp_x_current,\
																self.wp_y_current,self.lookahead_distance,\
																self.time_increment)
		

		if self.reached_destination:
			self.vel_mag = 0 
			self.env.AV_arrived = True
			
	
		if not self.env.AV_arrived:
			self.predict_check_and_avoid()

		
		


	def gather_agents_data(self):
		self.agents = []
		for agent in self.agents_env:
			if self.in_circle(self.pos[0], self.pos[1], self.sensors_radius, agent.pos[0], agent.pos[1]):
				self.agents.append(agent_surronding_AV(agent.pos,agent.yaw,agent.vel_vec,agent.pol))


	def in_circle(self,center_x, center_y, radius, x, y):
		dist = np.sqrt((center_x - x) ** 2 + (center_y - y) ** 2)
		if dist <= radius:
			return True
		else:
			return False

	
	def update_AV_data(self,polygon,polygon_safe,pos,vel_mag,wp_x,wp_y,lookahead_distance,time_increment):
		# Find which waypoint to target 
		index = self.get_lookahead_waypoint(pos[0],pos[1],wp_x,wp_y,lookahead_distance)

		# Calculate Yaw, Velocity Vector and Position
		angle_from_x_axis = np.arctan2(wp_y[index] - pos[1],wp_x[index] - pos[0]) 
		yaw = -(angle_from_x_axis + np.pi / 2)
		self.vel_vec  =  np.array([vel_mag*np.cos(angle_from_x_axis),vel_mag*np.sin(angle_from_x_axis)])
		pos =  pos + self.vel_vec*time_increment

		# Update polygons
		polygon = affinity.translate(polygon,pos[0],pos[1])
		polygon = affinity.rotate(polygon,np.rad2deg(-yaw), 'center')
		polygon_safe = affinity.translate(polygon_safe,pos[0],pos[1])
		polygon_safe = affinity.rotate(polygon_safe,np.rad2deg(-yaw), 'center')
		self.pol_pred  = polygon_safe

		# Check if reached destination
		diff       = pos - np.array([wp_x[-1],wp_y[-1]]) 
		dest_dist = np.sqrt(diff[0]**2 + diff[1]**2)
		if dest_dist <= self.dist_destination_stop:
			self.reached_destination = True
		

		return polygon,polygon_safe, pos



	def update_polygons(self):
		self.pol      = self.pol_origin
		self.pol_safe = self.pol_safe_origin

		self.pol      = affinity.translate(self.pol,self.pos[0],self.pos[1])
		self.pol      = affinity.rotate(self.pol,np.rad2deg(-self.yaw), 'center')
		self.pol_safe = affinity.translate(self.pol_safe,self.pos[0],self.pos[1])
		self.pol_safe = affinity.rotate(self.pol_safe,np.rad2deg(-self.yaw), 'center')
		self.pol_pred = self.pol_safe


	def closest_node(self,node,nodes):
		deltas = nodes - node
		dist_2 = np.einsum('ij,ij->i', deltas, deltas)
		dist   = np.sqrt(dist_2)
		return np.argmin(dist_2), dist

	def get_lookahead_waypoint(self,x,y,wp_x,wp_y,lookahead_distance):
		node   = np.array([x,y])
		nodes  = np.c_[wp_x.T, wp_y.T]
		index, nodes_dist  = self.closest_node(node,nodes)

		# Make sure we don't go beyond last waypoint
		if index == (len(wp_x) - 1):
			return index 

		# Get vector from position x,y to closest node index
		vect1 = np.array([wp_x[index] - node[0], wp_y[index] - node[1]])
		# Get vector from index to index+1
		vect2 = np.array([wp_x[index+1] - wp_x[index], wp_y[index+1] - wp_y[index]])
		# If closest point is behind current pos then shift to closest point in front.
		if np.linalg.norm(vect1 + vect2) <= np.linalg.norm(vect2):
			index += 1
		# Get index of closest node after look ahead distance 
		lookahead_index = np.nonzero(nodes_dist[index:] >= lookahead_distance)
		lookahead_index = lookahead_index[0]

		return index + lookahead_index[0]
		
	def predict_AV(self,polygon,pos,vel_mag,wp_x,wp_y,lookahead_distance,time_increment):
		# Find which waypoint to target 
		index = self.get_lookahead_waypoint(pos[0],pos[1],wp_x,wp_y,lookahead_distance)

		# Predict Yaw, Velocity Vector and Position
		angle_from_x_axis = np.arctan2(wp_y[index] - pos[1],wp_x[index] - pos[0]) 
		yaw_pred = -(angle_from_x_axis + np.pi / 2)
		vel_vec  =  np.array([vel_mag*np.cos(angle_from_x_axis),vel_mag*np.sin(angle_from_x_axis)])
		pos_pred =  pos + vel_vec*time_increment

		# Update prediction polygon
		polygon = affinity.translate(polygon,pos_pred[0],pos_pred[1])
		polygon = affinity.rotate(polygon,np.rad2deg(-yaw_pred), 'center')

		return polygon, pos_pred

	def predict_agents(self,agents,time_increment):
		# Loop over agents
		for agent in agents:
			# Predict Position
			agent.pos_pred  =  agent.pos_pred + agent.vel_vec*time_increment

			# Update prediction polygon 
			agent.pol_pred = agent.pol_origin
			agent.pol_pred = affinity.translate(agent.pol_pred,agent.pos_pred[0],agent.pos_pred[1])
			agent.pol_pred = affinity.rotate(agent.pol_pred,np.rad2deg(-agent.yaw), 'center')


	def check_collisions(self,AV,agents):
		for agent in AV.agents:
			if AV.pol_pred.intersects(agent.pol_pred):
				return False
		return True


	def predict_check_and_avoid(self):
		# Get prediction variables 
		time_increment      = self.time_increment
		max_pred_dist = self.calc_pred_dist(self.vel_mag,self.max_time_pred)

		self.wp_x_pred = self.wp_x_current
		self.wp_y_pred = self.wp_y_current
		AV_pos_pred    = self.pos

		j = 0
		k = 0


		while True:
			# Predict AV step
			self.pol_pred, AV_pos_pred = self.predict_AV(self.pol_safe_origin,AV_pos_pred,\
												self.vel_mag,self.wp_x_pred,self.wp_y_pred,\
												self.lookahead_distance,time_increment)

			# Predict agents step
			self.predict_agents(self.agents,time_increment)

			# Render prediction
			if self.plot_pred:
				self.env.render_env()

			# Check for intersections
			pred_route_clear = self.check_collisions(self,self.agents)

			# Check if prediction distance surpassed or reached destination
			diff1      = AV_pos_pred - self.pos 
			diff2      = AV_pos_pred - np.array([self.wp_x_pred[-1],self.wp_y_pred[-1]]) 
			pred_dist1 = np.sqrt(diff1[0]**2 + diff1[1]**2)
			pred_dist2 = np.sqrt(diff2[0]**2 + diff2[1]**2)
			if pred_dist1 >= max_pred_dist or pred_dist2 <= self.dist_destination_stop:
				end_of_predicted_route = True
			else:
				end_of_predicted_route = False

			# Store new path if okay and terminate loop.
			if  end_of_predicted_route and pred_route_clear: 
				self.wp_x_current = self.wp_x_pred
				self.wp_y_current = self.wp_y_pred
				break

			# Look for another path if predicted route not clear
			if not pred_route_clear:

				# Reset pred variables
				AV_pos_pred = self.pos
				for agent in self.agents:
					agent.pos_pred = agent.pos


				k += 1 
				if (k % 2) != 0: 
					j += 1

				j *= -1

				self.wp_x_pred = self.wp_x + self.x_offset * j
				self.wp_y_pred = self.wp_y + self.y_offset * j

				# Modify original path with overtaking points to make predicted path
				overtaking_waypoint_start  = np.array([self.wp_x_pred[0],self.wp_y_pred[0]])
				overtaking_waypoint_end    = np.array([self.wp_x_pred[1],self.wp_y_pred[1]])

				overtaking_waypoint_start  = overtaking_waypoint_start + 0.3 * (overtaking_waypoint_end - overtaking_waypoint_start)
				overtaking_waypoint_end    = overtaking_waypoint_end - 0.3 * (overtaking_waypoint_end - overtaking_waypoint_start) 

				self.wp_x_pred[0] = overtaking_waypoint_start[0]
				self.wp_y_pred[0] = overtaking_waypoint_start[1]

				self.wp_x_pred[1] = overtaking_waypoint_end[0]
				self.wp_y_pred[1] = overtaking_waypoint_end[1]

				self.wp_x_pred = np.insert(self.wp_x_pred, 0, self.pos[0], axis=0)
				self.wp_y_pred = np.insert(self.wp_y_pred, 0, self.pos[1], axis=0)

				self.wp_x_pred = np.insert(self.wp_x_pred, 3, self.wp_x[-1], axis=0)
				self.wp_y_pred = np.insert(self.wp_y_pred, 3, self.wp_y[-1], axis=0)

				pred_route_clear = True


		# ##############################
		# t = 0

		# pos_new = self.pos

		# self.wp_x_pred = self.wp_x_current
		# self.wp_y_pred = self.wp_y_current

		# pred_route_clear = True

		# i1 = self.i
		# i2 = 0
		# j  = 0
		# k  = 0

		# prediction_dist_max = 10
		# predicted_dist = 0

		# agents_pos_new = []
		# for i in range(length(self.agents)):
		# 	agents_pos_new.append(self.agents[i].pos)

		# pred_route_clear = True



		# while True:
		# 	# t += self.time_increment

		# 	# Check current path
		# 	# Finding which waypoint to target in route
		# 	diff_pos = pos_new - np.array([self.wp_x_pred[i1+1],self.wp_y_pred[i1+1]])
		# 	diff_pos_distance = np.hypot(diff_pos[0],diff_pos[1])

		# 	dis_to_dest_vec = pos_new - np.array([self.wp_x_pred[-1],self.wp_y_pred[-1]])
		# 	dis_to_dest = np.hypot(dis_to_dest_vec[0],dis_to_dest_vec[1])


		# 	if diff_pos_distance < self.look_ahead_waypoints and dis_to_dest > self.look_ahead_waypoints :
		# 		i1 += 1

		# 	# Predicitng AV movment
		# 	yaw_pred =  -np.arctan2(self.wp_y_pred[i1+1] - self.wp_y_pred[i1],self.wp_x_pred[i1+1] - self.wp_x_pred[i1]) + np.pi / 2 
		# 	vel_vec_pred = np.array([self.vel_mag*np.cos(np.pi / 2 -yaw_pred),self.vel_mag*np.sin(np.pi / 2 -yaw_pred)])

		# 	pos_pred = pos_new + vel_vec_pred*self.time_increment
		# 	self.pol_pred = self.pol_safe_origin
		# 	trans    = pos_pred - self.pos 
		# 	self.pol_pred = affinity.translate(self.pol_pred,pos_pred[0],pos_pred[1])
		# 	self.pol_pred = affinity.rotate(self.pol_pred,np.rad2deg(-yaw_pred), 'center')
		# 	predicted_dist = np.sqrt(trans[0]**2 + trans[1]**2)

		# 	pos_new  = pos_pred

		# 	# Predicitng agents movment
		# 	for agent in self.agents:#range(length(self.agents)):
		# 	# self.agents_pol_pred[i] = self.agents.pol
		# 		agent_yaw_pred =  -np.arctan2(agent.wp_y[i2+1] - agent.wp_y[i2],agent.wp_x[i2+1] - agent.wp_x[i2]) + np.pi / 2 
		# 		agent_vel_vec_pred = np.array([agent.vel_mag*np.cos(np.pi / 2 -agent_yaw_pred),agent.vel_mag*np.sin(np.pi / 2 -agent_yaw_pred)])
		# 		############Stopped Here##############
		# 		Stopped Here
		# 		###############

		# 		agent_pos_pred = agent_pos_new + agent_vel_vec_pred*agent.time_increment
		# 		self.agent_pol_pred = agent.pol
		# 		agent_trans    = agent_pos_pred - agent.pos 
		# 		self.agent_pol_pred = affinity.translate(self.agent_pol_pred,agent_trans[0],agent_trans[1])

		# 		agent_pos_new  = agent_pos_pred

		# 	if self.plot_pred:
		# 		self.env.render_env()


		# 	# modify to for loop to go over all of the agents in env
		# 	if self.pol_pred.intersects(self.agent_pol_pred):
		# 		pred_route_clear = False
		# 		continue 

		# 	if dis_to_dest < self.look_ahead_waypoints or predicted_dist >= prediction_dist_max:
		# 		# store the path found 
		# 		self.wp_x_current = self.wp_x_pred
		# 		self.wp_y_current = self.wp_y_pred
		# 		break




		# 	if not pred_route_clear:
		# 		# if predicted route 
		# 		k += 1 
		# 		t = 0
		# 		i1 = self.i
		# 		i2 = 0
		# 		pos_new = self.pos
		# 		agent_pos_new = self.agent.pos
		# 		predicted_dist = 0


		# 		if (k % 2) != 0: 
		# 			j += 1

		# 		j *= -1
		# 		self.wp_x_pred = self.wp_x + self.x_offset * j
		# 		self.wp_y_pred = self.wp_y + self.y_offset * j

		# 		# Modify path with overtaking points
		# 		overtaking_waypoint_start  = np.array([self.wp_x_pred[0],self.wp_y_pred[0]])
		# 		overtaking_waypoint_end    = np.array([self.wp_x_pred[1],self.wp_y_pred[1]])

		# 		overtaking_waypoint_start  = overtaking_waypoint_start + 0.3 * (overtaking_waypoint_end - overtaking_waypoint_start)
		# 		overtaking_waypoint_end    = overtaking_waypoint_end - 0.3 * (overtaking_waypoint_end - overtaking_waypoint_start) 

		# 		self.wp_x_pred[0] = overtaking_waypoint_start[0]
		# 		self.wp_y_pred[0] = overtaking_waypoint_start[1]

		# 		self.wp_x_pred[1] = overtaking_waypoint_end[0]
		# 		self.wp_y_pred[1] = overtaking_waypoint_end[1]

		# 		self.wp_x_pred = np.insert(self.wp_x_pred, 0, self.pos[0], axis=0)
		# 		self.wp_y_pred = np.insert(self.wp_y_pred, 0, self.pos[1], axis=0)

		# 		self.wp_x_pred = np.insert(self.wp_x_pred, 3, self.wp_x[-1], axis=0)
		# 		self.wp_y_pred = np.insert(self.wp_y_pred, 3, self.wp_y[-1], axis=0)

		# 		# # store the path found 
		# 		# self.wp_x_current = self.wp_x_pred
		# 		# self.wp_y_current = self.wp_y_pred
		# 		pred_route_clear = True

			# t += self.time_increment

			# # Finding which waypoint to target in route
			# diff_pos = pos_new - np.array([self.wp_x_pred[i1+1],self.wp_y_pred[i1+1]])
			# diff_pos_distance = np.hypot(diff_pos[0],diff_pos[1])

			# dis_to_dest_vec = pos_new - np.array([self.wp_x_pred[-1],self.wp_y_pred[-1]])
			# dis_to_dest = np.hypot(dis_to_dest_vec[0],dis_to_dest_vec[1])


			# if diff_pos_distance < self.look_ahead_waypoints and dis_to_dest > self.look_ahead_waypoints :
				# i1 += 1

			# # Predicitng AV movment
			# yaw_pred =  -np.arctan2(self.wp_y_pred[i1+1] - self.wp_y_pred[i1],self.wp_x_pred[i1+1] - self.wp_x_pred[i1]) + np.pi / 2 
			# vel_vec_pred = np.array([self.vel_mag*np.cos(np.pi / 2 -yaw_pred),self.vel_mag*np.sin(np.pi / 2 -yaw_pred)])

			# pos_pred = pos_new + vel_vec_pred*self.time_increment
			# self.pol_pred = self.pol_safe_origin
			# trans    = pos_pred - self.pos 
			# self.pol_pred = affinity.translate(self.pol_pred,pos_pred[0],pos_pred[1])
			# self.pol_pred = affinity.rotate(self.pol_pred,np.rad2deg(-yaw_pred), 'center')
			# predicted_dist = np.sqrt(trans[0]**2 + trans[1]**2)


			# pos_new  = pos_pred

			# # Predicitng agents movment
			# for agent in self.agents:#range(length(self.agents)):
			# # self.agents_pol_pred[i] = self.agents.pol
			# 	agent_yaw_pred =  -np.arctan2(agent.wp_y[i2+1] - agent.wp_y[i2],agent.wp_x[i2+1] - agent.wp_x[i2]) + np.pi / 2 
			# 	agent_vel_vec_pred = np.array([agent.vel_mag*np.cos(np.pi / 2 -agent_yaw_pred),agent.vel_mag*np.sin(np.pi / 2 -agent_yaw_pred)])
			# 	############Stopped Here##############
			# 	Stopped Here
			# 	###############

			# 	agent_pos_pred = agent_pos_new + agent_vel_vec_pred*agent.time_increment
			# 	self.agent_pol_pred = agent.pol
			# 	agent_trans    = agent_pos_pred - agent.pos 
			# 	self.agent_pol_pred = affinity.translate(self.agent_pol_pred,agent_trans[0],agent_trans[1])

			# 	agent_pos_new  = agent_pos_pred

			# if self.plot_pred:
			# 	self.env.render_env()


			# # modify to for loop to go over all of the agents in env
			# if self.pol_pred.intersects(self.agent_pol_pred):
			# 	pred_route_clear = False
			# 	continue 

			# if dis_to_dest < self.look_ahead_waypoints or predicted_dist >= prediction_dist_max:
			# 	# store the path found 
			# 	self.wp_x_current = self.wp_x_pred
			# 	self.wp_y_current = self.wp_y_pred
			# 	break


			######################################################
			# Limit AV prediciton to certain distance
			# Add polygon check for multiple agents
			# Have the whole scenario build i.e. two pedestrian and vehicle
			# Limit path planning arround to fit lane size 
			#####################################################

			# if diff_pos_mag < look_ahead:
			# 	i += 1 


			# if main_route_clear and following_main_route:
			# 	break

			# if main_route_clear and not following_main_route:

			

		#check main route
			#if main_route_clear and "following main route" flag is ON: DO NOTHING

			#if main_route_clear and "following main route" flag is OFF: 
				# check going back to main route
					# if okay then go back to main route

			#if not main_route_clear and "following main route" flag is ON: 
				# Find alternative path with in road lanes 
					# if no paths with in road lanes then stop

			#if not main_route_clear and "following main route" flag is OFF:
				# check current route
					# if current_route_clear: DO NOTHING

					# if not current_route_clear:
						# Find alternative path with in road lanes 
							# if no paths with in road lanes then stop



	# def update_prediction_polygons(self):


class agent_surronding_AV():
	def __init__(self,pos,yaw,vel_vec,polygon):
		dim     = np.array([0.8,0.4])
		self.pos       = pos
		self.yaw       = yaw
		self.vel_vec   = vel_vec
		self.pol       = polygon
		self.pol_pred  = polygon
		self.pos_pred  = pos
		self.pol_origin = Polygon([[-dim[0]/2, dim[1]/2],\
									[dim[0]/2, dim[1]/2],\
									[dim[0]/2, -dim[1]/2],\
									[-dim[0]/2,-dim[1]/2],\
									[-dim[0]/2, dim[1]/2]])


class ped():
	def __init__(self,wp_x,wp_y,vel_mag,env):
		dim     = np.array([0.8,0.4])  
		
		self.wp_x       = wp_x
		self.wp_y       = wp_y
		self.vel_mag    = vel_mag

		self.yaw        = np.arctan2(self.wp_x[1] - self.wp_x[0],self.wp_y[1] - self.wp_y[0]) 
		self.pos        = np.array([self.wp_x[0],self.wp_y[0]])

		self.vel_mag    = vel_mag 
		self.vel_vec    = np.array([self.vel_mag*np.cos(np.pi / 2 -self.yaw),self.vel_mag*np.sin(np.pi / 2 -self.yaw)])
		
		


		self.pol = Polygon([[-dim[0]/2, dim[1]/2],\
				[dim[0]/2, dim[1]/2],\
				[dim[0]/2, -dim[1]/2],\
				[-dim[0]/2,-dim[1]/2],\
				[-dim[0]/2, dim[1]/2]])
		

		self.trans     =  self.pos
		self.rotate    = -self.yaw
		self.update_polygon()

		self.env = env
		self.time_increment = self.env.time_increment

		self.reached_destination = False

	def step(self):
		# Calculate distance to destination
		stopping_distance = 0.5
		dis_to_dest_vec = self.pos - np.array([self.wp_x[-1],self.wp_y[-1]])
		dis_to_dest = np.hypot(dis_to_dest_vec[0],dis_to_dest_vec[1])
		
		if dis_to_dest < stopping_distance:
			self.reached_destination = True

		if self.reached_destination:
			self.vel_mag = 0 

		self.vel_vec = np.array([self.vel_mag*np.cos(np.pi / 2 -self.yaw),self.vel_mag*np.sin(np.pi / 2 -self.yaw)])
		pos_next = self.pos + self.vel_vec*self.time_increment
		yaw_next = np.arctan2(self.wp_x[1] - self.wp_x[0],self.wp_y[1] - self.wp_y[0]) 
		self.trans    = pos_next - self.pos 
		self.rotate    = yaw_next - self.yaw

		self.update_polygon()

		self.pos = pos_next


	def update_polygon(self):
		self.pol      = affinity.translate(self.pol,self.trans[0],self.trans[1])
		self.pol      = affinity.rotate(self.pol,np.rad2deg(self.rotate), 'center')





class AV_Testing_env():
	def __init__(self):
		self.pave1         = Polygon([[-10.0,-71.4],[-9.6,-83.4],[-7.1,-83.3],[-7.5,-71.3]])
		self.lane1         = Polygon([[-7.8,-61.4],[-6.8,-95.2],[-3.4,-94.9],[-4.4,-61.4]])
		self.lane2         = Polygon([[-4.4,-61.4],[-3.4,-94.9],[0.3,-94.6],[-0.7,-61.1]])
		self.pave2         = Polygon([[-0.45,-71.0],[-0.05,-83.1],[3.1,-82.9],[2.9,-70.8]]) 
		self.crossing      = Polygon([[-0.3,-74.2],[-0.2,-78.8],[-7.3,-79.0],[-7.4,-74.6]])

		self.AV_arrived    = False

	def set(self):
		self.t = 0
		self.time_increment = 0.5
		self.no_of_ped      = 2

		self.ped_list = []
		# Setting ped agents
		for i in range(self.no_of_ped):
			ped_wp_start = generate_random_number_in_polygon(self.pave2, number=1)
			ped_wp_end   = generate_random_number_in_polygon(self.pave1, number=1)
			ped_wp_x     = np.array([ped_wp_start[0][0][0],ped_wp_end[0][0][0]])#np.array([-2,-8])
			ped_wp_y     = np.array([ped_wp_start[0][0][1],ped_wp_end[0][0][1]])#np.array([-80,-75])
			ped_vel_mag  = 2
			ped_i     = ped(ped_wp_x,ped_wp_y,ped_vel_mag,self)
			self.ped_list.append(ped_i)

		# Setting AV
		AV_wp_x    = np.array([-6.2,-5.3])
		AV_wp_y    = np.array([-63.6,-92.8])
		AV_vel_mag = 4
		self.AV    = autonomous_vehicle(AV_wp_x,AV_wp_y,AV_vel_mag,self)

	def step(self):
		self.t += self.time_increment

		for i in range(self.no_of_ped):
			self.ped_list[i].step()
		# self.ped1 = self.ped_list[0]
		# self.ped1.step()
		self.AV.step()
		

		self.render_env()

		# Do randomisation first then for loop for multiple agents



	def render_env(self):
		plt.cla()
		plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
		ax = plt.gca()

		# Plot static objects
		ax.add_patch(PolygonPatch(self.pave1, fc='red', alpha=0.2, ec='none',label='Pave 1'))
		ax.add_patch(PolygonPatch(self.lane1, fc='blue', alpha=0.2, ec='none',label='Lane 1'))
		ax.add_patch(PolygonPatch(self.lane2, fc='purple', alpha=0.2, ec='none',label='Lane 2'))
		ax.add_patch(PolygonPatch(self.pave2, fc='red', alpha=0.2, ec='none',label='Pave 2'))
		ax.add_patch(PolygonPatch(self.crossing, fc='yellow', alpha=0.2, ec='none',label='Crossing'))
		

		# Plot AV and prediction
		ax.add_patch(PolygonPatch(self.AV.pol, fc='grey', alpha=0.9, ec='none',label='AV'))
		ax.add_patch(PolygonPatch(self.AV.pol_safe, fc='grey', alpha=0.4, ec='none',label='AV Safety boundary'))
		plt.plot(self.AV.wp_x, self.AV.wp_y,'--',color='black', label='Main path');
		plt.plot(self.AV.wp_x_current, self.AV.wp_y_current,'-',color='green', label='Avoidance path');
		plt.plot(self.AV.pos[0], self.AV.pos[1], 'o', color='black');
		plt.quiver(self.AV.pos[0], self.AV.pos[1], self.AV.vel_vec[0], self.AV.vel_vec[1], angles='xy', scale_units='xy', zorder=9, scale=1)
		ax.add_patch(plt.Circle((self.AV.pos[0], self.AV.pos[1]), self.AV.sensors_radius, fc='red',alpha=0.1,))

		# Plot agents and prediction
		for i in range(self.no_of_ped):
			ax.add_patch(PolygonPatch(self.ped_list[i].pol, fc='grey', alpha=0.9, ec='none',label='Dynamic Objects'))
			plt.plot(self.ped_list[i].pos[0], self.ped_list[i].pos[1], 'o', color='black');
			plt.quiver(self.ped_list[i].pos[0], self.ped_list[i].pos[1], self.ped_list[i].vel_vec[0], self.ped_list[i].vel_vec[1], angles='xy', scale_units='xy', zorder=10, scale=1)

		plt.plot(*self.AV.pol_pred.exterior.xy, '--', color='grey');
		plt.plot(self.AV.wp_x_pred, self.AV.wp_y_pred,'--',color='grey', label='Predicited path');
		for agent in self.AV.agents:
			plt.plot(*agent.pol_pred.exterior.xy, '--', color='grey');

		
		plt.grid()
		ax.set_aspect('equal')
		# ax.legend(loc='upper right')
		plt.pause(0.1)


for i in range(10):
	print("Run Number = ",i)
	env = AV_Testing_env()
	env.set()

	while True:

		env.step()
		if env.AV_arrived:
			break
		# print("rendered")
