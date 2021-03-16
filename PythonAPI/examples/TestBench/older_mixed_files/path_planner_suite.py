import carla
from __init__ import client, world
from TB_common_functions import wraptopi

from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.ops import cascaded_union

import numpy as np



class map_section():
	"""
		A polygon class for defining map. Contains:
		- IDs 
		- Coordinates defining polygon edges
		- Direction for path planning
		Obstacle       = 0
		North          = 1
		South          = 2
		East           = 3
		West           = 4
		North-East     = 5
		North-West     = 6
		South-East     = 7
		South-West     = 8
		All-directions = 9
	"""
	def __init__(self, ID, travel_direction, boundaries):
		self.ID               = ID
		self.boundaries       = boundaries
		self.travel_direction = travel_direction
		self.polygon          = Polygon(boundaries) 




def Generate_Numeric_Map(list_of_sections):
	"""
		Numerical grid map for pathplanning
	"""

	list_of_polygons = []
	
	for section in list_of_sections:
		list_of_polygons.append(section.polygon)
		
		
	# Create a union of the polygons
	u = cascaded_union(list_of_polygons)
	
	# Get bounds of union
	[min_bound_X, min_bound_Y, max_bound_X, max_bound_Y] = u.bounds
	
	# Create "coordinates" and "object in each coordinate" matrices
	map_res = 0.5  # Resolution in meters
	max_X = int(np.ceil(abs((max_bound_X - min_bound_X) / map_res))) + 1 # +1 to  make it inclusive
	max_Y = int(np.ceil(abs((max_bound_Y - min_bound_Y) / map_res))) + 1 # +1 to  make it inclusive
	map_x_coord   = (np.zeros((max_X,max_Y)));
	map_y_coord   = (np.zeros((max_X,max_Y)));
	map_objects   = (np.zeros((max_X,max_Y)));
	
	# Populate "coordinates" matrices
	for i_x in range(max_X):
		for i_y in range(max_Y):
			map_x_coord[i_x][i_y] = min_bound_X + i_x * map_res
			map_y_coord[i_x][i_y] = min_bound_Y + i_y * map_res
	

	# Populate obstacles in "object in each coordinate" matrix 
	for i_x in range(max_X):
		for i_y in range(max_Y):

			x_coord = map_x_coord[i_x][i_y]
			y_coord = map_y_coord[i_x][i_y]
			pnt     = Point(x_coord,y_coord)
			
			if u.contains(pnt):
				for p in list_of_sections:
					if p.polygon.contains(pnt) or p.polygon.exterior.distance(pnt) <= map_res:
						map_objects[i_x][i_y] = p.travel_direction
						break

	print("Progress: Numerical map successfully generated!!")           
	# Return Numerical_Maps
	return map_objects, map_x_coord, map_y_coord  



def find_nearest(array, value):
	arr2D = abs(array - value)
	result = np.where(arr2D == np.amin(arr2D))
	# zip the 2 arrays to get the exact coordinates
	listOfCordinates = list(zip(result[0], result[1]))
	return listOfCordinates







# =================================================================
# =================================================================
# Path planner related functions
# =================================================================
# =================================================================

class Node:
	"""
		A node class for A* Pathfinding
		parent is parent of the current Node
		position is current position of the Node in the maze
		g is cost from start to current Node
		h is heuristic based estimated cost for current Node to end Node
		f is total cost of present node i.e. :  f = g + h
	"""

	def __init__(self, parent=None, position=None):
		self.parent = parent
		self.position = position

		self.g = 0
		self.h = 0
		self.f = 0
	def __eq__(self, other):
		return self.position == other.position



#This function return the path of the search
def return_path(current_node,maze):
	path = []
	no_rows, no_columns = np.shape(maze)
	# here we create the initialized result maze with -1 in every position
	result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
	current = current_node
	while current is not None:
		path.append(current.position)
		current = current.parent
	# Return reversed path as we need to show from start to end path
	path = path[::-1]
	start_value = 0
	# we update the path of start to end found by A-star serch with every step incremented by 1
	for i in range(len(path)):
		result[path[i][0]][path[i][1]] = start_value
		start_value += 1
	return path,result


def get_move(node, maze):
#     print(node.position)
	i = int(maze[node.position])
	prone_to_crossing_lane = False
	UE_flag = False
	if UE_flag:
		c = -1 # y-axis in unreal engine 4 is inverted
	else:
		c = 1 
			
#     print(i)
	if i == 1: 
		# North
		move = [[ 0, 1 * c ], 
				[ 1, 1 * c ], 
				[-1, 1 * c ]]
#         If we want vehicle not to cross lanes to get to target, start from below but would need to improve heuristics

#         North_East_node_pos = (node.position[0] + 1, node.position[1] + 1)
#         check_i = int(maze[North_East_node_pos])
#         if check_i == 2:
#             prone_to_crossing_lane = True
		
#         if prone_to_crossing_lane: 
#             move = [[ 0, 1 ], 
#                     [-1, 1 ]]
#         else:
#             move = [[ 0, 1 ], 
#                     [ 1, 1 ], 
#                     [-1, 1 ]]
				
	elif i == 2:
		# South 
		move = [[ 0,-1 * c ], 
				[ 1,-1 * c ], 
				[-1,-1 * c ]]
	elif i == 3:
		# East 
		move = [[ 1, 0 * c ], 
				[ 1, 1 * c ], 
				[ 1,-1 * c ]]
	elif i == 4:
		# West 
		move = [[-1, 0 * c ], 
				[-1, 1 * c ], 
				[-1,-1 * c ]]
	elif i == 5:
		# North-East 
		move = [[ 1, 1 * c ], 
				[ 1, 0 * c ], 
				[ 0, 1 * c ]]
	elif i == 6:
		# North-West 
		move = [[-1, 1 * c ], 
				[-1, 0 * c ], 
				[ 0, 1 * c ]]
	elif i == 7:
		# South-East 
		move = [[ 1,-1 * c ], 
				[ 1, 0 * c ], 
				[ 0,-1 * c ]]
	elif i == 8:
		# South-West 
		move = [[-1,-1 * c ], 
				[-1, 0 * c ], 
				[ 0,-1 * c ]]
	elif i == 9:
		# All-directions 
		move = [[ 0, 1 ], 
				[ 1, 1 ], 
				[ 1, 0 ], 
				[ 1,-1 ], 
				[ 0,-1 ],
				[-1,-1 ],
				[-1, 0 ], 
				[-1, 1 ]]
	else:  
		move = [[ 0, 1 ], 
				[ 1, 1 ], 
				[ 1, 0 ], 
				[ 1,-1 ], 
				[ 0,-1 ],
				[-1,-1 ],
				[-1, 0 ], 
				[-1, 1 ]]

	return move




def astar_search(start, end, cost, AV_pos, Target_pos, maze, map_x_coord, map_y_coord, want_to_plot,plot_every):
	"""
		Returns a list of tuples as a path from the given start to the given end in the given maze
		:param maze:
		:param cost
		:param start:
		:param end:
		:return:
	"""
	if maze[start] == 0 or maze[end] == 0:
		print("ERROR: Initial pos or destination is not on a road in the Map!")
	else:
		# Create start and end node with initized values for g, h and f
		start_node = Node(None, tuple(start))
		start_node.g = start_node.h = start_node.f = 0
		end_node = Node(None, tuple(end))
		end_node.g = end_node.h = end_node.f = 0
		distance_to_target_threshold = 10 # meters

		# Initialize both yet_to_visit and visited list
		# in this list we will put all node that are yet_to_visit for exploration. 
		# From here we will find the lowest cost node to expand next
		yet_to_visit_list = []  
		# in this list we will put all node those already explored so that we don't explore it again
		visited_list = [] 

		# Add the start node
		yet_to_visit_list.append(start_node)

		# Adding a stop condition. This is to avoid any infinite loop and stop 
		# execution after some reasonable number of steps
		outer_iterations = 0
		max_iterations   = (len(maze) // 2) ** 10

		# what squares do we search . serarch movement is left-right-top-bottom 
		#(4 movements) from every positon

		# which squares do we search 
		move = get_move(start_node, maze)

		"""
			1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
			2) Check max iteration reached or not . Set a message and stop execution
			3) Remove the selected node from yet_to_visit list and add this node to visited list
			4) Perofmr Goal test and return the path else perform below steps
			5) For selected node find out all children (use move to find children)
				a) get the current postion for the selected node (this becomes parent node for the children)
				b) check if a valid position exist (boundary will make few nodes invalid)
				c) if any node is a wall then ignore that
				d) add to valid children node list for the selected parent

				For all the children node
					a) if child in visited list then ignore it and try next node
					b) calculate child node g, h and f values
					c) if child in yet_to_visit list then ignore it
					d) else move the child to yet_to_visit list
		"""
		#find maze has got how many rows and columns 
		no_rows, no_columns = np.shape(maze)

		# Loop until you find the end

		while len(yet_to_visit_list) > 0:

			# Every time any node is referred from yet_to_visit list, counter of limit operation incremented
			outer_iterations += 1    


			# Get the current node
			current_node = yet_to_visit_list[0]
			current_index = 0
			for index, item in enumerate(yet_to_visit_list):
				if item.f < current_node.f:
					current_node = item
					current_index = index

			# if we hit this point return the path such as it may be no solution or 
			# computation cost is too high
			if outer_iterations > max_iterations:
				print ("giving up on pathfinding too many iterations")
				return return_path(current_node,maze)

			# Pop current node out off yet_to_visit list, add to visited list
			yet_to_visit_list.pop(current_index)
			visited_list.append(current_node)

			# test if goal is reached or not, if yes then return the path
			if current_node == end_node:
				print("Progress: Path successfully generated!!")
				return return_path(current_node,maze)

			# Generate children from all adjacent squares
			children = []
			
			# which squares do we search
			distance_to_target = np.sqrt((((current_node.position[0] - end_node.position[0]) ** 2) + 
										  ((current_node.position[1] - end_node.position[1]) ** 2))) 
			if distance_to_target <= distance_to_target_threshold:
				move = [[ 0, 1 ], 
						[ 1, 1 ], 
						[ 1, 0 ], 
						[ 1,-1 ], 
						[ 0,-1 ],
						[-1,-1 ],
						[-1, 0 ], 
						[-1, 1 ]]
			else:
				move = get_move(current_node, maze)
			

			for new_position in move: 

				# Get node position
				node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

				# Make sure within range (check if within maze boundary)
				if (node_position[0] > (no_rows - 1) or 
					node_position[0] < 0 or 
					node_position[1] > (no_columns -1) or 
					node_position[1] < 0):
					continue

				# Make sure walkable terrain
				if maze[node_position[0]][node_position[1]] == 0:
					continue

				# Create new node
				new_node = Node(current_node, node_position)

				# Append
				children.append(new_node)

			# Loop through children
			for child in children:

				# Child is on the visited list (search entire visited list)
				if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
					continue

				# Create the f, g, and h values
				child.g = current_node.g + cost
				## Heuristic costs calculated here, this is using eucledian distance
				child.h = (((child.position[0] - end_node.position[0]) ** 2) + 
						   ((child.position[1] - end_node.position[1]) ** 2)) 

				child.f = child.g + child.h

				# Child is already in the yet_to_visit list and g cost is already lower
				if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
					continue

				# Add the child to the yet_to_visit list
				yet_to_visit_list.append(child)