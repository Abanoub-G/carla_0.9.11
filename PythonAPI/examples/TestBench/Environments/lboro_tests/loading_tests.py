import pandas as pd
import numpy as np
from shapely.geometry import Polygon
from shapely import affinity
from descartes import PolygonPatch
import matplotlib.pyplot as plt

def split(word): 
	word_split = []
	for char in word:
		word_split.append(char)
	return word_split

# def convert_to_game_coordinates(AV_pos,AV_yaw,AV_length,hazard_pos_fromAV):
# 	x0     = AV_pos[0]
# 	y0     = AV_pos[1]
# 	theta0 = AV_yaw
# 	l0     = AV_length
# 	# the if function below is to cater for loughborough inconsistent test grid
# 	if hazard_pos_fromAV[0] >= -1 and hazard_pos_fromAV[0] <= 1:
# 		x1     = hazard_pos_fromAV[0] * 0.8
# 	elif hazard_pos_fromAV[0] < -1:
# 		x1     =  -0.8 + (hazard_pos_fromAV[0]+1)
# 	elif hazard_pos_fromAV[0] > 1:
# 		x1     =   0.8 + (hazard_pos_fromAV[0]-1) 

# 	y1     = hazard_pos_fromAV[1] 
# 	theta1 = np.arctan2(x1,y1)
# 	l1     = np.sqrt(x1**2 + y1**2)

# 	hazard_posx =   x0 \
#                  + (l0/2) * np.cos(np.pi/2 - theta0) \
#                  + l1 * np.cos(np.pi / 2 - theta0 - theta1)             
# 	hazard_posy =   y0 \
#                  + (l0/2) * np.sin(np.pi / 2 - theta0) \
#                  + l1 * np.sin(np.pi / 2 - theta0 - theta1)  
# 	return [hazard_posx, hazard_posy]


# def get_hazard(test_details,AV_pos,AV_yaw,AV_length):
# 	location = test_details["Hazard specific location"].values
# 	location = np.array_str(location)            # Convert to numpy str
# 	location = location[3:-3]                    # Remove brackets
# 	location = np.array(location.split(','))     # Split
# 	location = location.astype(np.float)         # Convert to float
# 	location = [location[0],location[1]] 
# 	hazard_pos = convert_to_game_coordinates(AV_pos,AV_yaw,AV_length,location) 
	
# 	orientation_description = test_details["Hazard orientation"].values
# 	hazard_orientation = get_orientation(orientation_description) + AV_yaw
# 	print(hazard_orientation)

# 	h_type = test_details["Hazard type"].values
# 	hazard_type = get_type(h_type)

# 	return hazard_pos,hazard_orientation,hazard_type

def convert_to_game_coordinates(pod_manoeuvre,grid_origin,grid_orientation,hazard_pos_on_grid):
	x0     = grid_origin[0]
	y0     = grid_origin[1]
	theta0 = grid_orientation+np.pi/2
	l0     = 0 # offset
	polar_offset_from_rect_grid = -5

	if pod_manoeuvre == "Straight line":
		hazard_pos_on_grid = np.array_str(hazard_pos_on_grid)            # Convert to numpy str
		hazard_pos_on_grid = hazard_pos_on_grid[3:-3]                    # Remove brackets
		hazard_pos_on_grid = np.array(hazard_pos_on_grid.split(','))     # Split
		hazard_pos_on_grid = hazard_pos_on_grid.astype(np.float)         # Convert to float
		# hazard_pos_on_grid[0] = 2
		# hazard_pos_on_grid[1] = 2.5
		print("hazard_pos_on_grid = ",hazard_pos_on_grid)
		hazard_pos_on_grid = [hazard_pos_on_grid[0],hazard_pos_on_grid[1]] 
		# the if function below is to cater for loughborough inconsistent test grid
		if hazard_pos_on_grid[0] >= -1 and hazard_pos_on_grid[0] <= 1:
			x1     = hazard_pos_on_grid[0] * 0.8
		elif hazard_pos_on_grid[0] < -1:
			x1     =  -0.8 + (hazard_pos_on_grid[0]+1)
		elif hazard_pos_on_grid[0] > 1:
			x1     =   0.8 + (hazard_pos_on_grid[0]-1) 
		y1     = hazard_pos_on_grid[1] 


		# Converting from lboro rectangualr gird to Unreal game coordinates
		print("x1 = ", x1)
		print("y1 = ", y1)
		theta1 = np.arctan2(x1,y1)
		l1     = np.sqrt(x1**2 + y1**2)

		hazard_posx =   x0 \
                     + (l0/2) * np.cos(np.pi/2 - theta0) \
                     + l1 * np.cos(np.pi / 2 - theta0 - theta1)             
		hazard_posy =   y0 \
                     + (l0/2) * np.sin(np.pi / 2 - theta0) \
                     + l1 * np.sin(np.pi / 2 - theta0 - theta1)  
		print([hazard_posx, hazard_posy])

	if pod_manoeuvre == "Turning left":
		hazard_pos_on_grid = np.array_str(hazard_pos_on_grid)            # Convert to numpy str
		print("hazard_pos_on_grid = ",hazard_pos_on_grid)
		letter = hazard_pos_on_grid[2]                    # Extract letter component of hazard_pos_on_grid
		print("letter = ",letter)
		number = float(hazard_pos_on_grid[3:-2])                 # Extract number component of hazard_pos_on_grid
		print("number = ",number)

		# processing letter: each letter is worth pi/14
		if letter == "A":
			polar_angle    = 7*np.pi/14 

		elif letter == "B":
			polar_angle    = 6*np.pi/14

		elif letter == "C":
			polar_angle    = 5*np.pi/14

		elif letter == "D":
			polar_angle    = 4*np.pi/14

		elif letter == "E":
			polar_angle    = 3*np.pi/14

		elif letter == "F":
			polar_angle    = 2*np.pi/14

		elif letter == "G":
			polar_angle    = 1*np.pi/14

		elif letter == "H":
			polar_angle    = 0

		else:
			polar_angle    = 0


		# processing number : distance on lboro grid 0-1 = 2.2m | 1-2 = 1m | 2-3 = 1m | 3-4 = 0.8m | 4-5 = 0.8m | 5-6 = 1m | 6-7 = 1m
		if number <= 1:
			polar_distance   = number * 2.2  

		elif number > 1 and number <= 2:
			polar_distance   = (number-1) * 1 + 2.2

		elif number > 2 and number <= 3:
			polar_distance   = (number-2) * 1 + 2.2 + 1

		elif number > 3 and number <= 4:
			polar_distance   = (number-3) * 0.8 + 2.2 + 1 + 1

		elif number > 4 and number <= 5:
			polar_distance   = (number-4) * 0.8 + 2.2 + 1 + 1 + 0.8

		elif number > 5 and number <= 6:
			polar_distance   = (number-5) * 1 + 2.2 + 1 + 1 + 0.8 + 0.8

		elif number > 6 and number <= 7:
			polar_distance   = (number-6) * 1 + 2.2 + 1 + 1 + 0.8 + 0.8 + 1

		elif number > 7:
			polar_distance   = (number-7) * 1 + 2.2 + 1 + 1 + 0.8 + 0.8 + 1 + 1

		# polar_distance = 3.5##################### remove these
		# polar_angle    = np.pi/14#############

		# Convert polar coordinates to rectangular coordinates using lboro rectangular grid as the origin. 
		hazard_posx = x0 + polar_offset_from_rect_grid + polar_distance * np.cos(polar_angle)
		hazard_posy = y0  - polar_distance * np.sin(polar_angle)#- polar_distance * np.sin(polar_angle-np.pi/2) # The offset is to align the polar origin with the recatgular grid origin
		print("polar angle", polar_angle)
		print("to seeeeeee", polar_distance * np.cos(polar_angle))

		# hazard_pos_on_grid = np.array(hazard_pos_on_grid.split(','))     # Split
		# print(hazard_pos_on_grid)
		# hazard_pos_on_grid = hazard_pos_on_grid.astype(np.float)         # Convert to float
		# hazard_pos_on_grid = [hazard_pos_on_grid[0],hazard_pos_on_grid[1]] 
		# # the if function below is to cater for loughborough inconsistent test grid
		# if hazard_pos_on_grid[0] >= -1 and hazard_pos_on_grid[0] <= 1:
		# 	x1     = hazard_pos_on_grid[0] * 0.8
		# elif hazard_pos_on_grid[0] < -1:
		# 	x1     =  -0.8 + (hazard_pos_on_grid[0]+1)
		# elif hazard_pos_on_grid[0] > 1:
		# 	x1     =   0.8 + (hazard_pos_on_grid[0]-1) 
		# y1     = hazard_pos_on_grid[1] 

	# # Converting from lboro rectangualr gird to Unreal game coordinates
	# print("x1 = ", x1)
	# print("y1 = ", y1)
	# theta1 = np.arctan2(x1,y1)
	# l1     = np.sqrt(x1**2 + y1**2)

	# hazard_posx =   x0 \
 #                 + (l0/2) * np.cos(np.pi/2 - theta0) \
 #                 + l1 * np.cos(np.pi / 2 - theta0 - theta1)             
	# hazard_posy =   y0 \
 #                 + (l0/2) * np.sin(np.pi / 2 - theta0) \
 #                 + l1 * np.sin(np.pi / 2 - theta0 - theta1)  
	print("[hazard_posx, hazard_posy] = ",[hazard_posx, hazard_posy])

	return [hazard_posx, hazard_posy]
	# return [x1, y1]

def get_hazard(test_details,pod_manoeuvre,grid_origin,grid_orientation):
	# if pod_manoeuvre == "Straight line":
	location = test_details["Hazard specific location"].values
	hazard_pos = convert_to_game_coordinates(pod_manoeuvre,grid_origin,grid_orientation,location) 
	# print("hazard_pos_lboro",location)
	# print("hazard_pos",hazard_pos)
	orientation_description = test_details["Hazard orientation"].values
	hazard_orientation = get_orientation(orientation_description) + grid_orientation
	# print(hazard_orientation)


	h_type = test_details["Hazard type"].values
	hazard_type = get_type(h_type)

	return hazard_pos,hazard_orientation,hazard_type


def get_type(h_type):
	if h_type == "5yo child pedestrian dummy":
		hazard_type = "kid"
	elif h_type == "Adult pedestrian":
		hazard_type = "adult"
	elif h_type == "Pushchair (empty)":
		hazard_type = "pushchair"
	elif h_type == "Trunki":
		hazard_type = "trunki"
	elif h_type == "Bicycle":
		hazard_type = "bicycle"

	return hazard_type

def get_orientation(orientation_description):
	if orientation_description == "Side on, as if crossing L->R":
		hazard_orientation = np.pi/2
	elif orientation_description == "45 degree angle, crossing UL->LR":
		hazard_orientation = np.pi/4
	elif orientation_description == "Side on":
		hazard_orientation = 0
	elif orientation_description == "Head on":
		hazard_orientation = 0
	elif orientation_description == "45 degree angle, crossing UR->LL":
		hazard_orientation = -np.pi/4

	return hazard_orientation


def create_grid(AV_pos,AV_yaw,AV_length):
	l0 = 0.4
	l1 = 1
	x_shift =   AV_pos[0] #+ (AV_length/2) * np.cos(np.pi/2 - AV_yaw)
	y_shift =   AV_pos[1] + (AV_length/2) #* np.sin(np.pi/2 - AV_yaw)
	small_box = Polygon([[-l0/2, l1/2],\
				[l0/2, l1/2],\
				[l0/2, -l1/2],\
				[-l0/2,-l1/2],\
				[-l0/2, l1/2]])

	box = Polygon([[-l1/2, l1/2],\
				[l1/2, l1/2],\
				[l1/2, -l1/2],\
				[-l1/2,-l1/2],\
				[-l1/2, l1/2]])

	grid_pol_list = []

	for j in range(5):
		for i in range(4):
			for k in [1,-1]:
				if i <= 1:
					pol = small_box
					trans_x = (i*l0 + l0/2)*k + x_shift

				if i > 1:	
					pol = box
					trans_x = (l0*2 + (i-2)*l1 + l1/2)*k + x_shift

				trans_y = l1/2 + l1*j + y_shift
				rotate  = np.pi - AV_yaw
				pol = affinity.translate(pol,trans_x,trans_y)
				pol = affinity.rotate(pol,np.rad2deg(-AV_yaw), origin=(AV_pos[0],AV_pos[1]))
				grid_pol_list.append(pol)

	return(grid_pol_list)





testing = False#True#False
# testing = True#True#False

if testing:
	file_name =  "Capri_tests.xlsx"
	sheet     =  "Static hazard tests"
	TestNo    =  136#2#133

	df           = pd.read_excel(io=file_name, sheet_name=sheet)
	test_details = df[df.TestNo  == TestNo]
	pod_manoeuvre = test_details["Pod manoeuvre"].values[0]
	hazard_pos_on_grid = test_details["Hazard specific location"].values
	grid_origin        = [0,0]
	grid_orientation   = 0

	x,y = convert_to_game_coordinates(pod_manoeuvre,grid_origin,grid_orientation,hazard_pos_on_grid)




	## PLOT##
	plt.cla()
	plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
	ax = plt.gca()

	#############REACTANGULAR#####
	l0 = 0.4
	l1 = 1
	unit_vector      = np.array([np.cos(grid_orientation),np.sin(grid_orientation)])
	unit_vector_orth = np.array([np.cos(grid_orientation+np.pi/2),np.sin(grid_orientation+np.pi/2)])

	# Centre line
	p_start          = grid_origin
	p_end            = l1*5*unit_vector + grid_origin
	begin = [p_start[0],p_start[1]]#carla.Location(x=float(p_start[0]), y=float(p_start[1]), z=float(self.spawn_height/2))
	end   = [p_end[0],p_end[1]]#carla.Location(x=float(p_end[0]), y=float(p_end[1]), z=float(self.spawn_height/2))
	plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='black');

	# Drawing vertical lines
	for i in range(5):
		for k in [1,-1]:
			if i <= 2:
				p_start          = unit_vector_orth*l0*i*k + grid_origin
				p_end            = unit_vector_orth*l0*i*k + (l1*5*unit_vector + grid_origin)
			else:
				p_start          = unit_vector_orth*l1*(i-2)*k + unit_vector_orth*l0*2*k + grid_origin
				p_end            = unit_vector_orth*l1*(i-2)*k + unit_vector_orth*l0*2*k + (l1*5*unit_vector + grid_origin)


			begin = [p_start[0],p_start[1]]#carla.Location(x=float(p_start[0]), y=float(p_start[1]), z=float(self.spawn_height/2))
			end   = [p_end[0],p_end[1]]#carla.Location(x=float(p_end[0]), y=float(p_end[1]), z=float(self.spawn_height/2))
			plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='black');

	# Drawing horizontal lines
	for i in range(6):
		p_start          = unit_vector*l1*i + (grid_origin + (l0*2 + l1*2)*unit_vector_orth)
		p_end            = unit_vector*l1*i + (grid_origin - (l0*2 + l1*2)*unit_vector_orth)


		begin = [p_start[0],p_start[1]]#carla.Location(x=float(p_start[0]), y=float(p_start[1]), z=float(self.spawn_height/2))
		end   = [p_end[0],p_end[1]]#carla.Location(x=float(p_end[0]), y=float(p_end[1]), z=float(self.spawn_height/2))
		plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='black');


	#############POLAR#####
	polar_grid_len = 7.8
	angle_inc = -np.pi/14
	grid_angle_start = 0
	grid_angle_end   = -np.pi/2
	for angle in np.arange(grid_angle_start,grid_angle_end, angle_inc):
		angle = angle + grid_orientation
		
		# plotting the lines
		offset_from_rect_grid = 5
		begin = [0,offset_from_rect_grid]
		end   = [polar_grid_len*np.cos(angle),polar_grid_len*np.sin(angle)+offset_from_rect_grid]
		plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='red');

		# plotting the circles
		for r in [2.2,3.2,4.2,5,5.8,6.8,7.8]:
			begin = [r*np.cos(angle),r*np.sin(angle)+offset_from_rect_grid]
			end   = [r*np.cos(angle+angle_inc),r*np.sin(angle+angle_inc)+offset_from_rect_grid]
			plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='red');

	begin = [0,offset_from_rect_grid]
	end   = [polar_grid_len*np.cos(angle+angle_inc),polar_grid_len*np.sin(angle+angle_inc)+offset_from_rect_grid]
	plt.plot([begin[0],end[0]],[begin[1],end[1]], '-', color='red');


	# plot hazart pos
	plt.plot(x,y, 'x', color='green');


	plt.grid()
	ax.set_aspect('equal')
	plt.pause(10)























# AV_pos       = [-10.,-15.5]
# AV_yaw       = 3*np.pi/4
# AV_length    = 3.667
# AV_width     = 1.605
# dim          = np.array([AV_width,AV_length]) 
# AV_pol       = Polygon([[-dim[0]/2, dim[1]/2],\
# 				[dim[0]/2, dim[1]/2],\
# 				[dim[0]/2, -dim[1]/2],\
# 				[-dim[0]/2,-dim[1]/2],\
# 				[-dim[0]/2, dim[1]/2]])




# hazard_pos,hazard_orientation,hazard_type = get_hazard(test_details,AV_pos,AV_yaw,AV_length)

# AV_pol = affinity.translate(AV_pol,AV_pos[0],AV_pos[1])
# AV_pol = affinity.rotate(AV_pol,np.rad2deg(-AV_yaw), 'center')
# grid = create_grid(AV_pos,AV_yaw,AV_length) 


# plt.cla()
# plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
# ax = plt.gca()

# # Plot AV
# ax.add_patch(PolygonPatch(AV_pol, fc='grey', alpha=0.9, ec='none',label='AV'))
# plt.plot(AV_pos[0], AV_pos[1], 'o', color='black');

# # Plot grid in front of AV
# for pol in grid:
# 	plt.plot(*pol.exterior.xy, '-', color='black');

# # Plot static object
# plt.plot(hazard_pos[0], hazard_pos[1], 'o', color='black');


# plt.grid()
# ax.set_aspect('equal')
# plt.pause(10)
