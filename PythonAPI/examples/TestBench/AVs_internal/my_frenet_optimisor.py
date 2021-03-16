import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splev, splrep, interp1d
from shapely.geometry import Polygon
from shapely import affinity
from descartes import PolygonPatch


def interp_wp_linear(x,y,interp_res):
	f = interp1d(x, y)
	x_interp = np.arange(min(x), max(x)+interp_res, interp_res)
	y_interp = f(x_interp)

	return x_interp, y_interp

def interp_wp_cubic(x,y,interp_res):
	f = interp1d(x, y, kind='cubic')
	x_interp = np.arange(min(x), max(x)+interp_res, interp_res)
	y_interp = f(x_interp)

	return x_interp, y_interp

def interp_wp_spline(x,y,interp_res):
	spl = splrep(x, y)
	x_interp = np.arange(min(x), max(x)+interp_res, interp_res)
	y_interp = splev(x_interp, spl)

	return x_interp, y_interp


safety_offet = 0.5
safety_offetX2 = safety_offet * 2 # taking account of both sides

veh_sides = np.array([1.9,5]) 
veh_yaw   = 0
veh_pos     = np.array([-5,-7])
veh_vel_vec = np.array([0,4])

ped_sides = np.array([0.8,0.4]) 
ped_yaw   = 0
ped_pos     = np.array([0.,4.])
ped_vel_vec = np.array([-1.5,-1.5])

wp_x = np.array([-5,-5,-5])
wp_y = np.array([-5, 5,11])
#################################
road_width = 5
path_offset_res = 0.5

# wp_x = np.array([0,20])
# wp_y = np.array([0,20])

delta_x = wp_x[-1] - wp_x[0] 
delta_y = wp_y[-1] - wp_y[0]

L = np.sqrt(delta_x**2 + delta_y**2)  

adjustment_factor = path_offset_res / L 

x_offset =  delta_y * adjustment_factor
y_offset = -delta_x * adjustment_factor


##################################

# Defining Polygons ====================================================
veh_pol = Polygon([[-veh_sides[0]/2, veh_sides[1]/2],\
				[veh_sides[0]/2, veh_sides[1]/2],\
				[veh_sides[0]/2, -veh_sides[1]/2],\
				[-veh_sides[0]/2,-veh_sides[1]/2],\
				[-veh_sides[0]/2, veh_sides[1]/2]])

veh_sides_safety = np.array([veh_sides[0]+safety_offetX2,veh_sides[1]+safety_offetX2])

veh_pol_safe = Polygon([[-veh_sides_safety[0]/2, veh_sides_safety[1]/2],\
				[veh_sides_safety[0]/2, veh_sides_safety[1]/2],\
				[veh_sides_safety[0]/2, -veh_sides_safety[1]/2],\
				[-veh_sides_safety[0]/2,-veh_sides_safety[1]/2],\
				[-veh_sides_safety[0]/2, veh_sides_safety[1]/2]])




ped_pol = Polygon([[-ped_sides[0]/2, ped_sides[1]/2],\
				[ped_sides[0]/2, ped_sides[1]/2],\
				[ped_sides[0]/2, -ped_sides[1]/2],\
				[-ped_sides[0]/2,-ped_sides[1]/2],\
				[-ped_sides[0]/2, ped_sides[1]/2]])

ped_sides_safety = np.array([ped_sides[0]+safety_offetX2,ped_sides[1]+safety_offetX2])

ped_pol_safe = Polygon([[-ped_sides_safety[0]/2, ped_sides_safety[1]/2],\
				[ped_sides_safety[0]/2, ped_sides_safety[1]/2],\
				[ped_sides_safety[0]/2, -ped_sides_safety[1]/2],\
				[-ped_sides_safety[0]/2,-ped_sides_safety[1]/2],\
				[-ped_sides_safety[0]/2, ped_sides_safety[1]/2]])



# Moving Polygons ========================================================
veh_yaw = -np.arctan2(veh_vel_vec[0],veh_vel_vec[1]) + veh_yaw
veh_yaw = -np.arctan2(wp_x[1] - wp_x[0],wp_y[1] - wp_y[0]) + veh_yaw
veh_pol = affinity.translate(veh_pol,veh_pos[0],veh_pos[1])
veh_pol = affinity.rotate(veh_pol,np.rad2deg(veh_yaw), 'center')
veh_pol_safe = affinity.translate(veh_pol_safe,veh_pos[0],veh_pos[1])
veh_pol_safe = affinity.rotate(veh_pol_safe,np.rad2deg(veh_yaw), 'center')


ped_yaw = -np.arctan2(ped_vel_vec[0],ped_vel_vec[1]) + ped_yaw
ped_pol = affinity.translate(ped_pol,ped_pos[0],ped_pos[1])
ped_pol = affinity.rotate(ped_pol,np.rad2deg(ped_yaw), 'center')
ped_pol_safe = affinity.translate(ped_pol_safe,ped_pos[0],ped_pos[1])
ped_pol_safe = affinity.rotate(ped_pol_safe,np.rad2deg(ped_yaw), 'center')


# Predicitng ===================================
prediction_distance_max = 15
time_increment = 0.1

v_mag = 4
look_ahead = 0.5
dis_to_dest = np.inf

wp_x_current = wp_x
wp_y_current = wp_y

veh_yaw_current = veh_yaw
veh_yaw_new     = veh_yaw

prediction_distance  = 0
t = 0
ped_pos_new = ped_pos
veh_pos_new = veh_pos
i = 0
dis_to_dest = np.inf

j = 0
reset = False
k = 0

while dis_to_dest > look_ahead:
	
	if reset:
		k += 1 
		prediction_distance  = 0
		t = 0
		ped_pos_new = ped_pos
		veh_pos_new = veh_pos
		i = 0
		dis_to_dest = np.inf

		if (k % 2) != 0: 
			j += 1

		j *= -1
		wp_x_current = wp_x + x_offset * j
		wp_y_current = wp_y + y_offset * j

		temp_array  = np.array([wp_x_current[0],wp_y_current[0]])
		temp_array1 = np.array([wp_x_current[1],wp_y_current[1]])
		temp_array  = temp_array + 0.3 * (temp_array1 - temp_array) 

		wp_x_current[0] = temp_array[0]
		wp_y_current[0] = temp_array[1]

		wp_x_current = np.insert(wp_x_current, 0, veh_pos[0], axis=0)
		wp_y_current = np.insert(wp_y_current, 0, veh_pos[1], axis=0)
		reset = False

	t += time_increment 

	# detecting which waypoints to use
	diff_pos = veh_pos_new - np.array([wp_x_current[i+1],wp_y_current[i+1]])
	diff_pos_mag = np.hypot(diff_pos[0],diff_pos[1])

	dis_to_dest_vec = veh_pos_new - np.array([wp_x_current[-1],wp_y_current[-1]])
	dis_to_dest = np.hypot(dis_to_dest_vec[0],dis_to_dest_vec[1])

	if diff_pos_mag < look_ahead:
		i += 1
	veh_yaw =  -np.arctan2(wp_y_current[i+1] - wp_y_current[i],wp_x_current[i+1] - wp_x_current[i]) + np.pi / 2 

	# if np.absolute(veh_yaw_new - veh_yaw) != 0
	# 	veh_yaw = 

	# veh_yaw = - + veh_yaw_current
	veh_vel_vec = np.array([v_mag*np.cos(np.pi / 2 -veh_yaw),v_mag*np.sin(np.pi / 2 -veh_yaw)])



	#update vel vector
	ped_pos_pred = ped_pos_new + ped_vel_vec*time_increment
	ped_pol_pred = ped_pol_safe
	ped_trans    = ped_pos_pred - ped_pos 
	ped_pol_pred = affinity.translate(ped_pol_pred,ped_trans[0],ped_trans[1])

	veh_pos_pred = veh_pos_new + veh_vel_vec*time_increment
	veh_pol_pred = veh_pol_safe
	veh_trans    = veh_pos_pred - veh_pos 
	veh_pol_pred = affinity.translate(veh_pol_pred,veh_trans[0],veh_trans[1])

	prediction_distance = np.linalg.norm(veh_trans)

	ped_pos_new  = ped_pos_pred
	veh_pos_new  = veh_pos_pred

	if veh_pol_pred.intersects(ped_pol_pred):
		reset = True


	# Drawing ================================================================= 
	plt.cla()
	plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
	ax = plt.gca()
	ax.add_patch(PolygonPatch(veh_pol, fc='grey', alpha=0.5, ec='none',label='Main boundary'))
	ax.add_patch(PolygonPatch(veh_pol_safe, fc='darkgrey', alpha=0.5, ec='none',label='Safety boundary'))
	plt.plot(ped_pos[0], ped_pos[1], 'o', color='black');
	plt.quiver(ped_pos[0], ped_pos[1], ped_vel_vec[0], ped_vel_vec[1], angles='xy', scale_units='xy', scale=1)

	ax.add_patch(PolygonPatch(ped_pol, fc='grey', alpha=0.5, ec='none'))
	ax.add_patch(PolygonPatch(ped_pol_safe, fc='darkgrey', alpha=0.5, ec='none'))
	plt.plot(veh_pos[0], veh_pos[1], 'o', color='black');
	plt.quiver(veh_pos[0], veh_pos[1], veh_vel_vec[0], veh_vel_vec[1], angles='xy', scale_units='xy', scale=1)

	plt.plot(*ped_pol_pred.exterior.xy, '--', color='grey', label='Predicted evelop');
	plt.plot(*veh_pol_pred.exterior.xy, '--', color='grey');

	plt.plot(wp_x_current, wp_y_current,'--',color='black', label='Path');

	plt.xlim(-10, 10)
	plt.ylim(-10,15)
	plt.grid()
	ax.legend(loc='upper right')
	plt.pause(0.1)
