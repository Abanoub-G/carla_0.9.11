import os
import math
import numpy as np
import random
from scipy.interpolate import splev, splrep, interp1d

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


def write_file(x_list,y_list):
	# Folder "results" if not already there
	file_name = "path_straight.csv"
	# if not os.path.exists(output_folder):
	# 	os.makedirs(output_folder)

	file_path = os.path.join(file_name)
	with open(file_path, 'w') as log_file: 
		for i in range(len(x_list)):
			log_file.write('%3.14f, %3.14f\n' %\
				(x_list[i],y_list[i]))
	print('waypoints file SUCCESSFULLY generated!')




# Insert waypoints
x_list = [49.000338,49.000640]
y_list = [8.000006,8.000016]
num_points = 20
# interp res = 0.0000001
xvals = np.linspace(x_list[0], x_list[-1], num=num_points)
y_interp = np.interp(xvals, x_list, y_list)

# earth_radius = 6371000 #m

# # Opendrive geo reference values
# geo_ref_lat  = 49.0 
# geo_ref_lon  = 8.0

# # Convert to geo spatial coordinates
# lon = geo_ref_lon + (x_list/earth_radius)
# lat = geo_ref_lat - (y_list/earth_radius)

# Interpolate
# x_interp, y_interp = interp_wp_linear(x_list,y_list,interp_res)

# Write file
write_file(xvals,y_interp)