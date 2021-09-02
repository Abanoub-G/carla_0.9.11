import glob
import os
import sys
import numpy as np

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# Script level imports
sys.path.append('../../carla')
from agents.navigation.roaming_agent import RoamingAgent
from agents.navigation.basic_agent import BasicAgent

############################

from TB_common_functions import Ticking

# Import environments
# from Environments.AV_Testing import AV_Testing
# from Environments.ConstraintRandom import ConstraintRandom
from Environments.PedNavTest import PedNavTest
from Environments.Assertions_case_study import Assertions_case_study_overtaking
from Environments.Assertions_case_study import Assertions_case_study_left_turn_at_intersection
from Environments.Assertions_case_study import Assertions_case_study_right_turn_at_intersection_pedestrian_crossing
from Environments.GameRunTown3 import GameRun
# from Environments.lboro_tests.lboro_tests_env import static_and_dyanmic_tests

import __init__
from __init__ import client, world, settings


def main():

	try:

		world.set_weather(carla.WeatherParameters.ClearNoon)

		print("PROGRESS: Weather conditions set  :)")


		

		frame  = None
		world.tick()

		# Setting up environment
		# env = AV_Testing()
		# env = PedNavTest()
		# env = static_and_dyanmic_tests(world,"static")
		# env.set_test(1)
		# env = Assertions_case_study_overtaking(world)
		# env = Assertions_case_study_left_turn_at_intersection(world)
		# env = Assertions_case_study_right_turn_at_intersection_pedestrian_crossing(world)
		env = GameRun(world)
		# ===Set 
		# env.set()

		
#========================================================================================================		
		# # Use this bit of code below to convert from game coordinates to geolocation 
		# current_map = world.get_map() #map obtained by world.get_map()
		# degreesToRadians = 1/180.0*np.pi
		# x = 1.1538
		# y = -71.2938
		# z = -0.057
		# geolocation_of_map_origin = current_map.transform_to_geolocation(carla.Location(x=float(x), y=float(y), z=float(z)))
		# print("GeoLocation of map origin",geolocation_of_map_origin)
		# world.debug.draw_string(carla.Location(x=float(x), y=float(y), z=float(z)), 'O', draw_shadow=False,
		# 		color=carla.Color(r=255, g=0, b=0), life_time=120.0,persistent_lines=True)
#==========================================================================================================			
		
		

		while True:
			env.set()####
			while True:
				#===Tick
				ts, frame = Ticking(world,frame)
				# print("Ticking")

				# ===Step 
				env.step() 
				
				if env.test_ended == True:
					env.destroy()
					ts, frame = Ticking(world,frame)
					break

			if env.tests_ended == True:
				break
				
				print("Tick")
			



	finally:

		# env.end_tests()
		# env.destroy_actors()
		# env.destroy()

		settings.synchronous_mode = False
		world.apply_settings(settings)


if __name__ == '__main__':

	try:
		main()
	except KeyboardInterrupt:
		pass
	finally:
		print('\ndone.')


