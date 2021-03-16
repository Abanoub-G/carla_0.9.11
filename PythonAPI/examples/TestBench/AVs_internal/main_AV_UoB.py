import glob
import os
import sys

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

import __init__
from __init__ import client, world, settings

from environment_setup import Ticking
from environments import env_TB_ConstraintRandom, AV_Testing, PedNavTest


# import argparse
# import logging
# import random
# import numpy as np
# from numpy import zeros
# import math
# import csv
# import time


#=============================================
#===Declerations 
#=============================================

# HOST = '127.0.0.1'
# PORT = 2000


def main():

	# logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

	# client = carla.Client(HOST, PORT)
	# client.set_timeout(2.0)

	try:

		# #=============================================
		# #===Connecting to Simulator
		# #=============================================
		# print("PROGRESS: Connecting to Simulator, please wait for me! Will notify you when I am done :)")

		# world = client.get_world()
		# settings = world.get_settings()

		# settings.fixed_delta_seconds = 0.05
		# settings.synchronous_mode = True
		# world.apply_settings(settings)

		# # @todo cannot import these directly.
		# SpawnActor = carla.command.SpawnActor
		# SetAutopilot = carla.command.SetAutopilot
		# FutureActor = carla.command.FutureActor

		# print("PROGRESS: Connected to Simulator! :)")

		#=============================================
		#===Set Weather Conditions 
		#=============================================
		# weather = carla.WeatherParameters(
		# 	cloudyness=80.0,
		# 	precipitation=30.0,
		# 	sun_altitude_angle=70.0)
		# world.set_weather(weather)

		world.set_weather(carla.WeatherParameters.ClearNoon)

		print("PROGRESS: Weather conditions set  :)")


		

		frame  = None
		world.tick()

		# Setting up environment
		# env = AV_Testing()
		# env = PedNavTest()
		# env.set()


		while True:

			#===Tick
			ts, frame = Ticking(world,frame)
			print("Ticking")

			#===Step 
			# env.step() 



	finally:

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


