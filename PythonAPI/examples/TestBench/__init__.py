import logging
import carla 

HOST = '127.0.0.1'
PORT = 2000


logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

client = carla.Client(HOST, PORT)
client.set_timeout(2.0)


#=============================================
#===Connecting to Simulator
#=============================================
print("PROGRESS: Connecting to Simulator, please wait for me! Will notify you when I am done :)")

world = client.get_world()
settings = world.get_settings()

settings.fixed_delta_seconds = 0.05
settings.synchronous_mode = True
world.apply_settings(settings)

# @todo cannot import these directly.
SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor

print("PROGRESS: Connected to Simulator! :)")

