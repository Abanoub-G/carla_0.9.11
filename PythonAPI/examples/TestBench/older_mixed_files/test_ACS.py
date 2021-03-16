# importing requests, json and time
import requests 
import json
import time

# import requests

URL_start = 'http://localhost:5000/start_sim'
URL_data = 'http://localhost:5000/sim_data'
URL_end = 'http://localhost:5000/terminate_sim'

headers = {'Content-type': 'application/json',}
data = '{"simulation_id": "SIM101", \
         "gps_log_file": "path.csv", \
         "spawn_location": [-6,-26,0.5,-88],\
         "dissable_sensors": 0}'

response = requests.post(url = URL_start, headers=headers, data=data)

print("start wait")
time.sleep(5)
print("end wait")

response = requests.post(url = URL_end)


# print("Waiting 5s for AV to spawn...")
# time.sleep(5)

# r = requests.get(url = URL_data) 

# while True:
    # sending get request and saving the response as response object 
    # r = requests.get(url = URL_data) 

    # # extracting data in json format 
    # data = r.json() 

    # # extrqacting the data and printing. Then sleep for 1 second
    # print("Distance: %s Bearing: %s Lat: %s Lng: %s CurrentWP: %s" % (data['DISTANCE_TO_WP'], data['RELATIVE_BEARING'], data['CURRENT_LAT'], data['CURRENT_LON'], data['CURRENT_WP']))
    # time.sleep(1)