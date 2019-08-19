import numpy as np
import math

earth_radius = 6371000

#Number of waypoints
size = 8

origin_lat = 47.3977413
origin_lon = 8.5455941

#Boundaries (x and y)
lat_upper_bound_x = 400
lat_lower_bound_x = 0
lon_upper_bound_y = 400
lon_lower_bound_y = 0

#Waypoint parameters
payload_upper_bound = 0.3 #grams
payload_lower_bound = 0.1 #grams

lat_x = lat = np.random.uniform(lat_lower_bound_x, lat_upper_bound_x, size)
lon_y = np.random.uniform(lon_lower_bound_y, lon_upper_bound_y, size)
payload = (np.random.uniform(payload_lower_bound, payload_upper_bound, size))
deadline = (np.random.uniform(10*size, 50*size, size))
lat = []
lon = []

#Convert x,y coordinates into latitude and longitude
for i in range(0,size):
    lat_tmp = origin_lat + (((lat_x[i])/(earth_radius))*(180/math.pi))
    lon_tmp = origin_lon + (((lon_y[i])/(earth_radius))*(180/math.pi))
    lat.append(lat_tmp)
    lon.append(lon_tmp)

#Round to suitable decimal places
for i in range(0,size):
    deadline[i] = round(deadline[i], 1)
    lat[i] = round(lat[i], 8)
    lon[i] = round(lon[i], 8)
    payload[i] = round(payload[i], 4)

print("\nLat")
print(lat)
print("\nLon")
print(lon)
print("\nLat X")
print(lat_x)
print("\nLon Y")
print(lon_y)

with open("input.txt", "w+") as text_file:
    text_file.write("%s 1.380 4 0.12 5350 15.2 5 1 0.95" % size)
    for i in range(0,(size)):
        text_file.write("\n{} {} {} {} {} {} {} {}".format(i+1, str(unichr(65+i)), lat[i], lon[i], 20.0, 5, deadline[i], payload[i]))

# 1  A    47.39869 8.5460885 20.0000 5     10.0     0.1
# ID User Latitude Longitude Altiude Speed Deadline Payload
