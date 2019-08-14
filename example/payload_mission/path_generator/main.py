import numpy as np

# Input application here
lat_upper_bound = 47.9
lat_lower_bound = 47.7
lon_upper_bound = 8.55
lon_lower_bound = 8.53
payload_upper_bound = 0.3 #grams
payload_lower_bound = 0.1 #grams

size = 14

lat = np.random.uniform(lat_lower_bound, lat_upper_bound, size)
lon = np.random.uniform(lon_lower_bound, lon_upper_bound, size)
payload = (np.random.uniform(payload_lower_bound, payload_upper_bound, size))
for i in range(0,size):
    payload[i] = round(payload[i], 4)

print "\nLat"
print lat
print "\nLon"
print lon

with open("Downloads/MAVSDK/example/payload_mission/path_generator/input.txt", "w+") as text_file:
    text_file.write("%s 1.380 4 0.12 5350 15.2 5 1 0.95" % size)
    for i in range(0,(size)):
        text_file.write("\n{} {} {} {} {} {} {} {}".format(i+1, str(unichr(65+i)), lat[i], lon[i], 20.0, 5, 10.0, payload[i]))

# 1 A 47.3986943 8.5460885 20.0000000 5 10.0 0.1
