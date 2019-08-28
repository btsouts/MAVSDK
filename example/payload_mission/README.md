# Payload Mission

This program aims to take a list of waypoints from the path generator, and upload it to a drone running PX4. Drone and waypoint information is read from the input.txt file.

The input format in the input.txt file is:

	- 1st Line: Number of Waypoints, Drone Mass, Drone Number of Rotors, Drone Rotor Radius, Drone Battery Capacity, Drone Voltage, Drone Maximum Velocity, Drone Minimum Velocity, Drone Efficiency 

	- All other lines: Waypoint ID, User, Latitude, Longitude, Altitude, Speed, Deadline, Payload


Running the program:

	- To compile the program, navigate into the build folder and use the 'make' command.

	- To run the program using the simulator's UDP connection, use ./time_mission udp://:14540

To install MAVSDK, follow the instructions found at https://mavsdk.mavlink.io/develop/en/contributing/build.html
	
