This program aims to take a list of waypoints, each with a specific deadline, and create a flight trajectory that minimises the number of missed deadlines. Drone and waypoint information is read from the input.txt file.

The input format in the input.txt file is:

        - 1st Line: Number of Waypoints, Drone Weight, Drone Energy, Drone Maximum Velocity, Drone Minimum Velocity, Drone Efficiency

	- All other lines: Waypoint ID, User, Latitude, Longitude, Altitude, Speed, Deadline, Payload


Running the program:

	- To compile the program, navigate into the build folder and use the 'make' command.

	- To run the program using the simulator's UDP connection, use ./time_mission udp://:14540

