# Path Generator

@authors: Joseph Story, Vasileious Tsoutsouras

This script and all generated paths is property of the Physical Computation Laboratory, Cambridge, and it is intended solely for use by the Physical Computation Laboratory.
If you would like to request permission to use this intellectual property, please contact vt298@eng.cam.ac.uk.

File names are given by waypointsX_payload_distribution_YZ where X is the number of waypoints, Y is the path type, and Z is the iteration out of 3.

Path Types:
 - Type A = Grid of x=0->480m, y=0->480m, with the depot at (0,0)
 - Type B = Grid of x=-480->480m, y=-480->480m, with the depot at (0,0)
 - Type C = The same as Type A but with the centre located on the CAPE Building, Cambridge

Distributions:
 - Uniform
 - Normal/Poisson?

Payloads:
 - Light = 0.1 to 0.2kg
 - Mid = 0.1 to 0.4kg
 - Heavy = 0.1 to 0.8kg




Set the home location of the drone in Gazebo:
 - export PX4_HOME_LAT=X.XXXX
 - export PX4_HOME_LON=X.XXXX
 - export PX4_HOME_ALT=X.XXXX

Coordinates:
 - CAPE Cambridge = 52.210444, 0.092640
 - ETH Zurich = 47.3977413, 8.5455941
