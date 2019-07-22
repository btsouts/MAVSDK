#ifndef INPUT_H
#define INPUT_H

typedef struct {
    int identifier;
    double lat;
    double lon;
    float altitude;
    float speed;
}WAYPOINT;

//WAYPOINT set_waypoints();
//int set_num_waypoints();


WAYPOINT TAKEOFF = {1, 47.39791, 8.5458203, 10.0f, 0};
WAYPOINT A = {2, 47.3977759, 8.5462028, 10.0f, 5.0f};
WAYPOINT B = {3, 47.3980161, 8.5453055, 10.0f, 5.0f};
WAYPOINT C = {4, 47.3980003, 8.5459517, 10.0f, 4.0f};
WAYPOINT D = {5, 47.398008, 8.545701, 10.0f, 5.0f};

const int num_waypoints = 4;

WAYPOINT waypoint_array[num_waypoints+1] = {TAKEOFF, A, B, C, D};

double distance(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2);

#endif
