/**
 * @file fly_mission.cpp
 *
 * @brief Demonstrates how to Add & Fly Waypoint missions using the MAVSDK.
 * The example is summarised below:
 * 1. Adds mission items.
 * 2. Starts mission from first mission item.
 * 3. Illustrates Pause/Resume mission item.
 * 4. Exits after the mission is accomplished.
 *
 * @author Julian Oes <julian@oes.ch>,
 *         Shakthi Prashanth M <shakthi.prashanth.m@intel.com>
 * @date 2017-09-06
 */

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
//#include "input.h"

#include <cmath>
//extern const int num_waypoints;

////Set the number of waypoints
//int set_num_waypoints(){
//    const int num_waypoints = 4;
//    return num_waypoints;
//}


////Define the list of Waypoints
//WAYPOINT set_waypoints(){

//    //Takeoff Waypoint
//    WAYPOINT TAKEOFF;
//    TAKEOFF.identifier = 1;
//    TAKEOFF.lat = 0;
//    TAKEOFF.lon = 0;
//    TAKEOFF.altitude = 0;

//    //Mission Waypoint A
//    WAYPOINT A;
//    A.identifier = 2;
//    A.lat = 47.3977759; //47.398241338125118
//    A.lon = 8.5462028; //8.5455360114574432
//    A.altitude = 10.0f;

//    //Mission Waypoint B
//    WAYPOINT B;
//    B.identifier = 3;
//    B.lat = 47.3980161; //47.398001890458097
//    B.lon = 8.5453055; //8.5455576181411743
//    B.altitude = 10.0f;

//    //Mission Waypoint C
//    WAYPOINT C;
//    C.identifier = 4;
//    C.lat = 47.3980003; //47.398058617228855
//    C.lon = 8.5459517; //8.5454618036746979
//    C.altitude = 10.0f;

//    //Mission Waypoint D
//    WAYPOINT D;
//    D.identifier = 5;
//    D.lat = 47.398008;
//    D.lon = 8.545701;
//    D.altitude = 10;

//    WAYPOINT waypoint_array[num_waypoints+1] = {TAKEOFF, A, B, C, D};

//    return waypoint_array[0];

//}

const double earth_radius = 6371000; //metres;
const double pi = 3.1415926535897;

double distance(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2)
{
    double lat1_rad = (lat1/180)*pi;
    double lat2_rad = (lat2/180)*pi;
    double lon1_rad = (lon1/180)*pi;
    double lon2_rad = (lon2/180)*pi;

    double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));
    return x;
}
