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

#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <cmath>

#include <algorithm>
#include <map>
#include <string>
#include <set>
#include <functional>
#include <fstream>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

using namespace mavsdk;
using namespace std::placeholders; // for `_1`
using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

double lat_A;
double lon_A;
float altitude_A;
double lat_B;
double lon_B;
float altitude_B;
double lat_C;
double lon_C;
float altitude_C;
double lat_takeoff;
double lon_takeoff;
float altitude_absolute_takeoff;
double earth_radius = 6371000; //metres;
const double pi = 3.1415926535897;

// Handles Action's result
inline void handle_action_err_exit(Action::Result result, const std::string &message);
// Handles Mission's result
inline void handle_mission_err_exit(Mission::Result result, const std::string &message);
// Handles Connection result
inline void handle_connection_err_exit(ConnectionResult result, const std::string &message);


void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

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

int completed[10] = {0}, n, cost = 0;
int ary[10][10] = {

    { 0, 3, 4, 1,  0, 0, 0, 0, 0, 0},
    { 3, 0, 5, 6,  0, 0, 0, 0, 0, 0},
    { 4, 3, 0, 5,  0, 0, 0, 0, 0, 0},
    { 4, 6, 5, 0,  0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0, 0, 0}

};

//Takes cost matrix from user
void takeInput(){

//    int i, j;

//    std::cout << "Enter the number of waypoints";
//    std::cin >> n;

//    std::cout << "Enter distance matrix";

//    for (i=0; i<n; i++){
//        std::cout << "\nEnter Elements of Row: " << i+1 << "\n";

//        for (j=0; j< n; j++){
//            std::cin >> ary[i][j];
//        }
//        completed[i]=0;
//    }

//    std::cout << "n\nThe cost list is:";

//    for (i=0; i<n; i++){
//        std::cout << "\n";

//        for (j=0; j<n; j++){
//            std::cout << "\t" << ary[i][j];
//        }
//    }
}


//Finds the nearest neighbour that hasn't been visited?
int least(int c){
    int i,nc=999;
    int min=999,kmin;

    for (i=0;i<n;i++){
        if((ary[c][i]!=0)&&(completed[i]==0)){
            if(ary[c][i]+ary[i][c] < min){
                min = ary[i][0]+ary[c][i];
                kmin=ary[c][i];
                nc=i;
            }
        }
    }
    if(min!=999){
        cost+=kmin;
    }
    return nc;
}

//Finds the minimum cost route using the nearest neighbour algorithm
void mincost(int city){
    int ncity;

    completed[city]=1;

    std::cout << city+1 << "--->";
    ncity = least(city);

    if(ncity==999){
        ncity=0;
        std::cout << ncity+1;
        cost+=ary[city][ncity];
        return;
    }
    mincost(ncity);

}

typedef struct {
    int identifier;
    double lat;
    double lon;
    float altitude;
    float speed;
}WAYPOINT;

typedef struct {
    int identifier; //Char?
    double distance;
    int num_waypoints;
    WAYPOINT traj[3];
}ROUTE;

//Function Prototype
void sortArray(ROUTE array[], int size);

int main()
{
    //Mavsdk dc;

    takeInput();

    n = 4;

    mincost(0);

    std::cout << "\n\nMinimum cost is " << cost << std::endl;
    std::cout << "" << std::endl;

    //float speed = 5.0f;


    //Takeoff Waypoint
    WAYPOINT TAKEOFF;
    TAKEOFF.identifier = 0;
    TAKEOFF.lat = 47.398241;
    TAKEOFF.lon = 8.5455360;
    TAKEOFF.altitude = 0.0f;

    //Mission Waypoint A
    WAYPOINT A;
    A.identifier = 1;
    A.lat = 47.3977759; //47.398241338125118
    A.lon = 8.5462028; //8.5455360114574432
    A.altitude = 10.0f;

    //Mission Waypoint B
    WAYPOINT B;
    B.identifier = 2;
    B.lat = 47.3980161; //47.398001890458097
    B.lon = 8.5453055; //8.5455576181411743
    B.altitude = 10.0f;

    //Mission Waypoint C
    WAYPOINT C;
    C.identifier = 3;
    C.lat = 47.3980003; //47.398058617228855
    C.lon = 8.5459517; //8.5454618036746979
    C.altitude = 10.0f;

    //Mission Waypoint D
    WAYPOINT D;
    D.identifier = 4;
    D.lat = 47.38568;
    D.lon = 8.553201;
    D.altitude = 10;

    int num_waypoints = 4;

    //takeInput();

    //Declare a list of waypoints to work with
    WAYPOINT waypoint_array[num_waypoints+1] = {TAKEOFF, A, B, C, D};

    //Calculate the 2D distance array (spherical polar coordinates)
    double distance_array[num_waypoints+1][num_waypoints+1];
    for (int i = 0; i < num_waypoints+1; i++){
        for (int t = 0; t < num_waypoints+1; t++){
            distance_array[i][t] = distance(waypoint_array[i].altitude, waypoint_array[t].altitude, waypoint_array[i].lat, waypoint_array[t].lat, waypoint_array[i].lon, waypoint_array[t].lon);
        }
    }


    //Print the 2D distance array
    std::cout << "2D Distance Array:" << std::endl;
    std::cout << "" << std::endl;
    for (int i = 0; i < num_waypoints+1; i++){
        std::cout << "[";
        for (int t = 0; t < num_waypoints+1; t++){
            std::cout << distance_array[i][t] << ",  ";
        }
        std::cout << "]" << std::endl;
    }



    std::cout << "" << std::endl;


//    ROUTE ABC;
//    ABC.identifier = 38;
//    ABC.distance = route_abc;
//    ABC.num_waypoints = 3;
//    ABC.traj[0] = A;
//    ABC.traj[1] = B;
//    ABC.traj[2] = C;


//    const int SIZE = 1;
//    ROUTE array[SIZE];
//    array[0].identifier = ABC.identifier;
//    array[0].distance = ABC.distance;
//    array[0].num_waypoints = ABC.num_waypoints;
//    array[0].traj[0] = ABC.traj[0];
//    array[0].traj[1] = ABC.traj[1];
//    array[0].traj[2] = ABC.traj[2];



//    sortArray(array, SIZE);


//    for (int i = 0; i < SIZE; i++)
//    {
//        std::cout << array[i].identifier << std::endl;
//    }




    sleep_for(seconds(4));


}


inline void handle_action_err_exit(Action::Result result, const std::string &message)
{
    if (result != Action::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

inline void handle_mission_err_exit(Mission::Result result, const std::string &message)
{
    if (result != Mission::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Mission::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void handle_connection_err_exit(ConnectionResult result, const std::string &message)
{
    if (result != ConnectionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

void sortArray(ROUTE array[], int size)
{
    bool swapped;

    do
    {
        swapped = false;
        for (int count = 0; count < (size - 1); count++)
        {
            if (array[count].distance > array[count + 1].distance)
            {
                std::swap(array[count], array[count+1]);
                swapped = true;
            }
        }
    } while (swapped);
}
