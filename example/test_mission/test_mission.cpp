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

//Cost function variables
double cost = 0; //Should be double
int n=0;
const int num_waypoints = 4;
double cost_array[(num_waypoints+1)][(num_waypoints+1)]; //Should be double

int completed[num_waypoints+1] = {0};

// Handles Action's result
inline void handle_action_err_exit(Action::Result result, const std::string &message);
// Handles Mission's result
inline void handle_mission_err_exit(Mission::Result result, const std::string &message);
// Handles Connection result
inline void handle_connection_err_exit(ConnectionResult result, const std::string &message);

static std::shared_ptr<MissionItem> make_mission_item(double latitude_deg,
                                                      double longitude_deg,
                                                      float relative_altitude_m,
                                                      float speed_m_s,
                                                      bool is_fly_through,
                                                      float gimbal_pitch_deg,
                                                      float gimbal_yaw_deg,
                                                      MissionItem::CameraAction camera_action);

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

WAYPOINT route_array[num_waypoints+1];


/*  CONNECT TO THE DRONE  */







//Function Prototype
void sortArray(ROUTE array[], int size);


//Finds the nearest neighbour that hasn't been visited
int least(int p){
    int i,np=999;
    int min=999,kmin;

    for (i=0;i<num_waypoints+1;i++){
        if((cost_array[p][i]!=0)&&(completed[i]==0)){
            if(cost_array[p][i]+cost_array[i][p] < min){
                min = cost_array[i][0]+cost_array[p][i];
                kmin=cost_array[p][i];
                np=i;
            }
        }
    }
    if(min!=999){
        cost+=kmin;
    }
    return np;
}

//Finds the minimum cost route using the nearest neighbour algorithm
void mincost(int position, WAYPOINT array[num_waypoints+1]){
    int nposition;

    completed[position]=1;

    std::cout << position+1 << "--->";

    route_array[n] = array[position];
    n++;

    nposition = least(position);

    if(nposition==999){
        nposition=0;
        std::cout << nposition+1;
        cost+=cost_array[position][nposition];
        return;
    }
    mincost(nposition, array);

}


int main(int argc, char **argv)
{
    Mavsdk dc;

    {
        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();

        std::cout << "Waiting to discover system..." << std::endl;
        dc.register_on_discover([prom](uint64_t uuid) {
            std::cout << "Discovered system with UUID: " << uuid << std::endl;
            prom->set_value();
        });

        std::string connection_url;
        ConnectionResult connection_result;

        if (argc == 2) {
            connection_url = argv[1];
            connection_result = dc.add_any_connection(connection_url);
        } else {
            usage(argv[0]);
            return 1;
        }

        if (connection_result != ConnectionResult::SUCCESS) {
            std::cout << ERROR_CONSOLE_TEXT
                      << "Connection failed: " << connection_result_str(connection_result)
                      << NORMAL_CONSOLE_TEXT << std::endl;
            return 1;
        }

        future_result.get();
    }

    dc.register_on_timeout([](uint64_t uuid) {
        std::cout << "System with UUID timed out: " << uuid << std::endl;
        std::cout << "Exiting." << std::endl;
        exit(0);
    });

    // We don't need to specifiy the UUID if it's only one system anyway.
    // If there were multiple, we could specify it with:
    // dc.system(uint64_t uuid);
    System &system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto mission = std::make_shared<Mission>(system);
    auto telemetry = std::make_shared<Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }

    std::cout << "System ready" << std::endl;
    std::cout << "" << std::endl;

//    //Get number of waypoints from user
//    int num_waypoints;
//    std::cout << "Enter number of waypoints: ";
//    std::cin >> num_waypoints;
//    std::cout << "Number of waypoints = " << num_waypoints << std::endl;

//    //Get speed from user
//    float speed;
//    std::cout << "Enter speed: ";
//    std::cin >> speed;

//    //Declare a list of waypoints to work with
//    WAYPOINT A, B, C, D, E, F, G, H, I, J, K;
//    WAYPOINT waypoint_array[] = {A, B, C, D, E, F, G, H, I, J, K};


//    //Receive waypoint data from a file
//    std::ofstream outFile;
//    outFile.open("/home/joestory/Downloads/MAVSDK/example/test_mission/datafile.txt");
//    if (!outFile) {
//        std::cout << "Unable to open file datafile.txt when writing" << std::endl;
//        exit(1);   // call system to stop
//    }
//    outFile << "num_waypoints = " << num_waypoints << ";\n";
//    for (int i = 0; i < num_waypoints; i++)
//    {
//        outFile << "Waypoint name, latitude, longitude, altitude;\n";
//    }
//    outFile.close();

//    std::ifstream inFile;
//    std::string line;
//    inFile.open("/home/joestory/Downloads/MAVSDK/example/test_mission/datafile.txt");
//    if (!inFile) {
//        std::cout << "Unable to open file datafile.txt when reading" << std::endl;
//        exit(1);   // call system to stop
//    }

//    while (getline(inFile, line))
//    {
//        std::cout << line << std::endl;
//    }

//    inFile.close();

//    while ( getline (inFile,line) )
//    {
//        std::cout << line << '\n';
//    }




    //Receive waypoint data from the user
//    for (int i = 0; i<num_waypoints; i++)
//    {
//        double lat_temp, lon_temp, alt_temp;
//        std::cout << "Enter latitude: ";
//        std::cin >> lat_temp;
//        std::cout << "Enter longitude: ";
//        std::cin >> lon_temp;
//        std::cout << "Enter altitude: ";
//        std::cin >> alt_temp;

//        waypoint_array[i].name = "ABCDEFGHIJK";
//        waypoint_array[i].lat = lat_temp;
//        waypoint_array[i].lon = lon_temp;
//        waypoint_array[i].altitude = alt_temp;
//        waypoint_array[i].speed = speed;
//        waypoint_array[i].fly_through = true;
//        waypoint_array[i].gimbal_pitch_deg = 0;
//        waypoint_array[i].gimbal_yaw_deg = 0;
//        waypoint_array[i].camera_action = MissionItem::CameraAction::NONE;

//    }

    sleep_for(seconds(1));

    //Calculating mission plan
    std::vector<std::shared_ptr<MissionItem>> mission_items;

    std::cout << "Calculating best flight path:" << std::endl;
    std::cout << "" << std::endl;

//    //Takeoff Location
//    lat_takeoff = telemetry->position().latitude_deg;
//    //lat_takeoff_rad = ((lat_takeoff / 180) * pi);
//    lon_takeoff = telemetry->position().longitude_deg;
//    //lon_takeoff_rad = ((lon_takeoff / 180) * pi);
//    altitude_absolute_takeoff = telemetry->position().absolute_altitude_m;

//    std::cout << "Takeoff position is: " << lat_takeoff << ", " << lon_takeoff << ", " << altitude_absolute_takeoff << std::endl;

    /*(double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    MissionItem::CameraAction camera_action)*/


    //Calculate takeoff position
    lat_takeoff = telemetry->position().latitude_deg;
    lon_takeoff = telemetry->position().longitude_deg;
    altitude_absolute_takeoff = telemetry->position().absolute_altitude_m;

    float speed = 5.0f;

    //Takeoff Waypoint
    WAYPOINT TAKEOFF;
    TAKEOFF.identifier = 1;
    TAKEOFF.lat = lat_takeoff;
    TAKEOFF.lon = lon_takeoff;
    TAKEOFF.altitude = altitude_absolute_takeoff;

    //Mission Waypoint A
    WAYPOINT A;
    A.identifier = 2;
    A.lat = 47.3977759; //47.398241338125118
    A.lon = 8.5462028; //8.5455360114574432
    A.altitude = 10.0f;

    //Mission Waypoint B
    WAYPOINT B;
    B.identifier = 3;
    B.lat = 47.3980161; //47.398001890458097
    B.lon = 8.5453055; //8.5455576181411743
    B.altitude = 10.0f;

    //Mission Waypoint C
    WAYPOINT C;
    C.identifier = 4;
    C.lat = 47.3980003; //47.398058617228855
    C.lon = 8.5459517; //8.5454618036746979
    C.altitude = 10.0f;

    //Mission Waypoint D
    WAYPOINT D;
    D.identifier = 5;
    D.lat = 47.398008;
    D.lon = 8.545701;
    D.altitude = 10;



    //Declare a list of waypoints to work with
    WAYPOINT waypoint_array[num_waypoints+1] = {TAKEOFF, A, B, C, D};


    //Calculate the 2D distance array (spherical polar coordinates)
    for (int i = 0; i < num_waypoints+1; i++){
        for (int t = 0; t < num_waypoints+1; t++){
            cost_array[i][t] = distance(waypoint_array[i].altitude, waypoint_array[t].altitude, waypoint_array[i].lat, waypoint_array[t].lat, waypoint_array[i].lon, waypoint_array[t].lon);
        }
    }

    //route_array[0] = TAKEOFF;
    mincost(0, waypoint_array);

    //List the final waypoint as the start
    route_array[num_waypoints+1] = waypoint_array[0];




    std::cout << "\n\nMinimum cost is " << cost << std::endl;
    std::cout << "" << std::endl;

    //Print the 2D distance array
    std::cout << "2D Distance Array:" << std::endl;
    std::cout << "" << std::endl;
    for (int i = 0; i < num_waypoints+1; i++){
        std::cout << "[";
        for (int t = 0; t < num_waypoints+1; t++){
            std::cout << cost_array[i][t] << ",  ";
        }
        std::cout << "]" << std::endl;
    }

    std::cout << "" << std::endl;

    std::cout << "Route Array: " << std::endl;
    std::cout << "" << std::endl;
    for (int i=0; i<num_waypoints+1; i++){
        std::cout << (route_array[i].identifier) << "," << std::endl;
    }

//    double distance_a_b = distance(A.altitude, C.altitude, A.lat, C.lat, A.lon, C.lon);
//    double distance_a_c = distance(A.altitude, C.altitude, A.lat, C.lat, A.lon, C.lon);
//    double distance_b_c = distance(B.altitude, C.altitude, B.lat, C.lat, B.lon, C.lon);
//    double distance_takeoff_a = distance(altitude_absolute_takeoff, altitude_A, lat_takeoff, lat_A, lon_takeoff, lon_A);
//    double distance_takeoff_b = distance(altitude_absolute_takeoff, altitude_B, lat_takeoff, lat_B, lon_takeoff, lon_B);
//    double distance_takeoff_c = distance(altitude_absolute_takeoff, altitude_C, lat_takeoff, lat_C, lon_takeoff, lon_C);

//    std::cout << "Distance A to B is: " << distance_a_b << "m" << std::endl;
//    std::cout << "Distance A to C is: " << distance_a_c << "m" << std::endl;
//    std::cout << "Distance B to C is: " << distance_b_c << "m" << std::endl;
//    std::cout << "Distance Takeoff to A is: " << distance_takeoff_a << "m" << std::endl;
//    std::cout << "Distance Takeoff to B is: " << distance_takeoff_b << "m" << std::endl;
//    std::cout << "Distance Takeoff to C is: " << distance_takeoff_c << "m" << std::endl;


//    double route_abc = distance_takeoff_a + distance_a_b + distance_b_c + distance_takeoff_c;
//    double route_acb = distance_takeoff_a + distance_a_c + distance_b_c + distance_takeoff_b;
//    double route_bac = distance_takeoff_b + distance_a_b + distance_a_c + distance_takeoff_c;
//    double route_bca = distance_takeoff_b + distance_b_c + distance_a_c + distance_takeoff_a;
//    double route_cab = distance_takeoff_c + distance_a_c + distance_a_b + distance_takeoff_b;
//    double route_cba = distance_takeoff_c + distance_b_c + distance_a_b + distance_takeoff_a;


//    std::cout << "" << std::endl;
//    std::cout << "Route distance calculations: " << std::endl;
//    std::cout << "" << std::endl;

//    std::cout << "Route ABC is: " << route_abc << "m" << std::endl;
//    std::cout << "Route ACB is: " << route_acb << "m" << std::endl;
//    std::cout << "Route BAC is: " << route_bac << "m" << std::endl;
//    std::cout << "Route BCA is: " << route_bca << "m" << std::endl; //This is a repeat
//    std::cout << "Route CAB is: " << route_cab << "m" << std::endl; //This is a repeat
//    std::cout << "Route CBA is: " << route_cba << "m" << std::endl; //This is a repeat
//    //const int SIZE = 6;

//    std::cout << "" << std::endl;

//    std::cout << "Sorted List:" << std::endl;
//    std::cout << "" << std::endl;

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


//    std::cout << "" << std::endl;
//    std::cout << "The best route is: " << std::endl;
//    std::cout << array[0].identifier << std::endl;
//    std::cout << "" << std::endl;

//    double route_array [] = {route_abc, route_acb, route_bac, route_bca, route_cab, route_cba};
//    std::sort(route_array, route_array + SIZE);

//    for(int i=0; i < SIZE; i++)
//    {
//        std::cout << route_array[i] << std::endl;
//    }

//    std::cout << route_array[0] << "m" << std::endl;


//    std::cout << "" << std::endl;
//    std::cout << "Route dictionary: " << std::endl;
//    std::cout << "" << std::endl;

//    std::map<std::string, double> route_dictionary;

//    route_dictionary["ABC"]=route_abc;
//    route_dictionary["ACB"]=route_acb;
//    route_dictionary["BAC"]=route_bac;
//    route_dictionary["BCA"]=route_bca;
//    route_dictionary["CAB"]=route_cab;
//    route_dictionary["CBA"]=route_cba;

    //std::cout << route_dictionary["ABC"] << "m" << std::endl;


    sleep_for(seconds(4));


    std::cout << "Creating and uploading mission" << std::endl;

    for (int x = 1; x<num_waypoints+1; x++){
        mission_items.push_back(make_mission_item(route_array[x].lat,
                                                  route_array[x].lon,
                                                  route_array[x].altitude,
                                                  speed,
                                                  false,
                                                  20.0f,
                                                  60.0f,
                                                  MissionItem::CameraAction::NONE));
    }





    {
        std::cout << "Uploading mission..." << std::endl;
        // We only have the upload_mission function asynchronous for now, so we wrap it using
        // std::future.
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        mission->upload_mission_async(mission_items,
                                      [prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::SUCCESS) {
            std::cout << "Mission upload failed (" << Mission::result_str(result) << "), exiting."
                      << std::endl;
            return 1;
        }
        std::cout << "Mission uploaded." << std::endl;
    }

    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();
    handle_action_err_exit(arm_result, "Arm failed: ");
    std::cout << "Armed." << std::endl;

    std::atomic<bool> want_to_pause{false};
    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission->subscribe_progress([&want_to_pause](int current, int total) {
        std::cout << "Mission status update: " << current << " / " << total << std::endl;

        if (current >= 2) {
            // We can only set a flag here. If we do more request inside the callback,
            // we risk blocking the system.
            want_to_pause = true;
        }
    });

    {
        std::cout << "Starting mission." << std::endl;
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        mission->start_mission_async([prom](Mission::Result result) {
            prom->set_value(result);
            std::cout << "Started mission." << std::endl;
        });

        const Mission::Result result = future_result.get();
        handle_mission_err_exit(result, "Mission start failed: ");
    }

//    while (!want_to_pause) {
//        sleep_for(seconds(1));
//    }

//    {
//        auto prom = std::make_shared<std::promise<Mission::Result>>();
//        auto future_result = prom->get_future();

//        std::cout << "Pausing mission..." << std::endl;
//        mission->pause_mission_async([prom](Mission::Result result) { prom->set_value(result); });

//        const Mission::Result result = future_result.get();
//        if (result != Mission::Result::SUCCESS) {
//            std::cout << "Failed to pause mission (" << Mission::result_str(result) << ")"
//                      << std::endl;
//        } else {
//            std::cout << "Mission paused." << std::endl;
//        }
//    }

    // Pause for 5 seconds.
    //sleep_for(seconds(5));

    // Then continue.
//    {
//        auto prom = std::make_shared<std::promise<Mission::Result>>();
//        auto future_result = prom->get_future();

//        std::cout << "Resuming mission..." << std::endl;
//        mission->start_mission_async([prom](Mission::Result result) { prom->set_value(result); });

//        const Mission::Result result = future_result.get();
//        if (result != Mission::Result::SUCCESS) {
//            std::cout << "Failed to resume mission (" << Mission::result_str(result) << ")"
//                      << std::endl;
//        } else {
//            std::cout << "Resumed mission." << std::endl;
//        }
//    }

    while (!mission->mission_finished()) {
        sleep_for(seconds(1));
    }

    {
        // We are done, and can do RTL to go home.
        std::cout << "Commanding RTL..." << std::endl;
        const Action::Result result = action->return_to_launch();
        if (result != Action::Result::SUCCESS) {
            std::cout << "Failed to command RTL (" << Action::result_str(result) << ")"
                      << std::endl;
        } else {
            std::cout << "Commanded RTL." << std::endl;
        }
    }

    // We need to wait a bit, otherwise the armed state might not be correct yet.
    sleep_for(seconds(2));

    while (telemetry->armed()) {
        // Wait until we're done.
        sleep_for(seconds(1));
    }
    std::cout << "Disarmed, exiting." << std::endl;
}

std::shared_ptr<MissionItem> make_mission_item(double latitude_deg,
                                               double longitude_deg,
                                               float relative_altitude_m,
                                               float speed_m_s,
                                               bool is_fly_through,
                                               float gimbal_pitch_deg,
                                               float gimbal_yaw_deg,
                                               MissionItem::CameraAction camera_action)
{
    std::shared_ptr<MissionItem> new_item(new MissionItem());
    new_item->set_position(latitude_deg, longitude_deg);
    new_item->set_relative_altitude(relative_altitude_m);
    new_item->set_speed(speed_m_s);
    new_item->set_fly_through(is_fly_through);
    new_item->set_gimbal_pitch_and_yaw(gimbal_pitch_deg, gimbal_yaw_deg);
    new_item->set_camera_action(camera_action);
    return new_item;
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

