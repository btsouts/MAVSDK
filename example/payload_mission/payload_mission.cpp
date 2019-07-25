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
#include "payload_mission.h"

#include <functional>
#include <future>
#include <iostream>
#include <memory>

#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <new>

#include <stdio.h>
#include <stdlib.h>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

using namespace mavsdk;
using namespace std::placeholders; // for `_1`
using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

//Cost function variables
double cost = 0;
int n=0;
int numOfWaypoints;

//Declare dynamic arrays
WAYPOINTS * route_array;
WAYPOINTS * waypoint_array;
double ** cost_array;
int * completed;

//Declare drone variables
double drone_weight;
double drone_energy;
double drone_max_vel;
double drone_min_vel; //Is this really necessary???


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

//Finds the nearest neighbour that hasn't been visited
int least(int p, int num_waypoints);

//Finds the minimum cost route using the nearest neighbour algorithm
void mincost(int position, WAYPOINTS array[], int num_waypoints);

void calc_cost(const int num_waypoints, WAYPOINTS array[]);

int main(int argc, char **argv)
{
    Mavsdk dc;

    std::cout.precision(8);
//    std::cout << "Enter the number of waypoints: ";
//    std::cin >> numOfWaypoints;

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

    sleep_for(seconds(1));

    //Open the input.txt file that contains the waypoints
    std::ifstream infile;
    infile.open ("/home/joestory/Downloads/MAVSDK/example/payload_mission/input.txt");
    if (infile.is_open())
        std::cout << "Opened input.txt" << std::endl;
    std::cout << "" << std::endl;

    //Get the number of waypoints from the first line of the text file
    std::string line;
    std::getline(infile, line);
    std::istringstream iss(line);
    if (!(iss >> numOfWaypoints >> drone_weight >> drone_energy >> drone_max_vel >> drone_min_vel)) { std::cerr << "READ ERROR 1" << std::endl; return 1; }
    std::cout << "Number of waypoints found: " << numOfWaypoints << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Drone weight: " << drone_weight << std::endl;
    std::cout << "Drone energy: " << drone_energy << std::endl;
    std::cout << "Drone max vel: " << drone_max_vel << std::endl;
    std::cout << "Drone min vel: " << drone_min_vel << std::endl;
    std::cout << "" << std::endl;

    //Declare all dynamic arrays
    //waypoint_array = new WAYPOINT [numOfWaypoints+2];
    waypoint_array = new WAYPOINTS [numOfWaypoints+2];
    route_array = new WAYPOINTS [numOfWaypoints+2];

    //Create the 2D dynamic cost array
    cost_array = new double * [numOfWaypoints+2];
    for (int i=0; i < numOfWaypoints+1; i++){
        cost_array[i] = new double [numOfWaypoints+2];
    }

    //Create the dynamic "Completed" array and set all values to zero (No points visited)
    completed = new int [numOfWaypoints+1];
    for (int i=0; i<numOfWaypoints+1; i++){
        completed[i]=0;
    }

//    target_array[0].id = 1;
//    target_array[0].lat = 47.460154;
//    target_array[1].id = 2;
//    target_array[1].lat = 47.359876;

//    std::cout << target_array[0].id << ", " << target_array[0].lat << ", " << target_array[1].id << ", " << target_array[1].lat << std::endl;

    //Update takeoff position
    //waypoint_array[0] = TAKEOFF;
    waypoint_array[0].id = 0;
    waypoint_array[0].user = "Drone";
    waypoint_array[0].lat = telemetry->position().latitude_deg;
    waypoint_array[0].lon = telemetry->position().longitude_deg;
    waypoint_array[0].alt = telemetry->position().absolute_altitude_m;
    waypoint_array[0].speed = 5.0f;

    //Get the information from the input.txt file and append it to the waypoint_array
    int i = 1;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int id;
        std::string user;
        double lat, lon;
        float alt;
        float speed;
        float deadline;
        double payload;

        //Check that the file has been read properly, if not then terminate the program
        if (!(iss >> id >> user >> lat >> lon >> alt >> speed >> deadline >> payload)) {
            std::cerr << "READ ERROR 2" << std::endl;
            std::cerr << id << ", " << user << ", " << lat << ", " << lon << ", " << alt << ", " << speed << ", " << deadline << ", " << payload << std::endl;
            return 1;
        }

        waypoint_array[i].id = id;
        waypoint_array[i].user = user;
        waypoint_array[i].lat = lat;
        waypoint_array[i].lon = lon;
        waypoint_array[i].alt = alt;
        waypoint_array[i].speed = speed;
        waypoint_array[i].deadline = deadline;
        waypoint_array[i].payload = payload;
        i++;

//        if (!(iss >> username)) { std::cerr << "READ ERROR 2" << std::endl; return 1; }
//        waypoint_array[i].id = id;
//        waypoint_array[i].user = username;
//        waypoint_array[i].lat = lat;
//        waypoint_array[i].lon = lon;
//        waypoint_array[i].alt = alt;
//        waypoint_array[i].speed = speed;
//        i++;
    }

    //Set the last point as the home position
    waypoint_array[numOfWaypoints+1]=waypoint_array[0];

    //Set the starting payload
    for (int i=1; i<numOfWaypoints+1; i++){
        waypoint_array[0].payload += waypoint_array[i].payload;
    }


    //Print
    for (int i=0; i<numOfWaypoints+1;i++) {
        std::cout << "Waypoint " << waypoint_array[i].id << ": "
                  << waypoint_array[i].user << ", "
                  << waypoint_array[i].lat << ", "
                  << waypoint_array[i].lon << ", "
                  << waypoint_array[i].alt << ", "
                  << waypoint_array[i].speed << ", "
                  << waypoint_array[i].deadline << ", "
                  << waypoint_array[i].payload << std::endl;
    }
    std::cout << "" << std::endl;

    //Calculate the 2D time array (spherical polar coordinates)
    std::cout << "This program minimises the number of missed deadlines" << std::endl;
    std::cout << "" << std::endl;
    calc_cost(numOfWaypoints, waypoint_array);
    std::cout << "" << std::endl;

    //Calculating mission plan
    std::vector<std::shared_ptr<MissionItem>> mission_items;
    std::cout << "Calculating best flight path:" << std::endl;
    std::cout << "" << std::endl;
    mincost(0, waypoint_array, numOfWaypoints);
    cost += 3*numOfWaypoints;
    std::cout << "\n\nMinimum cost is " << cost << "s" << std::endl;
    std::cout << "" << std::endl;

    sleep_for(seconds(3));

    //Upload the mission plan using the route_array that was created by the mincost function
    std::cout << "Creating and uploading mission" << std::endl;

    /* Mission item structure:

    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    MissionItem::CameraAction camera_action)*/

    for (int x = 1; x<numOfWaypoints+1; x++){
        mission_items.push_back(make_mission_item(route_array[x].lat,
                                                  route_array[x].lon,
                                                  route_array[x].alt,
                                                  route_array[x].speed,
                                                  false,
                                                  20.0f,
                                                  60.0f,
                                                  MissionItem::CameraAction::NONE));
    }

    //Return to home
    mission_items.push_back(make_mission_item(waypoint_array[0].lat,
                                              waypoint_array[0].lon,
                                              waypoint_array[0].alt,
                                              waypoint_array[0].speed,
                                              false,
                                              20.0f,
                                              60.0f,
                                              MissionItem::CameraAction::NONE));

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

    for (int i=0; i<20; i++){
        double north, east, down;
        north = telemetry->ground_speed_ned().velocity_north_m_s;
        east = telemetry->ground_speed_ned().velocity_east_m_s;
        down = telemetry->ground_speed_ned().velocity_down_m_s;
        std::cout << "North Velocity: " << north << ", East Velocity: " << east << ", Downwards Velocity: " << down << std::endl;
        sleep_for(seconds(2));
    }

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

//Finds the nearest neighbour that hasn't been visited
int least(int p, int num_waypoints){
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

//Finds a close to optimal route using the 'Greedy' method
void mincost(int position, WAYPOINTS array[], int num_waypoints){
    int nposition;

    completed[position]=1;

    std::cout << position << "--->";

    route_array[n] = array[position];
    n++;

    nposition = least(position, num_waypoints);

    if(nposition==999){
        nposition=0;
        std::cout << nposition;
        cost+=cost_array[position][nposition];
        return;
    }
    mincost(nposition, array, num_waypoints);
}

//Calculates the 2D cost function array (distance, time, power, etc.)
void calc_cost(int num_waypoints, WAYPOINTS array[]){
    WAYPOINTS cost_object;
    for (int i = 0; i < num_waypoints+1; i++){
        for (int t = 0; t < num_waypoints+1; t++){
            cost_array[i][t] = cost_object.time (array[i].alt, array[t].alt, array[i].lat, array[t].lat, array[i].lon, array[t].lon, array[t].speed);
        }
    }
    //Print the 2D Cost array
    std::cout << "2D Cost Array:" << std::endl;
    std::cout << "" << std::endl;
    for (int i = 0; i < num_waypoints+1; i++){
        std::cout << "[";
        for (int t = 0; t < num_waypoints+1; t++){
            std::cout << cost_array[i][t] << ",  ";
        }
        std::cout << "]" << std::endl;
    }
}
