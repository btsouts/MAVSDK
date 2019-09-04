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
#include <iomanip>
#include <tuple>

#include <stdio.h>
#include <stdlib.h>

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

using namespace mavsdk;
using namespace std;
using namespace std::placeholders; // for `_1`
using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

//Cost function variables
float cost = 0;
int n=0;
int numOfWaypoints;
WAYPOINTS waypoints;
DRONE drone;
TRAJECTORY trajectory;

//Declare dynamic arrays
WAYPOINTS * route_array;
WAYPOINTS * waypoint_array;
float ** cost_array;

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
                                                      float hold_s,
                                                      bool is_fly_through,
                                                      float deadline,
                                                      float payloadWeight);

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int main(int argc, char **argv)
{
    Mavsdk dc;	
	string inFileName;
    float flightSpeed=3.0;

    {
        /*** Attempt to connect to the drone ***/

        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();
		

        std::cout << "Waiting to discover system..." << std::endl;
        dc.register_on_discover([prom](uint64_t uuid) {
            std::cout << "Discovered system with UUID: " << uuid << std::endl;
            prom->set_value();
        });

        std::string connection_url;
        ConnectionResult connection_result;

        if (argc == 4) {
            flightSpeed = atof(argv[3]);
            inFileName = argv[2];

			connection_url = argv[1];
            connection_result = dc.add_any_connection(connection_url);
        } else if (argc == 3) {
            printf("flightSpeed (arg 3) not set. Defaulting to 3.0\n");

            inFileName = argv[2];

			connection_url = argv[1];
            connection_result = dc.add_any_connection(connection_url);
		} else if (argc == 2) {
            printf("flightSpeed (arg 3) not set. Defaulting to 3.0\n");

            inFileName = "../input.txt";
            printf("inFileName (arg 2) not set. Defaulting to ../input.txt\n");

			connection_url = argv[1];
            connection_result = dc.add_any_connection(connection_url);
        } else {
            usage(argv[0]);
            return -1;
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

    /*** Gather the information needed to create a trajectory ***/

    //Open the input.txt file that contains the waypoints
    std::ifstream infile;
    infile.open (inFileName);
	
	if (infile.is_open())
        std::cout << "Opened " << inFileName << std::endl;
    std::cout << "" << std::endl;
	
    //Get the number of waypoints and drone information from the first line of the text file
    std::string line;
    std::getline(infile, line);
    std::istringstream iss(line);
    if (!(iss >> numOfWaypoints >> drone.mass >> drone.num_rotors >> drone.rotor_radius >> drone.bat_capacity >> drone.bat_voltage >> drone.max_velocity >> drone.min_velocity >> drone.efficiency)) { std::cerr << "READ ERROR 1" << std::endl; return 1; }
    std::cout << "Number of waypoints found: " << numOfWaypoints << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Drone Mass: " << drone.mass << "kg" << std::endl;
    std::cout << "Drone Number of Rotors: " << drone.num_rotors << std::endl;
    std::cout << "Drone Rotor Radius: " << drone.rotor_radius << "m" << std::endl;
    std::cout << "Drone Max Velocity: " << drone.max_velocity << "m/s" << std::endl;
    std::cout << "Drone Min Velocity: " << drone.min_velocity << "m/s" << std::endl;
    std::cout << "Drone Battery Capacity: " << drone.bat_capacity << "mAh" << std::endl;
    std::cout << "Drone Battery Voltage: " << drone.bat_voltage << "V" << std::endl;
    std::cout << "Drone Efficiency: " << drone.efficiency << "%" << std::endl;
    std::cout << "" << std::endl;

    float max_flight_time = drone.calc_max_flight_time();
    std::cout << std::setprecision(3) << "Maximum flight time is " << max_flight_time << " minutes" << std::endl;;
    std::cout << "" << std::endl;

    //Declare all dynamic arrays
    //waypoint_array = new WAYPOINT [numOfWaypoints+2];
    waypoint_array = new WAYPOINTS [numOfWaypoints+2];
    route_array = new WAYPOINTS [numOfWaypoints+2];

    //Create the 2D dynamic cost array
    cost_array = new float * [numOfWaypoints+2];
    for (int i=0; i < numOfWaypoints+1; i++){
        cost_array[i] = new float [numOfWaypoints+2];
    }

    /*Update takeoff position -- with current position -- still set to zurich */
    /* Takeoff coordinates are lat: 47.397751 lon: 8.545607 alt: -0.012000 */
    waypoint_array[0].id = 0;
    waypoint_array[0].user = "Takeoff";
    waypoint_array[0].lat = telemetry->position().latitude_deg;
    waypoint_array[0].lon = telemetry->position().longitude_deg;
    waypoint_array[0].alt = telemetry->position().absolute_altitude_m;
    waypoint_array[0].speed = flightSpeed;
    
    //printf("Takeoff coordinates are lat: %f lon: %f alt: %f\n",waypoint_array[0].lat,waypoint_array[0].lon,waypoint_array[0].alt);
    
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
        float payload;

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
        waypoint_array[i].speed = flightSpeed;
        waypoint_array[i].deadline = deadline;
        waypoint_array[i].payload = payload;
        i++;
    }

    //Set the last point as the home position
    waypoint_array[numOfWaypoints+1]=waypoint_array[0];

    //Print
    for (int i=0; i<=numOfWaypoints+1;i++) {
        std::cout << "Waypoint " << waypoint_array[i].id << ": "
                  << waypoint_array[i].user << ", "
                  << std::setprecision(8) << waypoint_array[i].lat << ", "
                  << std::setprecision(8) << waypoint_array[i].lon << ", "
                  << std::setprecision(2) << waypoint_array[i].alt << ", "
                  << waypoint_array[i].speed << ", "
                  << waypoint_array[i].deadline << ", "
                  << waypoint_array[i].payload << std::endl;
    }
    std::cout << "" << std::endl;

    std::cout.precision(5);

    //Calculating mission plan
    std::vector<std::shared_ptr<MissionItem>> mission_items;
    
    /*** Upload the mission plan using the route_array that was created by the mincost function ***/

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

    //*** ADD IN A LOITER TIME: MissionItem::set_loiter_time(3) ***//

    //Send home information first
    mission_items.push_back(make_mission_item(waypoint_array[0].lat,
                                              waypoint_array[0].lon,
                                              20.0f,
                                              waypoint_array[0].speed,
                                              0.0f,
                                              false,
                                              waypoint_array[0].deadline,
                                              waypoint_array[0].payload));

    //Upload stock waypoints
    for (int x = 1; x<=numOfWaypoints+1; x++){
        mission_items.push_back(make_mission_item(waypoint_array[x].lat,
                                                  waypoint_array[x].lon,
                                                  waypoint_array[x].alt,
                                                  waypoint_array[x].speed,
                                                  0.0f,
                                                  false,
                                                  waypoint_array[x].deadline,
                                                  waypoint_array[x].payload));
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

    sleep_for(seconds(3));

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

    std::cout << "Starting mission." << std::endl;
    auto prom = std::make_shared<std::promise<Mission::Result>>();
    auto future_result = prom->get_future();
    mission->start_mission_async([prom](Mission::Result result) {
        prom->set_value(result);
        std::cout << "Started mission." << std::endl;
    });

    const Mission::Result result = future_result.get();
    handle_mission_err_exit(result, "Mission start failed: ");


    //Wait until the drone is armed
    while (!telemetry->armed()){
        sleep_for(seconds(3));
    }
    // Wait until we're done.
    while (telemetry->armed()) {
        printf("North: %f, East: %f, down: %f\n",
            telemetry->ground_speed_ned().velocity_north_m_s, telemetry->ground_speed_ned().velocity_east_m_s, telemetry->ground_speed_ned().velocity_down_m_s);
        sleep_for(seconds(2));

        if (mission->mission_finished() == true) {
            printf("Mission finished detected\n");

            // Now just land here.
            /*
            std::cout << "Landing..." << std::endl;
            const Action::Result land_result = action->land();
            if (land_result != Action::Result::SUCCESS) {
                std::cout << ERROR_CONSOLE_TEXT << "Land failed: " << Action::result_str(land_result)
                        << NORMAL_CONSOLE_TEXT << std::endl;
                return 1;
            }
            */
            std::cout << "DisArming..." << std::endl;
            const Action::Result arm_result = action->disarm();
            handle_action_err_exit(arm_result, "DisArm failed: ");
            std::cout << "DisArmed." << std::endl;
        }
    }
}



std::shared_ptr<MissionItem> make_mission_item(double latitude_deg,
                                               double longitude_deg,
                                               float relative_altitude_m,
                                               float speed_m_s,
                                               float hold_s,
                                               bool is_fly_through,
                                               float deadline,
                                               float payloadWeight)
{
    std::shared_ptr<MissionItem> new_item(new MissionItem());
    new_item->set_position(latitude_deg, longitude_deg);
    new_item->set_relative_altitude(relative_altitude_m);
    new_item->set_speed(speed_m_s);
    new_item->set_loiter_time(hold_s);
    new_item->set_fly_through(is_fly_through);

    /* Additions for trajectory calculation */
    printf("Adding payloadWeight %f, deadline %f\n",(double) payloadWeight,(double) deadline);
    new_item->set_waypoint_deadline(deadline);
    new_item->set_payload_weight(payloadWeight);
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
