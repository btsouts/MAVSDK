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
    std::string name; //Char?
    double lat;
    double lon;
    float altitude;
    float speed;
    bool fly_through;
    float gimbal_pitch_deg;
    float gimbal_yaw_deg;
    mavsdk::MissionItem::CameraAction camera_action;
}WAYPOINT;

typedef struct {
    std::string name; //Char?
    double distance;
    int num_waypoints;
    WAYPOINT traj[3];
}ROUTE;

//Function Prototype
void sortArray(ROUTE array[], int size);

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



    //Calculating mission plan
    std::vector<std::shared_ptr<MissionItem>> mission_items;

    std::cout << "Calculating best flight path:" << std::endl;
    std::cout << "" << std::endl;

    //Takeoff Location
    lat_takeoff = telemetry->position().latitude_deg;
    //lat_takeoff_rad = ((lat_takeoff / 180) * pi);
    lon_takeoff = telemetry->position().longitude_deg;
    //lon_takeoff_rad = ((lon_takeoff / 180) * pi);
    altitude_absolute_takeoff = telemetry->position().absolute_altitude_m;

    std::cout << "Takeoff position is: " << lat_takeoff << ", " << lon_takeoff << ", " << altitude_absolute_takeoff << std::endl;

    /*(double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    MissionItem::CameraAction camera_action)*/

    float speed = 5;


    //Mission Waypoint A
    lat_A = 47.398241338125118; //47.398241338125118
    //lat_A_rad = ((lat_A / 180) * pi);
    lon_A = 8.5455360114574432; //8.5455360114574432
    //lon_A_rad = ((lon_A / 180) * pi);
    altitude_A = 10;


    WAYPOINT A;
    A.name = "A";
    A.lat = lat_A;
    A.lon = lon_A;
    A.altitude = altitude_A;
    A.speed = speed;
    A.fly_through = true;
    A.gimbal_pitch_deg = 0;
    A.gimbal_yaw_deg = 0;
    A.camera_action = MissionItem::CameraAction::NONE;


    //Mission Waypoint B
    lat_B = 47.398001890458097; //47.398001890458097
    //lat_B_rad = ((lat_B / 180) * pi);
    lon_B = 8.5455576181411743; //8.5455576181411743
    //lon_B_rad = ((lon_B / 180) * pi);
    altitude_B = 10;

    WAYPOINT B;
    B.name = "B";
    B.lat = lat_B;
    B.lon = lon_B;
    B.altitude = altitude_B;
    B.speed = speed;
    B.fly_through = true;
    B.gimbal_pitch_deg = 0;
    B.gimbal_yaw_deg = 0;
    B.camera_action = MissionItem::CameraAction::NONE;

    //Mission Waypoint C
    lat_C = 47.398058617228855; //47.398058617228855
    //lat_C_rad = ((lat_C / 180) * pi);
    lon_C = 8.5454618036746979; //8.5454618036746979
    //lon_C_rad = ((lon_C / 180) * pi);;
    altitude_C = 10;

    WAYPOINT C;
    C.name = "C";
    C.lat = lat_C;
    C.lon = lon_C;
    C.altitude = altitude_C;
    C.speed = speed;
    C.fly_through = true;
    C.gimbal_pitch_deg = 0;
    C.gimbal_yaw_deg = 0;
    C.camera_action = MissionItem::CameraAction::NONE;

    sleep_for(seconds(1));

    //Haversine Formula
    //distance_a_b = (2*earth_radius)*( asin ( sqrt(   pow((sin((lat_B_rad-lat_A_rad)/2)), 2) + cos(lat_A_rad)*cos(lat_B_rad)*pow(sin((lon_B_rad-lon_A_rad)/2), 2)    )));

    //Spherical Polar Coordinates
    double distance_a_b = distance(A.altitude, B.altitude, A.lat, B.lat, A.lon, B.lon);
    double distance_a_c = distance(A.altitude, C.altitude, A.lat, C.lat, A.lon, C.lon);
    double distance_b_c = distance(B.altitude, C.altitude, B.lat, C.lat, B.lon, C.lon);
    double distance_takeoff_a = distance(altitude_absolute_takeoff, altitude_A, lat_takeoff, lat_A, lon_takeoff, lon_A);
    double distance_takeoff_b = distance(altitude_absolute_takeoff, altitude_B, lat_takeoff, lat_B, lon_takeoff, lon_B);
    double distance_takeoff_c = distance(altitude_absolute_takeoff, altitude_C, lat_takeoff, lat_C, lon_takeoff, lon_C);

    std::cout << "Distance A to B is: " << distance_a_b << "m" << std::endl;
    std::cout << "Distance A to C is: " << distance_a_c << "m" << std::endl;
    std::cout << "Distance B to C is: " << distance_b_c << "m" << std::endl;
    std::cout << "Distance Takeoff to A is: " << distance_takeoff_a << "m" << std::endl;
    std::cout << "Distance Takeoff to B is: " << distance_takeoff_b << "m" << std::endl;
    std::cout << "Distance Takeoff to C is: " << distance_takeoff_c << "m" << std::endl;


    double route_abc = distance_takeoff_a + distance_a_b + distance_b_c + distance_takeoff_c;
    double route_acb = distance_takeoff_a + distance_a_c + distance_b_c + distance_takeoff_b;
    double route_bac = distance_takeoff_b + distance_a_b + distance_a_c + distance_takeoff_c;
    double route_bca = distance_takeoff_b + distance_b_c + distance_a_c + distance_takeoff_a;
    double route_cab = distance_takeoff_c + distance_a_c + distance_a_b + distance_takeoff_b;
    double route_cba = distance_takeoff_c + distance_b_c + distance_a_b + distance_takeoff_a;


    std::cout << "" << std::endl;
    std::cout << "Route distance calculations: " << std::endl;
    std::cout << "" << std::endl;

    std::cout << "Route ABC is: " << route_abc << "m" << std::endl;
    std::cout << "Route ACB is: " << route_acb << "m" << std::endl;
    std::cout << "Route BAC is: " << route_bac << "m" << std::endl;
    std::cout << "Route BCA is: " << route_bca << "m" << std::endl; //This is a repeat
    std::cout << "Route CAB is: " << route_cab << "m" << std::endl; //This is a repeat
    std::cout << "Route CBA is: " << route_cba << "m" << std::endl; //This is a repeat
    //const int SIZE = 6;

    std::cout << "" << std::endl;

    std::cout << "Sorted List:" << std::endl;
    std::cout << "" << std::endl;

    ROUTE ABC;
    ABC.name = "ABC";
    ABC.distance = route_abc;
    ABC.num_waypoints = 3;
    ABC.traj[0] = A;
    ABC.traj[1] = B;
    ABC.traj[2] = C;

    ROUTE ACB;
    ACB.name = "ACB";
    ACB.distance = route_acb;
    ACB.num_waypoints = 3;
    ACB.traj[0] = A;
    ACB.traj[1] = C;
    ACB.traj[2] = B;

    ROUTE BAC;
    BAC.name = "BAC";
    BAC.distance = route_bac;
    BAC.num_waypoints = 3;
    BAC.traj[0] = B;
    BAC.traj[1] = A;
    BAC.traj[2] = C;


    const int SIZE = 3;
    ROUTE array[SIZE];
    array[0].name = ABC.name;
    array[0].distance = ABC.distance;
    array[0].num_waypoints = ABC.num_waypoints;
    array[0].traj[0] = ABC.traj[0];
    array[0].traj[1] = ABC.traj[1];
    array[0].traj[2] = ABC.traj[2];
    array[1].name = ACB.name;
    array[1].distance = ACB.distance;
    array[1].num_waypoints = ACB.num_waypoints;
    array[1].traj[0] = ACB.traj[0];
    array[1].traj[1] = ACB.traj[1];
    array[1].traj[2] = ACB.traj[2];
    array[2].name = BAC.name;
    array[2].distance = route_bac;
    array[2].num_waypoints = BAC.num_waypoints;
    array[2].traj[0] = BAC.traj[0];
    array[2].traj[1] = BAC.traj[1];
    array[2].traj[2] = BAC.traj[2];



    sortArray(array, SIZE);


    for (int i = 0; i < SIZE; i++)
    {
        std::cout << array[i].name << std::endl;
    }




//    std::cout << "The best route is: " << std::endl;

//    double route_array [] = {route_abc, route_acb, route_bac, route_bca, route_cab, route_cba};
//    std::sort(route_array, route_array + SIZE);

//    for(int i=0; i < SIZE; i++)
//    {
//        std::cout << route_array[i] << std::endl;
//    }

//    std::cout << route_array[0] << "m" << std::endl;




    std::cout << "" << std::endl;
    std::cout << "Route dictionary: " << std::endl;
    std::cout << "" << std::endl;

    std::map<std::string, double> route_dictionary;

    route_dictionary["ABC"]=route_abc;
    route_dictionary["ACB"]=route_acb;
    route_dictionary["BAC"]=route_bac;
    route_dictionary["BCA"]=route_bca;
    route_dictionary["CAB"]=route_cab;
    route_dictionary["CBA"]=route_cba;

    //std::cout << route_dictionary["ABC"] << "m" << std::endl;

    std::cout << "" << std::endl;


//    //Sort
//    typedef std::function<bool(std::pair<std::string, double>, std::pair<std::string, double>)> Comparator;

//    Comparator compFunctor =
//                    [](std::pair<std::string, double> elem1 ,std::pair<std::string, double> elem2)
//                    {
//                            return elem1.second < elem2.second;
//                    };


//    // Declaring a set that will store the pairs using above comparision logic
//    std::set<std::pair<std::string, double>, Comparator> setOfRoutes(
//                    route_dictionary.begin(), route_dictionary.end(), compFunctor);

//    // Iterate over a set using range base for loop
//    // It will display the items in sorted order of values
//    for (std::pair<std::string, double> element : setOfRoutes)
//        std::cout << element.first << " : " << element.second << std::endl;






    sleep_for(seconds(6000));


    std::cout << "Creating and uploading mission" << std::endl;


    mission_items.push_back(make_mission_item(47.398170327054473,
                                              8.5456490218639658,
                                              10.0f,
                                              5.0f,
                                              false,
                                              20.0f,
                                              60.0f,
                                              MissionItem::CameraAction::NONE));

    mission_items.push_back(make_mission_item(47.398241338125118,
                                              8.5455360114574432,
                                              10.0f,
                                              2.0f,
                                              true,
                                              0.0f,
                                              -60.0f,
                                              MissionItem::CameraAction::TAKE_PHOTO));

//    mission_items.push_back(make_mission_item(47.398139363821485,
//                                              8.5453846156597137,
//                                              10.0f,
//                                              5.0f,
//                                              true,
//                                              -45.0f,
//                                              0.0f,
//                                              MissionItem::CameraAction::START_VIDEO));

//    mission_items.push_back(make_mission_item(47.398058617228855,
//                                              8.5454618036746979,
//                                              10.0f,
//                                              2.0f,
//                                              false,
//                                              -90.0f,
//                                              30.0f,
//                                              MissionItem::CameraAction::STOP_VIDEO));

//    mission_items.push_back(make_mission_item(47.398100366082858,
//                                              8.5456969141960144,
//                                              10.0f,
//                                              5.0f,
//                                              false,
//                                              -45.0f,
//                                              -30.0f,
//                                              MissionItem::CameraAction::START_PHOTO_INTERVAL));

    mission_items.push_back(make_mission_item(47.398001890458097,
                                              8.5455576181411743,
                                              10.0f,
                                              5.0f,
                                              false,
                                              0.0f,
                                              0.0f,
                                              MissionItem::CameraAction::STOP_PHOTO_INTERVAL));

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

    while (!want_to_pause) {
        sleep_for(seconds(1));
    }

    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();

//        std::cout << "Pausing mission..." << std::endl;
//        mission->pause_mission_async([prom](Mission::Result result) { prom->set_value(result); });

//        const Mission::Result result = future_result.get();
//        if (result != Mission::Result::SUCCESS) {
//            std::cout << "Failed to pause mission (" << Mission::result_str(result) << ")"
//                      << std::endl;
//        } else {
//            std::cout << "Mission paused." << std::endl;
//        }
    }

    // Pause for 5 seconds.
    //sleep_for(seconds(5));

    // Then continue.
    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();

        std::cout << "Resuming mission..." << std::endl;
        mission->start_mission_async([prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();
        if (result != Mission::Result::SUCCESS) {
            std::cout << "Failed to resume mission (" << Mission::result_str(result) << ")"
                      << std::endl;
        } else {
            std::cout << "Resumed mission." << std::endl;
        }
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
