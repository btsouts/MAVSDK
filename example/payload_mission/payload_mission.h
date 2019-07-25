#ifndef PAYLOAD_MISSION_H
#define PAYLOAD_MISSION_H

#include <string>
#include <cmath>

const double earth_radius = 6371000; //metres;
const double pi = 3.1415926535897;

class WAYPOINTS{
  public:
    int id;
    std::string user;
    double lat;
    double lon;
    float alt;
    float speed;
    float deadline;
    double payload;

    //Calculate the distance between two waypoints in spherical polar coordinates (latitude, longitude, and altitude)
    double time(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2, float speed)
    {
        double lat1_rad = (lat1/180)*pi;
        double lat2_rad = (lat2/180)*pi;
        double lon1_rad = (lon1/180)*pi;
        double lon2_rad = (lon2/180)*pi;

        double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                        (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));

        double time = (x)/(speed);
        return time;
    }
};

double time(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2, float speed);

#endif
