#ifndef INPUT_H
#define INPUT_H

typedef struct {
    int id;
    double lat;
    double lon;
    float alt;
    float speed;
}WAYPOINT;

double distance(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2);

#endif
