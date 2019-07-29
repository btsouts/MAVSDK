#ifndef PAYLOAD_MISSION_H
#define PAYLOAD_MISSION_H

#include <string>
#include <cmath>
#include <iostream>
#include <tuple>

const double earth_radius = 6371000; //metres;
const double pi = 3.1415926535897;
const float g = 9.81; //ms^-2
const float rho = 1.225; //Density of air in kgm^-3

class WAYPOINTS{
  public:
    int id;
    std::string user;
    float lat;
    float lon;
    float alt;
    float speed;
    int deadline;
    int numOfWaypoints;
    float payload;


    /*** Waypoint trajectory functions ***/

    //Calculate the distance between two waypoints in spherical polar coordinates (latitude, longitude, and altitude)
    float time(float alt1, float alt2, float lat1, float lat2, float lon1, float lon2, float speed)
    {
        float lat1_rad = (lat1/180)*pi;
        float lat2_rad = (lat2/180)*pi;
        float lon1_rad = (lon1/180)*pi;
        float lon2_rad = (lon2/180)*pi;

        float x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                        (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));

        float time = (x)/(speed);
        return time;
    }

};

class DRONE{
  public:
    //Physical drone properties
    float mass;
    int num_rotors;
    float rotor_radius;
    float max_velocity;
    float min_velocity;

    //Battery and electrical properties
    float bat_capacity;
    float bat_voltage;
    float efficiency;

    //Calculate an estimate of the maximum drone flight time in minutes
    float calc_max_flight_time(){
        float rotor_area = num_rotors * (pi * pow(rotor_radius, 2));
        float power = sqrt( (2*pow(mass,3)*pow(g,3)) / (rho*rotor_area) );
        float energy = bat_voltage * bat_capacity * pow(10,-3); //Battery energy in Wh
        float max_flight_time = (energy/power) * 60; //Max flight time in minutes
        return max_flight_time;
    }
};

class TRAJECTORY{
    public:
        float** calc_cost(int num_waypoints, WAYPOINTS array[]){
            WAYPOINTS cost_object;
            float ** cost2d;
            cost2d = new float * [num_waypoints+2];

            for (int i=0; i < num_waypoints+1; i++){
                cost2d[i] = new float [num_waypoints+2];
            }

            for (int i = 0; i < num_waypoints+1; i++){
                for (int t = 0; t < num_waypoints+1; t++){
                    cost2d[i][t] = cost_object.time(array[i].alt, array[t].alt, array[i].lat, array[t].lat, array[i].lon, array[t].lon, array[t].speed);
                }
            }

            //Print the 2D Cost array
            std::cout << "2D Cost Array:" << std::endl;
            std::cout << "" << std::endl;
            for (int i = 0; i < num_waypoints+1; i++){
                std::cout << "[";
                for (int t = 0; t < num_waypoints+1; t++){
                    std::cout << cost2d[i][t] << ",  ";
                }
                std::cout << "]" << std::endl;
            }
            std::cout << "" << std::endl;

            return cost2d;
        }

        //Finds the nearest neighbour that hasn't been visited
        std::tuple<int, double, bool> least(int p, int num_waypoints, int completed[], float ** cost_array, float cost){
            int i,np=0;
            int min=99999,kmin;
            bool is_final = true;


            for (i=0;i<num_waypoints+1;i++){
                if((cost_array[p][i]!=0)&&(completed[i]==0)){
                    if(cost_array[p][i]+cost_array[i][p] < min){
                        min = cost_array[i][0]+cost_array[p][i];
                        kmin=cost_array[p][i];
                        np=i;
                        is_final = false;
                    }
                }
            }
            if(is_final != true){
                cost+=kmin;
            }
            return {np, cost, is_final};
        }

        //Finds a close to optimal route using the 'Greedy' method
        float mincost(int position, WAYPOINTS array[], int num_waypoints, float cost, int completed[], WAYPOINTS route_array[], int n, float ** cost_array){
            int nposition;
            bool is_final = false;

            completed[position]=1;

            std::cout << position << "--->";

            route_array[n] = array[position];
            n++;

            //nposition = least(position, num_waypoints);
            std::tie(nposition, cost, is_final) = TRAJECTORY::least(position, num_waypoints, completed, cost_array, cost);

            if(is_final == true){
                nposition=0;
                std::cout << nposition;
                cost+=cost_array[position][nposition];
                return cost;
            }

            return mincost(nposition, array, num_waypoints, cost, completed, route_array, n, cost_array);


        }


    private:
};

#endif
