#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

using namespace std;

class vehicle {

  /* 
  id: car's unique ID,
  x: car's x position in map coordinates,
  y: car's y position in map coordinates,
  v: car's speed,
  s: car's s position in frenet coordinates,
  d: car's d position in frenet coordinates. 
  */

  protected:

    int id;
    double x;
    double y;
    double v;
    double s;
    double d;
    double yaw;

    vector<double> previous_s;
    vector<double> previous_d;

  public:

    vehicle();
    vehicle(int id, double x, double y, double v, double s, double d);
    ~vehicle(){};

    int get_id();
    double get_x();
    double get_y();
    double get_v();
    double get_s();
    double get_d();
    double get_yaw();
    int get_lane();

    void update_vehicle_values(double x, double y, double v, double s, double d, double yaw);
    void set_previous_s(vector<double> previous_s);
    void set_previous_d(vector<double> previous_d);
    vector<double> prev_s();
    vector<double> prev_d();

};

#endif // VEHICLE_H