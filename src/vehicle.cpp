#include "vehicle.h"

using namespace std;

vehicle::vehicle(){
  this->id = -1;
}

vehicle::vehicle(int id, double x, double y, double v, double s, double d){
  this->id = id;
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
}

/* GETTERS*/
int vehicle::get_id(){
  return this->id;
}

double vehicle::get_x(){
  return this->x;
}

double vehicle::get_y(){
  return this->y;
}

double vehicle::get_v(){
  return this->v;
}

double vehicle::get_s(){
  return this->s;
}

double vehicle::get_d(){
  return this->d;
}

vector<double> vehicle::prev_s(){
  return this->previous_s;
}

vector<double> vehicle::prev_d(){
  return this->previous_d;
}

int vehicle::get_lane(){
  if (this->d < 4.0) {
    return 0;
  }
  else if ((this->d >= 4.0) && (this->d < 8.0)) {
    return 1;
  }
  else {
    return 2;
  }
}

void vehicle::update_vehicle_values(double x, double y, double v, double s, double d, double yaw){
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
}

void vehicle::set_previous_s(vector<double> previous_s){
  this->previous_s = previous_s;
}

void vehicle::set_previous_d(vector<double> previous_d){
  this->previous_d = previous_d;
}
