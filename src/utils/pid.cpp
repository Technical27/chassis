#include "chassis/utils/pid.hpp"
using namespace chassis;

pid::pid (double Kp, double Ki, double Kd, double target) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->target = target;
  this->pTime = steady_clock::now();
}

double pid::update (double input, double target) {
  this->target = target;
  this->dt = duration_cast<duration<double>>(steady_clock::now() - pTime);

  double error = target - input;

  double p = this->Kp * error;

  this->integral += error * this->dt.count();
  double i = this->Ki * this->integral;

  double d = this->Kd * ((error - this->error) / this->dt.count());

  this->error = error;
  this->pTime = steady_clock::now();

  double output = p + i + d;
  return output;
}

double pid::update (double input) {
  double error = this->target - input;
  this->dt = duration_cast<duration<double>>(steady_clock::now() - pTime);

  double p = this->Kp * error;

  this->integral += error * this->dt.count();
  double i = this->Ki * this->integral;

  double d = this->Kd * ((error - this->error) / this->dt.count());

  this->error = error;
  this->pTime = steady_clock::now();

  double output = p + i + d;
  return output;
}

void pid::setTarget (double target) {
  this->target = target;
}