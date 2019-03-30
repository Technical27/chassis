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
  return this->update(input);
}

double pid::update (double input) {
  double error = this->target - input;
  if (fabs(error) < 0.05) return this->output;
  duration<double> dt = duration_cast<duration<double>>(steady_clock::now() - pTime);

  double p = this->Kp * error;

  fabs(this->integral) > 300 ? this->integral = 0 : this->integral += (error * dt.count());
  double i = this->Ki * this->integral;

  double d = this->Kd * ((error - this->error) / dt.count());

  this->error = error;
  this->pTime = steady_clock::now();
  double output = p + i + d;
  if (this->limited) {
    if (output > this->max) output = this->max;
    if (output < this->min) output = this->min;
  }
  this->output = output;
  return output;
}

void pid::setTarget (double target) {
  this->target = target;
}

void pid::setLimits (double min, double max) {
  this->limited = true;
  this->min = min;
  this->max = max;
}

void pid::removeLimits () {
  this->limited = false;
}