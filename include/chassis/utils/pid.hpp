#pragma once
#include <chrono>
#include <ratio>
#include <cmath>
using namespace std::chrono;


namespace chassis {
class pid {
  public:
  /**
   * The constructor for the pid class, it takes the three constants used in pid control.
   * Optionally you can pass a target to set it in the constructor, otherwise you can set the target with the setTarget function.
  */
  pid(double Kp, double Ki, double Kd, double target = 0);

  /**
   * Calculates and returns the output from the pid loop.
   * With this funciton, you can also update the target.
  */
  double update(double input, double target);

  /**
   * Calculates and returns the output from the pid loop.
  */
  double update(double input);

  /**
   * sets the target for the pid loop.
  */
  void setTarget(double target);

  /**
   * sets the min and max for the pid loop.
  */
  void setLimits(double min, double max);

  /**
   * removes limits for the pid loop.
  */
  void removeLimits();

  private:
  double Kp, Ki, Kd, integral, error, target, min, max, output;
  bool limited;
  steady_clock::time_point pTime;
};
}