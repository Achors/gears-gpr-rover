
#ifndef GEARS_CONTROL_PID_CONTROLLER_HPP_
#define GEARS_CONTROL_PID_CONTROLLER_HPP_

#include <algorithm>

class DiscretePID {
public:
  DiscretePID(double kp, double ki, double kd, double out_min = -1.0, double out_max = 1.0)
  : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

  double compute(double setpoint, double measured, double dt) {
    if (dt <= 0.0) return 0.0;

    // If setpoint is near zero, reset integral so rover stops cleanly without creeping
    if (std::abs(setpoint) < 0.005 && std::abs(measured) < 0.02) {
      reset();
      return 0.0;
    }

    double error = setpoint - measured;

    // Proportional term
    double p_out = kp_ * error;

    // Integral term with anti-windup clamping
    integral_ += error * dt;
    double max_i = (ki_ > 0.0) ? (out_max_ / ki_) : 0.0;
    integral_ = std::clamp(integral_, -max_i, max_i);
    double i_out = ki_ * integral_;

    // Derivative term
    double derivative = (error - prev_error_) / dt;
    double d_out = kd_ * derivative;
    prev_error_ = error;

    double output = p_out + i_out + d_out;
    return std::clamp(output, out_min_, out_max_);
  }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

private:
  double kp_, ki_, kd_;
  double out_min_, out_max_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
};

#endif  // GEARS_CONTROL_PID_CONTROLLER_HPP_