

#ifndef GEARS_CONTROL_ENCODER_READER_HPP_
#define GEARS_CONTROL_ENCODER_READER_HPP_

#include <atomic>
#include <string>
#include <thread>
#include <gpiod.h>

class QuadratureEncoder {
public:
  QuadratureEncoder(int gpio_a, int gpio_b, double wheel_radius_m, int ticks_per_rev, const std::string& name);
  ~QuadratureEncoder();

  // Called every 20 ms (50 Hz) by controlLoop() to get linear velocity in m/s
  double computeVelocityMps(double dt_seconds);

  // Returns total accumulated distance traveled in meters
  double getTotalDistanceMeters() const;

private:
  void interruptWorker();

  int pin_a_;
  int pin_b_;
  double meters_per_tick_;
  std::string name_;

  std::atomic<int64_t> tick_count_{0};
  int64_t prev_tick_count_{0};

  std::atomic<bool> running_{false};
  std::thread worker_thread_;

  gpiod_chip* chip_ = nullptr;
  gpiod_line* line_a_ = nullptr;
  gpiod_line* line_b_ = nullptr;
};

#endif  // GEARS_CONTROL_ENCODER_READER_HPP_