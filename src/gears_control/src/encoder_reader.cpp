#include "gears_control/encoder_reader.hpp"
#include <cmath>
#include <iostream>

QuadratureEncoder::QuadratureEncoder(int gpio_a, int gpio_b, double wheel_radius_m, int ticks_per_rev, const std::string& name)
: pin_a_(gpio_a), pin_b_(gpio_b), name_(name) {
  // Circumference = 2 * pi * r
  double circumference = 2.0 * M_PI * wheel_radius_m;
  meters_per_tick_ = circumference / static_cast<double>(ticks_per_rev);

  chip_ = gpiod_chip_open_by_name("gpiochip0");
  if (!chip_) {
    std::cerr << "[ERROR] Failed to open gpiochip0 for encoder " << name_ << "\n";
    return;
  }

  line_a_ = gpiod_chip_get_line(chip_, pin_a_);
  line_b_ = gpiod_chip_get_line(chip_, pin_b_);

  if (!line_a_ || !line_b_) {
    std::cerr << "[ERROR] Failed to get GPIO lines for encoder " << name_ << "\n";
    return;
  }

  // Request rising-edge hardware events on Channel A and input mode on Channel B
  if (gpiod_line_request_rising_edge_events(line_a_, name_.c_str()) < 0 ||
      gpiod_line_request_input(line_b_, name_.c_str()) < 0) {
    std::cerr << "[ERROR] Failed to request GPIO interrupts for " << name_ << "\n";
    return;
  }

  running_ = true;
  worker_thread_ = std::thread(&QuadratureEncoder::interruptWorker, this);
}

QuadratureEncoder::~QuadratureEncoder() {
  running_ = false;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  if (line_a_) gpiod_line_release(line_a_);
  if (line_b_) gpiod_line_release(line_b_);
  if (chip_) gpiod_chip_close(chip_);
}

void QuadratureEncoder::interruptWorker() {
  struct timespec timeout = {0, 50000000L}; // 50 ms polling timeout so thread exits cleanly on shutdown
  struct gpiod_line_event event;

  while (running_) {
    int ret = gpiod_line_event_wait(line_a_, &timeout);
    if (ret > 0) {
      if (gpiod_line_event_read(line_a_, &event) == 0) {
        // Check Channel B level at the exact rising edge of Channel A to determine direction
        int b_val = gpiod_line_get_value(line_b_);
        if (b_val == 0) {
          tick_count_.fetch_add(1, std::memory_order_relaxed);  // Forward
        } else {
          tick_count_.fetch_sub(1, std::memory_order_relaxed);  // Reverse
        }
      }
    }
  }
}

double QuadratureEncoder::computeVelocityMps(double dt_seconds) {
  if (dt_seconds <= 0.0) return 0.0;

  int64_t current_ticks = tick_count_.load(std::memory_order_relaxed);
  int64_t delta_ticks = current_ticks - prev_tick_count_;
  prev_tick_count_ = current_ticks;

  double distance_m = static_cast<double>(delta_ticks) * meters_per_tick_;
  return distance_m / dt_seconds;
}

double QuadratureEncoder::getTotalDistanceMeters() const {
  return static_cast<double>(tick_count_.load(std::memory_order_relaxed)) * meters_per_tick_;
}