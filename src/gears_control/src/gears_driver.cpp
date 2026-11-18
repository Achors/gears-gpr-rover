#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gears_control/encoder_reader.hpp"
#include "gears_control/pid_controller.hpp"

using namespace std::chrono_literals;

class HardwareMotorOutput {
public:
  HardwareMotorOutput(int pwm_chip, int pwm_channel, int dir_gpio_pin, const std::string& name)
  : chip_num_(pwm_chip), channel_(pwm_channel), dir_pin_(dir_gpio_pin), name_(name) {
    base_path_ = "/sys/class/pwm/pwmchip" + std::to_string(chip_num_) + "/pwm" + std::to_string(channel_);
    exportPwm();
    setPeriod(50000); // 20 kHz
    setDutyCycle(0);
    enablePwm(true);
    initGpioDirection();
  }

  ~HardwareMotorOutput() {
    setDutyCycle(0);
    enablePwm(false);
    if (dir_line_) gpiod_line_release(dir_line_);
    if (gpio_chip_) gpiod_chip_close(gpio_chip_);
  }

  void setSpeedNormalized(double normalized_cmd) {
    normalized_cmd = std::clamp(normalized_cmd, -1.0, 1.0);
    int dir_val = (normalized_cmd >= 0.0) ? 1 : 0;
    if (dir_line_) gpiod_line_set_value(dir_line_, dir_val);
    uint64_t duty_ns = static_cast<uint64_t>(std::abs(normalized_cmd) * 50000.0);
    setDutyCycle(duty_ns);
  }

private:
  void writeSysfs(const std::string& path, const std::string& value) {
    std::ofstream fs(path);
    if (fs.is_open()) { fs << value; fs.close(); }
  }
  void exportPwm() {
    writeSysfs("/sys/class/pwm/pwmchip" + std::to_string(chip_num_) + "/export", std::to_string(channel_));
    std::this_thread::sleep_for(50ms);
  }
  void setPeriod(uint64_t period_ns) { writeSysfs(base_path_ + "/period", std::to_string(period_ns)); }
  void setDutyCycle(uint64_t duty_ns) { writeSysfs(base_path_ + "/duty_cycle", std::to_string(duty_ns)); }
  void enablePwm(bool enable) { writeSysfs(base_path_ + "/enable", enable ? "1" : "0"); }
  void initGpioDirection() {
    gpio_chip_ = gpiod_chip_open_by_name("gpiochip0");
    if (!gpio_chip_) return;
    dir_line_ = gpiod_chip_get_line(gpio_chip_, dir_pin_);
    if (dir_line_) gpiod_line_request_output(dir_line_, name_.c_str(), 0);
  }

  int chip_num_, channel_, dir_pin_;
  std::string name_, base_path_;
  gpiod_chip* gpio_chip_ = nullptr;
  gpiod_line* dir_line_ = nullptr;
};

class GearsRealtimeDriver : public rclcpp::Node {
public:
  GearsRealtimeDriver() : Node("gears_realtime_driver") {
    // Module 1: Hardware PWM & Direction Outputs
    left_motor_  = std::make_unique<HardwareMotorOutput>(0, 0, 23, "gears_left_dir");
    right_motor_ = std::make_unique<HardwareMotorOutput>(0, 1, 24, "gears_right_dir");

    // Module 2: Quadrature Encoders
    const double WHEEL_RADIUS = 0.10;
    const int TICKS_PER_REV = 360;
    left_encoder_  = std::make_unique<QuadratureEncoder>(17, 27, WHEEL_RADIUS, TICKS_PER_REV, "gears_left_enc");
    right_encoder_ = std::make_unique<QuadratureEncoder>(22, 5,  WHEEL_RADIUS, TICKS_PER_REV, "gears_right_enc");

    // Module 3: Closed-Loop PID Controllers (Starter gains — tune Kp, Ki, Kd on turf!)
    // Note: Keep out_max at 0.5 during bench tests if testing a 24V motor on a 36V PSU!
    left_pid_  = std::make_unique<DiscretePID>(0.8, 0.4, 0.02, -0.5, 0.5);
    right_pid_ = std::make_unique<DiscretePID>(0.8, 0.4, 0.02, -0.5, 0.5);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&GearsRealtimeDriver::cmdCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&GearsRealtimeDriver::imuCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);

    last_cmd_time_ = this->now();
    last_loop_time_ = this->now();

    control_timer_ = this->create_wall_timer(
      20ms, std::bind(&GearsRealtimeDriver::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Modules 1+2+3 Active: 20kHz PWM + Encoders + Closed-Loop PID + 30deg Tilt Cutoff.");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_linear_v_ = msg->linear.x;
    target_angular_w_ = msg->angular.z;
    last_cmd_time_ = this->now();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Convert quaternion to roll & pitch (in degrees)
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll_deg  = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    double sinp = 2.0 * (qw * qy - qz * qx);
    double pitch_deg = (std::abs(sinp) >= 1.0) ? std::copysign(90.0, sinp) : (std::asin(sinp) * 180.0 / M_PI);

    // Requirement R7: Cutoff if tilt >= 30 degrees
    if (std::abs(roll_deg) >= 30.0 || std::abs(pitch_deg) >= 30.0) {
      if (!tilt_fault_) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL TILT FAULT (Roll: %.1f deg, Pitch: %.1f deg) — MOTORS CUT!", roll_deg, pitch_deg);
      }
      tilt_fault_ = true;
    } else {
      tilt_fault_ = false;
    }
  }

  void controlLoop() {
    auto now = this->now();
    double dt_loop = (now - last_loop_time_).seconds();
    last_loop_time_ = now;

    // 1. Measure wheel velocities (Module 2)
    double v_meas_left  = left_encoder_->computeVelocityMps(dt_loop);
    double v_meas_right = right_encoder_->computeVelocityMps(dt_loop);

    const double L = 0.45;
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = (v_meas_left + v_meas_right) / 2.0;
    odom_msg.twist.twist.angular.z = (v_meas_right - v_meas_left) / L;
    odom_pub_->publish(odom_msg);

    // 2. Safety Checks: Rollover Tilt Cutoff OR 100ms Comms Watchdog (Requirements R6 & R7)
    double cmd_age_ms = (now - last_cmd_time_).seconds() * 1000.0;
    if (tilt_fault_ || cmd_age_ms > 100.0) {
      left_motor_->setSpeedNormalized(0.0);
      right_motor_->setSpeedNormalized(0.0);
      left_pid_->reset();
      right_pid_->reset();
      return;
    }

    // 3. Inverse Kinematics -> Target Wheel Velocities
    double v_left_target  = target_linear_v_ - (target_angular_w_ * L / 2.0);
    double v_right_target = target_linear_v_ + (target_angular_w_ * L / 2.0);

    // 4. Closed-Loop Discrete PID Output (Module 3)
    double pwm_cmd_left  = left_pid_->compute(v_left_target, v_meas_left, dt_loop);
    double pwm_cmd_right = right_pid_->compute(v_right_target, v_meas_right, dt_loop);

    left_motor_->setSpeedNormalized(pwm_cmd_left);
    right_motor_->setSpeedNormalized(pwm_cmd_right);
  }

  std::unique_ptr<HardwareMotorOutput> left_motor_;
  std::unique_ptr<HardwareMotorOutput> right_motor_;
  std::unique_ptr<QuadratureEncoder> left_encoder_;
  std::unique_ptr<QuadratureEncoder> right_encoder_;
  std::unique_ptr<DiscretePID> left_pid_;
  std::unique_ptr<DiscretePID> right_pid_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_loop_time_;

  double target_linear_v_ = 0.0;
  double target_angular_w_ = 0.0;
  bool tilt_fault_ = false;
};

int main(int argc, char * argv[]) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(3, &cpuset);
  pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

  struct sched_param param;
  param.sched_priority = 80;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GearsRealtimeDriver>());
  rclcpp::shutdown();
  return 0;
}