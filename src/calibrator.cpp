// calibrator.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <chrono>
#include <array>

using namespace std::chrono_literals;

class Calibrator : public rclcpp::Node
{
public:
  Calibrator()
  : Node("sensor_calibrator"),
    state_(State::WAIT_DARK),
    sample_count_(0)
  {
    // Subscribe to 5 sensor topics
    for (int i = 0; i < 5; ++i) {
      auto topic = "sensor" + std::to_string(i+1);
      subs_[i] = this->create_subscription<std_msgs::msg::UInt16>(
        topic, 10,
        [this, i](const std_msgs::msg::UInt16::SharedPtr msg) {
          raw_[i] = msg->data;
        });
    }

    // Record the start time and prompt user to place sensors in dark
    start_time_ = now();
    RCLCPP_INFO(this->get_logger(),
      "Dark calibration: please place sensors in a dark area. Waiting 5 seconds...");

    // Begin periodic timer
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Calibrator::on_timer, this));
  }

private:
  // Calibration states
  enum class State { WAIT_DARK, CALIB_DARK, WAIT_BRIGHT, CALIB_BRIGHT, DONE };

  // Timer callback to drive state machine
  void on_timer()
  {
    auto elapsed = now() - start_time_;

    switch (state_) {
      case State::WAIT_DARK:
        if (elapsed >= 5s) {
          // Start dark sampling
          cal_min_.fill(UINT16_MAX);
          sample_count_ = 0;
          state_ = State::CALIB_DARK;
          RCLCPP_INFO(this->get_logger(), "Starting dark sampling (500 samples)...");
        }
        break;

      case State::CALIB_DARK:
        if (sample_count_ < N_) {
          // Update minimum for each sensor
          for (int i = 0; i < 5; ++i) {
            cal_min_[i] = std::min(cal_min_[i], raw_[i]);
          }
          ++sample_count_;
        } else {
          // Dark calibration complete
          RCLCPP_INFO(this->get_logger(), "Dark calibration complete.");
          // Move to bright wait
          state_ = State::WAIT_BRIGHT;
          start_time_ = now();
          RCLCPP_INFO(this->get_logger(),
            "Bright calibration: please place sensors in a bright area. Waiting 5 seconds...");
        }
        break;

      case State::WAIT_BRIGHT:
        if (elapsed >= 5s) {
          // Start bright sampling
          cal_max_.fill(0);
          sample_count_ = 0;
          state_ = State::CALIB_BRIGHT;
          RCLCPP_INFO(this->get_logger(), "Starting bright sampling (500 samples)...");
        }
        break;

      case State::CALIB_BRIGHT:
        if (sample_count_ < N_) {
          // Update maximum for each sensor
          for (int i = 0; i < 5; ++i) {
            cal_max_[i] = std::max(cal_max_[i], raw_[i]);
          }
          ++sample_count_;
        } else {
          // Bright calibration complete
          RCLCPP_INFO(this->get_logger(), "Bright calibration complete.");
          print_results();
          state_ = State::DONE;
        }
        break;

      case State::DONE:
        // Do nothing once calibration is done
        break;
    }
  }

  // Print the calibration results
  void print_results()
  {
    for (int i = 0; i < 5; ++i) {
      RCLCPP_INFO(this->get_logger(),
        "Sensor %d: min = %u, max = %u",
        i+1, cal_min_[i], cal_max_[i]);
    }
  }

  // Helper to get current time (no longer const)
  rclcpp::Time now() { return this->get_clock()->now(); }

  static constexpr int N_ = 500;  // Number of samples per calibration
  State state_;
  size_t sample_count_;
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<uint16_t,5> raw_{};
  std::array<uint16_t,5> cal_min_;
  std::array<uint16_t,5> cal_max_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subs_[5];
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Calibrator>());
  rclcpp::shutdown();
  return 0;
}
