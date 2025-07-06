// calibrator.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
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

    // Publishers for calibrated min/max arrays
    cal_min_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "calibration_min", 10);
    cal_max_pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "calibration_max", 10);

    // Prompt for dark calibration
    start_time_ = now();
    RCLCPP_INFO(this->get_logger(),
      "Dark calibration: please place sensors in a dark area. Waiting 5 seconds...");

    // Start the periodic timer
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Calibrator::on_timer, this));
  }

private:
  // Calibration states
  enum class State { WAIT_DARK, CALIB_DARK, WAIT_BRIGHT, CALIB_BRIGHT, DONE };

  void on_timer()
  {
    auto elapsed = now() - start_time_;

    switch (state_) {
      case State::WAIT_DARK:
        if (elapsed >= 5s) {
          // Begin dark sampling
          cal_min_.fill(UINT16_MAX);
          sample_count_ = 0;
          state_ = State::CALIB_DARK;
          RCLCPP_INFO(this->get_logger(), "Starting dark sampling (500 samples)...");
        }
        break;

      case State::CALIB_DARK:
        if (sample_count_ < N_) {
          // Update per-sensor minimum
          for (int i = 0; i < 5; ++i) {
            cal_min_[i] = std::min(cal_min_[i], raw_[i]);
          }
          ++sample_count_;
        } else {
          // Dark calibration done
          RCLCPP_INFO(this->get_logger(), "Dark calibration complete.");
          // Prompt for bright calibration
          state_ = State::WAIT_BRIGHT;
          start_time_ = now();
          RCLCPP_INFO(this->get_logger(),
            "Bright calibration: please place sensors in a bright area. Waiting 5 seconds...");
        }
        break;

      case State::WAIT_BRIGHT:
        if (elapsed >= 5s) {
          // Begin bright sampling
          cal_max_.fill(0);
          sample_count_ = 0;
          state_ = State::CALIB_BRIGHT;
          RCLCPP_INFO(this->get_logger(), "Starting bright sampling (500 samples)...");
        }
        break;

      case State::CALIB_BRIGHT:
        if (sample_count_ < N_) {
          // Update per-sensor maximum
          for (int i = 0; i < 5; ++i) {
            cal_max_[i] = std::max(cal_max_[i], raw_[i]);
          }
          ++sample_count_;
        } else {
          // Bright calibration done
          RCLCPP_INFO(this->get_logger(), "Bright calibration complete.");
          publish_results();
          state_ = State::DONE;
        }
        break;

      case State::DONE:
        // Nothing more to do
        break;
    }
  }

  // Publish the calibration arrays
  void publish_results()
  {
    // Log to console
    for (int i = 0; i < 5; ++i) {
      RCLCPP_INFO(this->get_logger(),
        "Sensor %d: min = %u, max = %u",
        i+1, cal_min_[i], cal_max_[i]);
    }

    // Prepare and publish min array
    std_msgs::msg::UInt16MultiArray min_msg;
    min_msg.data.insert(min_msg.data.end(), cal_min_.begin(), cal_min_.end());
    cal_min_pub_->publish(min_msg);

    // Prepare and publish max array
    std_msgs::msg::UInt16MultiArray max_msg;
    max_msg.data.insert(max_msg.data.end(), cal_max_.begin(), cal_max_.end());
    cal_max_pub_->publish(max_msg);

    RCLCPP_INFO(this->get_logger(),
      "Published calibration_min and calibration_max topics.");
  }

  // Helper to get current time
  rclcpp::Time now() { return this->get_clock()->now(); }

  static constexpr int N_ = 500;  // Number of samples per phase
  State state_;
  size_t sample_count_;
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<uint16_t,5> raw_{};
  std::array<uint16_t,5> cal_min_;
  std::array<uint16_t,5> cal_max_;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subs_[5];
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr cal_min_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr cal_max_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Calibrator>());
  rclcpp::shutdown();
  return 0;
}
