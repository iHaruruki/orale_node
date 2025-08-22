#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class SensorReader : public rclcpp::Node
{
public:
  SensorReader()
  : Node("sensor_reader"), serial_port_(-1), bytes_waiting_(0)
  {
    // Publisher
    for (int i = 0; i < SENSOR_COUNT; ++i) {
      std::string topic = std::string("sensor") + std::to_string(i+1);
      sensor_pub_[i] = this->create_publisher<std_msgs::msg::UInt16>(
        topic, 10);
    }

    init_serial();

    // 50msごとに read_frame → publish
    timer_ = this->create_wall_timer(
      50ms, std::bind(&SensorReader::timer_callback, this));
  }

  ~SensorReader()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
    }
  }

private:
  static constexpr int SENSOR_COUNT = 9;

  void init_serial()
  {
    serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial port open failed: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    termios tty;
    tcgetattr(serial_port_, &tty);
    cfsetospeed(&tty, B1152000);
    cfsetispeed(&tty, B1152000);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VTIME] = 5;
    tty.c_cc[VMIN]  = 0;
    tcsetattr(serial_port_, TCSANOW, &tty);

    RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
  }

  void read_frame()
  {
    // シリアル未初期化なら何もしない
    if (serial_port_ < 0) {
      return;
    }

    // 内部バッファに溜まっているバイト数を取得
    ioctl(serial_port_, FIONREAD, &bytes_waiting_);
    if (bytes_waiting_ < 30) {
      // ヘッダ含め30バイト未満なら読み捨て
      return;
    }

    std::vector<uint8_t> buf(bytes_waiting_);
    int n = ::read(serial_port_, buf.data(), buf.size());
    if (n < 2 + 2 * SENSOR_COUNT) {
      // センサ全数分のデータがない
      return;
    }

    // 先頭から 0xFF 0xFF … のヘッダを探す
    int idx = -1;
    for (int i = 0; i <= n - 2 - 2 * SENSOR_COUNT; ++i) {
      if (buf[i] == 0xFF && buf[i+1] == 0xFF) {
        idx = i + 2;  // データ部スタート位置
        break;
      }
    }
    if (idx < 0) {
      return;
    }

    // SENSOR_COUNTセンサ×2バイトのデータを big-endian で取得（範囲チェックあり）
    for (int i = 0; i < SENSOR_COUNT; ++i) {
      int off = idx + 2 * i;
      if (off + 1 >= n) {
        // 想定外の短いフレーム
        return;
      }
      raw_[i] = (uint16_t(buf[off]) << 8)
              |  uint16_t(buf[off + 1]);
    }
  }

  void timer_callback()
  {
    read_frame();

    auto msg = std_msgs::msg::UInt16();
    for (int i = 0; i < SENSOR_COUNT; ++i) {
      if (!sensor_pub_[i]) {
        RCLCPP_WARN(this->get_logger(), "publisher %d not initialized", i);
        continue;
      }
      msg.data = raw_[i];
      sensor_pub_[i]->publish(msg);
    }
  }

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr sensor_pub_[SENSOR_COUNT];
  rclcpp::TimerBase::SharedPtr timer_;
  int serial_port_;
  int bytes_waiting_;
  uint16_t raw_[SENSOR_COUNT];
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorReader>());
  rclcpp::shutdown();
  return 0;
}
