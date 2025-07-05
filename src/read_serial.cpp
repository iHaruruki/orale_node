#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>

// custom clamp function
template <typename T>
T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

using namespace cv;
using namespace std::chrono_literals;

class SensorReader : public rclcpp::Node {
public:
    SensorReader()
    : Node("sensor_reader"),
      serial_port_(-1),
      image_(500, 500, CV_8UC3, Scalar(255, 255, 255)),
      stable_twist_count_(0)  // removed trailing comma
    {
        // Publisher
        //cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sensor_pub_[0] = this->create_publisher<std_msgs::msg::UInt16>("sensor1", 10);
        sensor_pub_[1] = this->create_publisher<std_msgs::msg::UInt16>("sensor2", 10);
        sensor_pub_[2] = this->create_publisher<std_msgs::msg::UInt16>("sensor3", 10);
        sensor_pub_[3] = this->create_publisher<std_msgs::msg::UInt16>("sensor4", 10);
        sensor_pub_[4] = this->create_publisher<std_msgs::msg::UInt16>("sensor5", 10);

        // OpenCV window
        namedWindow("img", WINDOW_AUTOSIZE);

        // シリアル初期化
        init_serial();

        // Calibration
        calibrate();

        // Timer
        timer_ = this->create_wall_timer(
            50ms, std::bind(&SensorReader::timer_callback, this));
    }

    ~SensorReader()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
        destroyAllWindows();
    }

private:
    // mnmber fanction
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr sensor_pub_[5];
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    cv::Mat image_;
    uint8_t byteswaiting;
    int stable_twist_count_;
    geometry_msgs::msg::Twist last_twist_;

    // 生データ格納バッファ
    uint16_t raw_[5];
    // キャリブレーション結果
    uint16_t cal_min_[5];
    uint16_t cal_max_[5];

    void init_serial()
    {
        struct termios tty;
        serial_port_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        tcgetattr(serial_port_, &tty);
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VTIME] = 5;
        tty.c_cc[VMIN] = 0;
        tcsetattr(serial_port_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    }

    void calibrate()
    {
        constexpr int N = 100; //Number of samples
        RCLCPP_INFO(this->get_logger(),
                    "=== Calibration Start ===\n"
                    "Place the sensor surface in a dark location.\n"
                    " Reading will begin in 5 seconds...");

        std::this_thread::sleep_for(5s);
        sample_range(cal_min_, N);

        RCLCPP_INFO(this->get_logger(),
                    "Place the sensor surface in a bright location.\n"
                    " Reading will begin in 5 seconds...");

        std::this_thread::sleep_for(5s);
        sample_range(cal_max_, N);

        for (int i = 0; i < 5; ++i) {
            RCLCPP_INFO(this->get_logger(),
                        "Sensor %d: min=%u, max=%u",
                        i+1, cal_min_[i], cal_max_[i]);
        }
        RCLCPP_INFO(this->get_logger(), "=== Calibration Done ===");
    }

    void sample_range(uint16_t out[5], int N)
    {
        // Dummy implementation: simply set the calibration values to zero for all sensors.
        for (int i = 0; i < 5; i++) {
            out[i] = (out == cal_min_ ? UINT16_MAX : 0);
        }

        // Sampling
        for(int cnt = 0; cnt < N; ++cnt){
            read_frame();   //raw_[]
            for(int i = 0; i < 5; ++i){
                if (out == cal_min_) {
                    out[i] = std::min(out[i], raw_[i]);
                } else {
                    out[i] = std::max(out[i], raw_[i]);
                }
            }
            std::this_thread::sleep_for(20ms);
        }
    }

    void read_frame()
    {
        ioctl(serial_port_, FIONREAD, &byteswaiting);
        if(byteswaiting < 14) return;

        std::vector<uint8_t> buf(byteswaiting);
        int n = ::read(serial_port_, buf.data(), buf.size());
        if(n < 14) return;

        // Find the header position
        int idx = 0;
        for (; idx <= n - 14; ++idx) {
            if (buf[idx] == 0xFF && buf[idx+1] == 0xFF
             && buf[idx+12] == 0xFF && buf[idx+13] == 0xFF)
            {
                idx += 2;
                break;
            }
        }
        if (idx > n-14) return;

        // Get 10 bytes of data
        uint8_t g[10];
        memcpy(g, &buf[idx], 10);

        // big-endian to uint16
        for (int i = 0; i < 5; ++i) {
            raw_[i] = (uint16_t(g[2*i]) << 8) | g[2*i+1];
        }
    }

    // Timer
    void timer_callback()
    {
        read_frame();

        float norm[5];
        for (int i = 0; i < 5; ++i) {
            float d = float(cal_max_[i] - cal_min_[i]);
            norm[i] = (d > 0.0f)
                ? (raw_[i] - float(cal_min_[i])) / d
                : 0.0f;
            norm[i] = clamp(norm[i], 0.0f, 1.0f);
        }

        // Publish sensor data
        std_msgs::msg::UInt16 msg;
        for (int i = 0; i < 5; ++i) {
            msg.data = uint16_t(norm[i] * 1000);
            sensor_pub_[i]->publish(msg);
        }

        // 描画
        // something
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorReader>());
    rclcpp::shutdown();
    return 0;
}
