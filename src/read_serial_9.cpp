#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <bitset>
#include <sstream>
#include <vector>

using namespace cv;
using namespace std;

class SensorReader : public rclcpp::Node {
public:
    SensorReader() : Node("sensor_reader"),
                     serial_port_(-1),
                     image_(500, 500, CV_8UC3, Scalar(255, 255, 255)),
                     payload_size_(9 * 2) // 9 sensors * 2 bytes each
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SensorReader::timer_callback, this));

        namedWindow("img", WINDOW_AUTOSIZE);

        init_serial();
    }

    ~SensorReader()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
        destroyAllWindows();
    }

private:
    void init_serial()
    {
        struct termios tty;
        serial_port_ = open("/dev/ttyUSB0", O_RDWR);

        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            return;
        }

        if (tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            return;
        }

        // initialize arrays
        bset_.assign(9, 0);
        bset_prev_.assign(9, 0);
        minv_.assign(9, 0);
        oneces_ = 0;
        countup_ = 0;

        RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    }

    void timer_callback()
    {
        int byteswaiting = 0;
        if (serial_port_ < 0) return;

        ioctl(serial_port_, FIONREAD, &byteswaiting);

        // need at least header(2) + payload + tail(2)
        const int min_packet_len = 2 + payload_size_ + 2;
        if (byteswaiting >= min_packet_len) {
            vector<uint8_t> read_buf(byteswaiting + 1);
            memset(read_buf.data(), '\0', read_buf.size());
            ssize_t r = read(serial_port_, read_buf.data(), read_buf.size());
            if (r <= 0) return;

            int head_idx = -1;
            for (size_t k = 0; k + min_packet_len <= read_buf.size(); ++k) {
                if (read_buf[k] == 0xff && read_buf[k + 1] == 0xff) {
                    // check tail exists at expected location
                    size_t tail_pos = k + 2 + payload_size_;
                    if (tail_pos + 1 < read_buf.size() && read_buf[tail_pos] == 0xff && read_buf[tail_pos + 1] == 0xff) {
                        head_idx = k + 2; // payload starts after 0xff 0xff
                        break;
                    }
                }
            }

            if (head_idx < 0) return;

            // read payload
            vector<uint8_t> g(payload_size_);
            memcpy(g.data(), &read_buf[head_idx], payload_size_);

            // shift previous
            for (int k = 0; k < 9; ++k) bset_prev_[k] = bset_[k];

            // decode 9 sensors (big-endian 2 bytes each)
            for (int k = 0; k < 9; ++k) {
                uint16_t hi = g[2 * k];
                uint16_t lo = g[2 * k + 1];
                bset_[k] = (hi << 8) | lo;
            }

            // initial calibration: collect min values for each sensor
            if (oneces_ == 0) {
                bool all_set = true;
                for (int k = 0; k < 9; ++k) {
                    if (minv_[k] == 0) {
                        // require non-zero and non-decreasing compared to previous sample
                        if (bset_[k] != 0 && (bset_[k] - bset_prev_[k]) >= 0) {
                            minv_[k] = bset_[k];
                        } else {
                            all_set = false;
                        }
                    }
                }
                if (all_set) {
                    oneces_ = 1;
                    RCLCPP_INFO(this->get_logger(), "Calibration complete for 9 sensors.");
                }
            }

            // debug print
            for (int k = 0; k < 9; ++k) {
                std::cout << "b" << (k + 1) << "data(" << std::dec << bset_[k] << ") ";
            }
            std::cout << std::endl;

            int border = 130;
            bool above_border = true;
            for (int k = 0; k < 9; ++k) if (bset_[k] <= border) { above_border = false; break; }

            if (above_border && oneces_ != 0) {
                image_ = Scalar(255, 255, 255);
                const uint16_t scor = 710;

                vector<float> s(9, 0.0f);
                for (int k = 0; k < 9; ++k) {
                    float mh = static_cast<float>(bset_[k]) - static_cast<float>(minv_[k]);
                    float mh2 = static_cast<float>(scor) - static_cast<float>(minv_[k]);
                    if (mh2 <= 0.0f) s[k] = 0.0f;
                    else s[k] = mh / mh2;
                    if (s[k] < 0.0f) s[k] = 0.0f;
                    if (s[k] > 1.0f) s[k] = 1.0f;
                }

                // positions for 3x3 grid
                vector<Point> positions = {
                    {125,125}, {250,125}, {375,125},
                    {125,250}, {250,250}, {375,250},
                    {125,375}, {250,375}, {375,375}
                };

                for (int k = 0; k < 9; ++k) {
                    int radius = static_cast<int>(35 + (65 * s[k]));
                    circle(image_, positions[k], radius, Scalar(255, 0, 0), 2);
                }

                // weighted center
                float sumw = 0.0f;
                float wx = 0.0f, wy = 0.0f;
                for (int k = 0; k < 9; ++k) {
                    wx += s[k] * positions[k].x;
                    wy += s[k] * positions[k].y;
                    sumw += s[k];
                }
                int cx = 250, cy = 250;
                if (sumw > 0.0f) {
                    cx = static_cast<int>(wx / sumw);
                    cy = static_cast<int>(wy / sumw);
                }
                circle(image_, Point(cx, cy), 10, Scalar(0, 0, 255), -1);

                imshow("img", image_);
                key_ = waitKey(10);
            }
        }
    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    cv::Mat image_;

    const int payload_size_;
    vector<uint16_t> bset_;
    vector<uint16_t> bset_prev_;
    vector<uint16_t> minv_;

    int countup_ = 0;
    int oneces_ = 0;
    int key_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}