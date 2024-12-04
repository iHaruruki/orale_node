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
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace cv;
using namespace std;

class SerialNode : public rclcpp::Node {
public:
    SerialNode() : Node("serial_node") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            30ms, std::bind(&SerialNode::timer_callback, this));
        init_serial();
    }

private:
    void init_serial() {
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        serial_port_ = open("/dev/ttyACM0", O_RDWR);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            return;
        }

        if(tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            return;
        }

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
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VTIME] = 10;   
        tty.c_cc[VMIN] = 0;
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            return;
        }
    }

    void timer_callback() {
        uint8_t bytesWaiting;
        ioctl(serial_port_, FIONREAD, &bytesWaiting);

        if (bytesWaiting > 25) {
            uint8_t read_buf[bytesWaiting + 1];
            memset(&read_buf, '\0', sizeof(read_buf));
            read(serial_port_, &read_buf, sizeof(read_buf));

            // データ処理とcmd_velメッセージの作成
            auto message = geometry_msgs::msg::Twist();
            // ここでmessage.linear.xやmessage.angular.zを設定
            publisher_->publish(message);
        }
    }

    int serial_port_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}