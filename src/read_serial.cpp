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

			// センサの値の読み取りと速度を計算
			float linear_velocity = calculate_linear_velocity(read_buf);
			float angular_velocity = calculate_angular_velocity(read_buf);

			// cmd_valメッセージの作成
			auto message = geometry_msgs::msg::Twist();
			message.linear.x = linear_velocity;
			message.angular.z = angular_velocity;

            // cmd_velトピックにパブリッシュ
            publisher_->publish(message);
        }
    }

	float calculate_linear_velocity(uint8_t *read_buf) {
		return static_cast<float>(read_buf[0]) / 255.0 * 1.5;	// 0~255の値を0~1.5()に変換
	}

	float calculate_angular_velocity(uint8_t *read_buf) {
		return static_cast<float>(read_buf[1]) / 255.0 * 3.0 - 1.5;		// 0~255の値を-1.5~1.5に変換
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