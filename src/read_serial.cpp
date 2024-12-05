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

#define MAX_LINEAR_VELOCITY 1.0
#define MAX_ANGULAR_VELOCITY 1.5

using namespace cv;
using namespace std;

class SensorToCmdVel : public rclcpp::Node {
public:
    SensorToCmdVel() : Node("sensor_to_cmd_vel")
    {
        init_serial();
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&SensorToCmdVel::read_sensor_and_publish, this));
    }

    ~SensorToCmdVel()
    {
        close(serial_port_);
    }

private:
    void init_serial() {
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        serial_port_ = open("/dev/ttyACM0", O_RDWR);
		// Check for errors
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            return;
        }

		// Read in existing settings, and handle any error
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
	
		uint8_t bytesWaiting;

        // Save tty settings, also checking for error
		if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
			printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		}

		memset(&bset12, '\0', sizeof(bset1));
		memset(&bset22, '\0', sizeof(bset2));
		memset(&bset32, '\0', sizeof(bset3));
		memset(&bset42, '\0', sizeof(bset4));
		memset(&bset52, '\0', sizeof(bset5));
		int countup = 0;
	}

	void read_sensor_and_publish()
	{
		uint8_t bytes_waiting;
		ioctl(serial_port_, FIONREAD, &bytes_waiting);

		if(bytes_waiting > 25){
			uint8_t read_buf[bytes_waiting];
			memset(&read_buf, '\0', sizeof(read_buf));
			//int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));

			for(i = 0; i < sizeof(read_buf); i++){//find the head point
				//cout << "|check(" <<  std::bitset<8>{read_buf[i]} << endl;//data check
				if((read_buf[i] == 0xff)&&(read_buf[i+1] == 0xff)&&(read_buf[i+2] != 0xff)&&(read_buf[i+12] == 0xff)&&(read_buf[i+13] == 0xff)){
				 	break;
				 	} 
			 	}
			i+=2;	

			memset(&g, '\0', sizeof(g));
			memcpy(&g, &read_buf[i], sizeof(g));

			memset(&bset1, '\0', sizeof(bset1));
			memset(&bset2, '\0', sizeof(bset2));
			memset(&bset3, '\0', sizeof(bset3));
			memset(&bset4, '\0', sizeof(bset4));
			memset(&bset5, '\0', sizeof(bset5));

			mempcpy(&g, &read_buf[i], sizeof(g));
			mempcpy(&bset1, &g[0], 1);bset1[0] = bset1[0] << bset1[0] << 8; memcpy(&bset1, &g[1], 1);	//read buff
			mempcpy(&bset2, &g[2], 1);bset2[0] = bset2[0] << bset2[0] << 8; memcpy(&bset2, &g[3], 1);
			mempcpy(&bset3, &g[4], 1);bset3[0] = bset3[0] << bset3[0] << 8; memcpy(&bset3, &g[5], 1);
			mempcpy(&bset4, &g[6], 1);bset4[0] = bset4[0] << bset4[0] << 8; memcpy(&bset4, &g[7], 1);
			mempcpy(&bset5, &g[8], 1);bset5[0] = bset5[0] << bset5[0] << 8; memcpy(&bset5, &g[9], 1);

			if(min1[0] == 0 && bset1[0] != 0 && (bset1[0]-bset12[0]) >= 0)memcpy(&min1,&bset1,1);
			if(min2[0] == 0 && bset2[0] != 0 && (bset2[0]-bset22[0]) >= 0)memcpy(&min2,&bset2,1);
			if(min3[0] == 0 && bset3[0] != 0 && (bset3[0]-bset32[0]) >= 0)memcpy(&min3,&bset3,1);
			if(min4[0] == 0 && bset4[0] != 0 && (bset4[0]-bset42[0]) >= 0)memcpy(&min4,&bset4,1);
			if(min5[0] == 0 && bset5[0] != 0 && (bset5[0]-bset52[0]) >= 0)memcpy(&min5,&bset5,1);

			if((min1[0] != 0) && (min2[0] != 0) && (min3[0] != 0) && (min4[0] != 0) && (min5[0] != 0)){
				oneces++;
			}

			RCLCPP_INFO(this->get_logger(), "b1data: %d", bset1[0]);
			RCLCPP_INFO(this->get_logger(), "b2data: %d", bset2[0]);
			RCLCPP_INFO(this->get_logger(), "b3data: %d", bset3[0]);
			RCLCPP_INFO(this->get_logger(), "b4data: %d", bset4[0]);
			RCLCPP_INFO(this->get_logger(), "b5data: %d", bset5[0]);

			int border = 130;
			if((bset1[0] > border) && (bset2[0] > border) && (bset3[0] > border) && (bset4[0] > border) && (bset5[0] > border)){
				image = cv::Scalar(255,255,255);
				uint16_t scor = 710;

				float mh1 = (bset1[0] - min1[0]);float mh12 = (scor - min1[0]);
				float mh2 = (bset2[0] - min2[0]);float mh22 = (scor - min2[0]);
				float mh3 = (bset3[0] - min3[0]);float mh32 = (scor - min3[0]);
				float mh4 = (bset4[0] - min4[0]);float mh42 = (scor - min4[0]);
				float mh5 = (bset5[0] - min5[0]);float mh52 = (scor - min5[0]);
				
				float s1 = mh1/mh12;
				float s2 = mh2/mh22; 
				float s3 = mh3/mh32;
				float s4 = mh4/mh42;
				float s5 = mh5/mh52;
				//printf("%f\n", s1);
				//std::cout << s5 << "," << s2 << "," << s3 << "," << s4 <<std::endl;
				circle(image, Point(250,250), (35+(65*s1)), cv::Scalar(255, 0,0),2);//center
				circle(image, Point(250,125), (35+(65*s2)), cv::Scalar(255, 0,0),2);//12
				circle(image, Point(250,375), (35+(65*s3)), cv::Scalar(255, 0,0),2);//6
				circle(image, Point(125,250), (35+(65*s4)), cv::Scalar(255, 0,0),2);//9
		  		circle(image, Point(375,250), (35+(65*s5)), cv::Scalar(255, 0,0),2);//3
				int x = (((s1*250)+(s2*250)+(s3*250)+(s4*25)+(s5*475))/(s1+s2+s3+s4+s5));
				int y = (((s1*250)+(s2*25)+(s3*475)+(s4*250)+(s5*250))/(s1+s2+s3+s4+s5));
				vel_x = x;
				
				cv::circle(image,Point(x,y), 10, cv::Scalar(0,0,255),-1);
			
				cv::imshow("img",image);
				key = cv::waitKey(10);
				//printf("\n");
			}

			// センサの値の読み取り (0~255)
			//uint8_t sensor_value = read_buf[0];

			// センサデータの値を速度に変換
			float linear_speed = (vel_x / 255.0) * MAX_LINEAR_VELOCITY;

			// cmd_velメッセージの作成
			geometry_msgs::msg::Twist cmd_vel_msg;
			cmd_vel_msg.linear.x = linear_speed;
			cmd_vel_msg.linear.y = 0.0;
			cmd_vel_msg.angular.z = 0.0;

			// cmd_velパブリッシュする
			cmd_vel_pub_->publish(cmd_vel_msg);

			//RCLCPP_INFO(this->get_logger(), "linear_speed: %f", linear_speed);
		}
	}

	int serial_port_;
	struct termios tty_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	cv::Point pos;
    cv::Mat image;

	uint8_t g[10];
	uint16_t bset1 [1], bset2 [1], bset3 [1], bset4 [1], bset5 [1];
	uint16_t bset12 [1], bset22 [1], bset32 [1], bset42 [1], bset52 [1];
	uint16_t min1 [1], min2 [1], min3 [1], min4 [1], min5 [1];
	int i;
	int va = 0;
	int oneces = 0;	
	int min13;
	int max1;
	int flag;	
	int key;
	int c = 500;
	int p = 0;      //position number
	int nv = 4;		//number of vertices
	int cen = 0;	//center
	int vel_x = 0, vel_y = 0;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SensorToCmdVel>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}