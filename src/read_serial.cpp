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

using namespace cv;
using namespace std;

Point pos(-10,-10);//first position
Mat image(500, 500, CV_8UC3);//y,x

class SensorReader : public rclcpp::Node {
public:
    SensorReader() : Node("sensor_reader"),
                     serial_port_(-1),
                     image_(500, 500, CV_8UC3, Scalar(255, 255, 255))
    {
        // パブリッシャーの初期化
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // タイマーの設定（50msごとに実行）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SensorReader::timer_callback, this));

        // OpenCVウィンドウの作成
        namedWindow("img", WINDOW_AUTOSIZE);

        // シリアルポートの初期化
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
    void init_serial() {
        struct termios tty;
        serial_port_ = open("/dev/ttyACM0", O_RDWR);

        // Check for errors
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            return;
        }

        // Read in existing setttings, and handle any error
        if (tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            return;
        }

        // Set tty settings, also check for error
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0; // 0 means read doesn't block

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            return;
        }

        Point pos(-10,-10); //first position
        memset(&bset12, '\0', sizeof(bset1));
        memset(&bset22, '\0', sizeof(bset2));
        memset(&bset32, '\0', sizeof(bset3));
        memset(&bset42, '\0', sizeof(bset4));
        memset(&bset52, '\0', sizeof(bset5));

        countup = 0;

        RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    }

    void timer_callback() {

        ioctl(serial_port_, FIONREAD, &byteswaiting);	//read data from serial

		if (byteswaiting > 25){
			uint8_t read_buf[byteswaiting + 1];
		  	memset(&read_buf, '\0', sizeof(read_buf));
		  	read(serial_port_, &read_buf, sizeof(read_buf));

			//printf("2bsize(%d\n", byteswaiting);
			//printf("3r_bsz(%ld\n", sizeof(read_buf));
			//cout << "6r_bBy(" << std::bitset<16>{read_buf[0]} << endl;

			for(i = 0; i < sizeof(read_buf); i++){//find the head point
				//cout << "|check(" <<  std::bitset<8>{read_buf[i]} << endl;//data check
				if((read_buf[i] == 0xff)&&(read_buf[i+1] == 0xff)&&(read_buf[i+2] != 0xff)&&(read_buf[i+12] == 0xff)&&(read_buf[i+13] == 0xff)){
				 	break;
				 	}
			 	}
			i+=2;
			//printf("7point(%d\n", i);
			//---------------------------------------------------
			memset(&g, '\0', sizeof(g));
			memcpy(&g, &read_buf[i], sizeof(g));
			//for(int l = 0; l < sizeof(g); l++) cout << "|gdata(" <<  std::bitset<8>{g[l]} << endl;//data check

			memset(&bset1, '\0', sizeof(bset1));//fill in the array
			memset(&bset2, '\0', sizeof(bset2));
			memset(&bset3, '\0', sizeof(bset3));
			memset(&bset4, '\0', sizeof(bset4));
			memset(&bset5, '\0', sizeof(bset5));

			memcpy(&g, &read_buf[i], sizeof(g));
			memcpy(&bset1, &g[0], 1);bset1[0] = bset1[0] << 8;memcpy(&bset1, &g[1], 1);//read buff
			memcpy(&bset2, &g[2], 1);bset2[0] = bset2[0] << 8;memcpy(&bset2, &g[3], 1);
			memcpy(&bset3, &g[4], 1);bset3[0] = bset3[0] << 8;memcpy(&bset3, &g[5], 1);
			memcpy(&bset4, &g[6], 1);bset4[0] = bset4[0] << 8;memcpy(&bset4, &g[7], 1);
			memcpy(&bset5, &g[8], 1);bset5[0] = bset5[0] << 8;memcpy(&bset5, &g[9], 1);




			while(oneces == 0){
				ioctl(serial_port_, FIONREAD, &byteswaiting);	//read data from serial

				if (byteswaiting > 25){
					uint8_t read_buf [byteswaiting+1];
				  	memset(&read_buf, '\0', sizeof(read_buf));
				  	read(serial_port_, &read_buf, sizeof(read_buf));

					//printf("2bsize(%d\n", byteswaiting);
					//printf("3r_bsz(%ld\n", sizeof(read_buf));
					//cout << "6r_bBy(" << std::bitset<16>{read_buf[0]} << endl;

					for(i = 0; i < sizeof(read_buf); i++){//find the head point
						//cout << "|check(" <<  std::bitset<8>{read_buf[i]} << endl;//data check
						if((read_buf[i] == 0xff)&&(read_buf[i+1] == 0xff)&&(read_buf[i+2] != 0xff)&&(read_buf[i+12] == 0xff)&&(read_buf[i+13] == 0xff)){
						 	break;
						}
					}
					i+=2;
					//printf("7point(%d\n", i);
					//---------------------------------------------------
					memset(&g, '\0', sizeof(g));
					memcpy(&g, &read_buf[i], sizeof(g));
					//for(int l = 0; l < sizeof(g); l++) cout << "|gdata(" <<  std::bitset<8>{g[l]} << endl;//data check

					memcpy(&bset12, &bset1, 1);
					memcpy(&bset22, &bset2, 1);
					memcpy(&bset32, &bset3, 1);
					memcpy(&bset42, &bset4, 1);
					memcpy(&bset52, &bset5, 1);

					memset(&bset1, '\0', sizeof(bset1));//fill in the array
					memset(&bset2, '\0', sizeof(bset2));
					memset(&bset3, '\0', sizeof(bset3));
					memset(&bset4, '\0', sizeof(bset4));
					memset(&bset5, '\0', sizeof(bset5));

					memcpy(&g, &read_buf[i], sizeof(g));
					memcpy(&bset1, &g[0], 1);bset1[0] = bset1[0] << 8;memcpy(&bset1, &g[1], 1);//read buff
					memcpy(&bset2, &g[2], 1);bset2[0] = bset2[0] << 8;memcpy(&bset2, &g[3], 1);
					memcpy(&bset3, &g[4], 1);bset3[0] = bset3[0] << 8;memcpy(&bset3, &g[5], 1);
					memcpy(&bset4, &g[6], 1);bset4[0] = bset4[0] << 8;memcpy(&bset4, &g[7], 1);
					memcpy(&bset5, &g[8], 1);bset5[0] = bset5[0] << 8;memcpy(&bset5, &g[9], 1);



	 				if(min1[0] == 0 && bset1[0] != 0 && (bset1[0]-bset12[0]) >= 0)memcpy(&min1,&bset1,1);
					if(min2[0] == 0 && bset2[0] != 0 && (bset2[0]-bset22[0]) >= 0)memcpy(&min2,&bset2,1);
					if(min3[0] == 0 && bset3[0] != 0 && (bset3[0]-bset32[0]) >= 0)memcpy(&min3,&bset3,1);
					if(min4[0] == 0 && bset4[0] != 0 && (bset4[0]-bset42[0]) >= 0)memcpy(&min4,&bset4,1);
					if(min5[0] == 0 && bset5[0] != 0 && (bset5[0]-bset52[0]) >= 0)memcpy(&min5,&bset5,1);


					if((min1[0] != 0) && (min2[0] != 0) && (min3[0] != 0) && (min4[0] != 0) && (min5[0] != 0)){
					oneces++;

					break;
					}
				}
			}
			//cout << "b1data(" << std::dec << (bset1[0]) << endl;
            RCLCPP_INFO(this->get_logger(), "b1:%d", bset1[0]);
			//cout << "b2data(" << std::dec << (bset2[0]) << endl;
            RCLCPP_INFO(this->get_logger(), "b2:%d", bset2[0]);
			//cout << "b3data(" << std::dec << (bset3[0]) << endl;
            RCLCPP_INFO(this->get_logger(), "b3:%d", bset3[0]);
			//cout << "b4data(" << std::dec << (bset4[0]) << endl;
            RCLCPP_INFO(this->get_logger(), "b4:%d", bset4[0]);
			//cout << "b5data(" << std::dec << (bset5[0]) << endl;
            RCLCPP_INFO(this->get_logger(), "b5:%d", bset5[0]);

			int border = 130;
			if((bset1[0] > border)&&(bset2[0] > border)&&(bset3[0] > border)&&(bset4[0] > border)&&(bset5[0] > border)){
				image = Scalar(255, 255, 255);
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
                RCLCPP_INFO(this->get_logger(), "x: %d, y: %d", x, y);

                // 速度

				geometry_msgs::msg::Twist twist_msg;
                if(x > 280)
                {
                    twist_msg.angular.z = 0.2;
                }
                else if(x < 220)
                {
                    twist_msg.angular.z = -0.2;
                }
                else if(y > 280)
                {
                    twist_msg.linear.x = -0.1;
                }
                else if(y < 220)
                {
                    twist_msg.linear.x = 0.1;
                }
                else
                {
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                }
				cmd_vel_pub_->publish(twist_msg);

				circle(image,Point(x,y), 10, cv::Scalar(0,0,255),-1);
			//}
			imshow("img",image);
			key = waitKey(10);
			//printf("\n");
			}

		}

        void init_serial();
        void timer_callback();
        void process_data(uint8_t* data, int size);
        void update_sensor_values();
        void calculate_and_publish_velocity();
        void update_display();
        int calculate_x_position();
        float map_to_velocity(int x);
        void draw_sensor_circles();

    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    cv::Mat image_;
    uint8_t byteswaiting;

    int countup;
    int i, oneces;
    int x, y;
    int key;

    uint8_t g[10];
    uint16_t bset1[1], bset2[1], bset3[1], bset4[1], bset5[1];
    uint16_t bset12[1], bset22[1], bset32[1], bset42[1], bset52[1];
    uint16_t min1[1], min2[1], min3[1], min4[1], min5[1];
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
