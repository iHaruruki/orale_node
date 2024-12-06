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
                     image_(500, 500, CV_8UC3, Scalar(255, 255, 255)),
                     
    
    // パブリッシャーの初期化
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // タイマーの設定（100msごとに実行）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SensorReader::timer_callback, this));

    // OpenCVウィンドウの作成
    namedWindow("img", WINDOW_AUTOSIZE);

    // シリアルポートの初期化
    init_serial();
    

    ~SensorReader()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
        destroyAllWindows();
    }

private:
    void init_serial() {
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
        
        ioctl(serial_port_, FIONREAD, &byteswaiting);   // read data from serial port

        if(byteswaiting >= 20){
            uint8_t read_buf[byteswaiting];
            memset(read_buf, '\0', sizeof(read_buf));
            read(serial_port_, &read_buf, sizeof(read_buf));

            RCLCPP_INFO(this->get_logger(), "2bsize(%d)", byteswaiting);
            RCLCPP_INFO(this->get_logger(), "3r_bsz(%ld)", read_buf[0]);
            RCLCPP_INFO(this->get_logger(), "6r_bBy:", std::bitset<16>{read_buf[0]});

            // find the head point
            for(i = 0; i < sizeof(read_buf); i++){
                if((read_buf[i] == 0xff) && (read_buf[i+1] == 0xff) && (read_buf[i+2] != 0xff) && (read_buf[i+12] == 0xff) && (read_buf[i+13] == 0xff)){
                    //break;
                }   
            }

            i+=2;
            RCLCPP_INFO(this->get_logger(), "7point(%d)", i);

            memset(&g, '\0', sizeof(g));
            memcpy(&g, &read_buf[i], sizeof(g));

            // fill in the array
            memset(&bset1, '\0', sizeof(bset1));
            memset(&bset2, '\0', sizeof(bset2));
            memset(&bset3, '\0', sizeof(bset3));
            memset(&bset4, '\0', sizeof(bset4));
            memset(&bset5, '\0', sizeof(bset5));

            memcpy(&g,&read_buf[i], sizeof(g));
            memcpy(&bset1, &g[0], sizeof(bset1)); bset1[0] = bset1[0] << 8; memcpy(&bset1, &g[1], 1);
            memcpy(&bset2, &g[2], sizeof(bset2)); bset2[0] = bset2[0] << 8; memcpy(&bset2, &g[3], 1);
            memcpy(&bset3, &g[4], sizeof(bset3)); bset3[0] = bset3[0] << 8; memcpy(&bset3, &g[5], 1);
            memcpy(&bset4, &g[6], sizeof(bset4)); bset4[0] = bset4[0] << 8; memcpy(&bset4, &g[7], 1);
            memcpy(&bset5, &g[8], sizeof(bset5)); bset5[0] = bset5[0] << 8; memcpy(&bset5, &g[9], 1);
        
            if(min1[0] == 0 && bset1[0] != 0 && (bset1[0] - bset12[0]) >= 0) memcpy(&min1,&bset1,1);
            if(min2[0] == 0 && bset2[0] != 0 && (bset2[0] - bset22[0]) >= 0) memcpy(&min2,&bset2,1);
            if(min3[0] == 0 && bset3[0] != 0 && (bset3[0] - bset32[0]) >= 0) memcpy(&min3,&bset3,1);
            if(min4[0] == 0 && bset4[0] != 0 && (bset4[0] - bset42[0]) >= 0) memcpy(&min4,&bset4,1);
            if(min5[0] == 0 && bset5[0] != 0 && (bset5[0] - bset52[0]) >= 0) memcpy(&min5,&bset5,1);
            
            if((min1[0] )!= 0 && (min2[0] != 0) && (min3[0] != 0) && (min4[0] != 0) && (min5[0] != 0)){
                oneces++;
                //break;
            }
        }
        
    }

    void process_data(uint8_t* data, int size) {
        // ここでは最大10バイトをg_にコピー
        /*memset(g_, 0, sizeof(g_));
        memcpy(g_, data, std::min(size, static_cast<int>(sizeof(g_))));

        // センサー値の更新
        update_sensor_values();

        // 位置計算と速度指令値の生成
        calculate_and_publish_velocity();

        // 画像の更新と表示
        update_display();*/
    }

    /*void update_sensor_values() {
        bset1_[0] = (g_[0] << 8) | g_[1];
        bset2_[0] = (g_[2] << 8) | g_[3];
        bset3_[0] = (g_[4] << 8) | g_[5];
        bset4_[0] = (g_[6] << 8) | g_[7];
        bset5_[0] = (g_[8] << 8) | g_[9];

        // センサー値をログ出力
        RCLCPP_INFO(this->get_logger(), "b1data: %d", bset1_[0]);
        RCLCPP_INFO(this->get_logger(), "b2data: %d", bset2_[0]);
        RCLCPP_INFO(this->get_logger(), "b3data: %d", bset3_[0]);
        RCLCPP_INFO(this->get_logger(), "b4data: %d", bset4_[0]);
        RCLCPP_INFO(this->get_logger(), "b5data: %d", bset5_[0]);
    }*/

    /*void calculate_and_publish_velocity() {
        int x = calculate_x_position();

        // 速度指令値の生成と発行
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = map_to_velocity(x);
        cmd_vel_pub_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x = %f", twist_msg.linear.x);
    }*/

    /*void update_display() {
        // 画像を白でクリア
        image_ = cv::Scalar(255, 255, 255);

        // センサー値に基づいて円を描画
        draw_sensor_circles();

        // OpenCVウィンドウの更新
        cv::imshow("img", image_);
        cv::waitKey(1);
    }*/

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    cv::Mat image_;
    uint8_t byteswaiting;

    int countup;
    int i, oneces;
    
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