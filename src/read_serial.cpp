#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bitset>
#include <vector>
#include <cstring>
#include <chrono>

using namespace cv;
using namespace std::chrono_literals;

class SensorReader : public rclcpp::Node {
public:
    SensorReader()
    : Node("sensor_reader"),
      serial_port_(-1),
      image_(500, 500, CV_8UC3, Scalar(255, 255, 255)),
      countup(0), i(0), oneces(0), x(0), y(0), key(0), byteswaiting(0)
    {
        // cmd_vel パブリッシャー
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // センサ値用パブリッシャー
        sensor1_pub_ = this->create_publisher<std_msgs::msg::UInt16>("sensor1", 10);
        sensor2_pub_ = this->create_publisher<std_msgs::msg::UInt16>("sensor2", 10);
        sensor3_pub_ = this->create_publisher<std_msgs::msg::UInt16>("sensor3", 10);
        sensor4_pub_ = this->create_publisher<std_msgs::msg::UInt16>("sensor4", 10);
        sensor5_pub_ = this->create_publisher<std_msgs::msg::UInt16>("sensor5", 10);

        // 50ms ごとのタイマー
        timer_ = this->create_wall_timer(
            50ms, std::bind(&SensorReader::timer_callback, this));

        // OpenCV ウィンドウ
        namedWindow("img", WINDOW_AUTOSIZE);

        // シリアル初期化
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
        serial_port_ = open("/dev/ttyACM0", O_RDWR);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            return;
        }
        if (tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr error %i: %s", errno, strerror(errno));
            return;
        }
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
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
        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr error %i: %s", errno, strerror(errno));
            return;
        }

        // キャリブレーション用配列の初期化
        memset(bset12, 0, sizeof(bset12));
        memset(bset22, 0, sizeof(bset22));
        memset(bset32, 0, sizeof(bset32));
        memset(bset42, 0, sizeof(bset42));
        memset(bset52, 0, sizeof(bset52));

        RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    }

    void timer_callback()
    {
        // シリアル入力バイト数を取得
        ioctl(serial_port_, FIONREAD, &byteswaiting);
        if (byteswaiting > 25) {
            std::vector<uint8_t> read_buf(byteswaiting);
            int n = read(serial_port_, read_buf.data(), read_buf.size());
            if (n <= 0) return;

            // フレームヘッダ探査
            int idx = 0;
            for (; idx < n - 13; ++idx) {
                if (read_buf[idx] == 0xff && read_buf[idx+1] == 0xff &&
                    read_buf[idx+2] != 0xff &&
                    read_buf[idx+12] == 0xff && read_buf[idx+13] == 0xff) {
                    break;
                }
            }
            idx += 2;

            // 10 バイトのデータ取得
            uint8_t g[10];
            memcpy(g, &read_buf[idx], sizeof(g));

            // 各センサ値を結合 (big-endian)
            bset1[0] = (uint16_t(g[0]) << 8) | g[1];
            bset2[0] = (uint16_t(g[2]) << 8) | g[3];
            bset3[0] = (uint16_t(g[4]) << 8) | g[5];
            bset4[0] = (uint16_t(g[6]) << 8) | g[7];
            bset5[0] = (uint16_t(g[8]) << 8) | g[9];

            // 初回のみ最小値キャリブレーション
            if (oneces == 0) {
                memcpy(min1, bset1, sizeof(bset1));
                memcpy(min2, bset2, sizeof(bset2));
                memcpy(min3, bset3, sizeof(bset3));
                memcpy(min4, bset4, sizeof(bset4));
                memcpy(min5, bset5, sizeof(bset5));
                oneces = 1;
                RCLCPP_INFO(this->get_logger(), "Calibration done.");
            }

            // 値のログ出力
            RCLCPP_INFO(this->get_logger(),
                        "b1:%u b2:%u b3:%u b4:%u b5:%u",
                        bset1[0], bset2[0], bset3[0], bset4[0], bset5[0]);

            // 各センサ値を publish
            std_msgs::msg::UInt16 msg16;
            msg16.data = bset1[0]; sensor1_pub_->publish(msg16);
            msg16.data = bset2[0]; sensor2_pub_->publish(msg16);
            msg16.data = bset3[0]; sensor3_pub_->publish(msg16);
            msg16.data = bset4[0]; sensor4_pub_->publish(msg16);
            msg16.data = bset5[0]; sensor5_pub_->publish(msg16);

            // 位置計算＆描画（既存ロジック）
            int border = 50;
            if (bset1[0] > border && bset2[0] > border &&
                bset3[0] > border && bset4[0] > border &&
                bset5[0] > border)
            {
                image_ = Scalar(255,255,255);
                float scor = 710.0f;
                float bs[5] = {
                    float(bset1[0]), float(bset2[0]),
                    float(bset3[0]), float(bset4[0]),
                    float(bset5[0])
                };
                float mn[5] = {
                    float(min1[0]), float(min2[0]),
                    float(min3[0]), float(min4[0]),
                    float(min5[0])
                };
                float s[5];
                for (int k = 0; k < 5; ++k) {
                    s[k] = (bs[k] - mn[k]) / (scor - mn[k]);
                }
                // 円描画
                circle(image_, Point(250,250), int(35+65*s[0]), Scalar(255,0,0), 2);//中心
                circle(image_, Point(250,125), int(35+65*s[2]), Scalar(255,0,0), 2);//左
                circle(image_, Point(250,375), int(35+65*s[1]), Scalar(255,0,0), 2);//右
                circle(image_, Point(125,250), int(35+65*s[3]), Scalar(255,0,0), 2);//上
                circle(image_, Point(375,250), int(35+65*s[4]), Scalar(255,0,0), 2);//下

                // 重心計算
                x = int((s[0]*250 + s[1]*250 + s[2]*250 + s[3]*25 + s[4]*475) /
                        (s[0]+s[1]+s[2]+s[3]+s[4]));
                y = int((s[0]*250 + s[2]*25  + s[1]*475 + s[3]*250 + s[4]*250) /
                        (s[0]+s[1]+s[2]+s[3]+s[4]));
                RCLCPP_INFO(this->get_logger(), "x:%d, y:%d", x, y);

                // Twist publish
                geometry_msgs::msg::Twist twist;
                if      (x > 280) twist.angular.z =  0.2f;
                else if (x < 220) twist.angular.z = -0.2f;
                else if (y > 280) twist.linear.x  = -0.1f;
                else if (y < 220) twist.linear.x  =  0.1f;
                /*else{
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                }*/
                cmd_vel_pub_->publish(twist);

                // 重心点表示
                circle(image_, Point(x,y), 10, Scalar(0,0,255), -1);
                imshow("img", image_);
                waitKey(1);
            }
        }
    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr
        sensor1_pub_, sensor2_pub_, sensor3_pub_, sensor4_pub_, sensor5_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    cv::Mat image_;
    uint8_t byteswaiting;
    int countup, i, oneces, x, y, key;
    uint16_t bset1[1], bset2[1], bset3[1], bset4[1], bset5[1];
    uint16_t bset12[1], bset22[1], bset32[1], bset42[1], bset52[1];
    uint16_t min1[1], min2[1], min3[1], min4[1], min5[1];
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorReader>());
    rclcpp::shutdown();
    return 0;
}
