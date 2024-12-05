#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <unistd.h> 
#include <sys/ioctl.h>
#include <bitset>

using namespace std;
using namespace cv;

class SensorToCmdVel : public rclcpp::Node
{
public:
    SensorToCmdVel()
    : Node("sensor_to_cmd_vel"),
      pos_(Point(-10, -10)),
      image_(500, 500, CV_8UC3, Scalar(255, 255, 255)),
      serial_port_(-1),
      oneces_(0)
    {
        // cmd_vel パブリッシャーの作成
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // タイマーの設定（100msごとにコールバック実行）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorToCmdVel::read_sensor_and_publish, this));

        // シリアルポートの初期化
        init_serial();

        // OpenCVウィンドウの作成
        namedWindow("img", WINDOW_AUTOSIZE);
    }

    ~SensorToCmdVel()
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

        tty.c_cflag &= ~PARENB; // Disable parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        tty.c_cc[VTIME] = 10; // 1 second timeout
        tty.c_cc[VMIN] = 0;

        // Set baud rate
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    }

    void read_sensor_and_publish()
    {
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not initialized.");
            return;
        }

        uint8_t bytesWaiting;
        ioctl(serial_port_, FIONREAD, &bytesWaiting);

        if (bytesWaiting > 25) {
            uint8_t read_buf[bytesWaiting + 1];
            memset(read_buf, 0, sizeof(read_buf));
            int n = read(serial_port_, read_buf, sizeof(read_buf));

            if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
                return;
            }

            // 特定のデータパターンを探す
            int start_idx = -1;
            for(int i = 0; i < n - 13; i++) { // シーケンス長を考慮
                if(read_buf[i] == 0xff && read_buf[i+1] == 0xff && read_buf[i+2] != 0xff &&
                   read_buf[i+12] == 0xff && read_buf[i+13] == 0xff) {
                    start_idx = i + 2;
                    break;
                }
            }

            if(start_idx == -1) {
                RCLCPP_WARN(this->get_logger(), "Start sequence not found.");
                return;
            }

            // データをgにコピー
            memcpy(g_, &read_buf[start_idx], sizeof(g_));

            // bset1~bset5を生成
            for(int j = 0; j < 5; j++) {
                bset_[j][0] = (g_[j*2] << 8) | g_[j*2 + 1];
            }

            // 最小値の更新
            for(int j = 0; j < 5; j++) {
                if(min_[j][0] == 0 && bset_[j][0] != 0 && (bset_[j][0] - bset_prev_[j][0]) >= 0) {
                    min_[j][0] = bset_[j][0];
                }
            }

            // すべてのminが更新されたか確認
            bool all_min_updated = true;
            for(int j = 0; j < 5; j++) {
                if(min_[j][0] == 0) {
                    all_min_updated = false;
                    break;
                }
            }

            if(all_min_updated && oneces_ == 0) {
                oneces_ = 1;
            }

            // 前回のbsetを更新
            for(int j = 0; j < 5; j++) {
                bset_prev_[j][0] = bset_[j][0];
            }

            // データの表示
            for(int j = 0; j < 5; j++) {
                RCLCPP_INFO(this->get_logger(), "b%ddata: %d", j+1, bset_[j][0]);
            }

            // 速度計算の条件
            int border = 130;
            bool above_border = true;
            for(int j = 0; j < 5; j++) {
                if(bset_[j][0] <= border) {
                    above_border = false;
                    break;
                }
            }

            if(above_border) {
                uint16_t scor = 710;

                float mh[5];
                float mh_ratio[5];
                for(int j = 0; j < 5; j++) {
                    mh[j] = static_cast<float>(bset_[j][0] - min_[j][0]);
                    mh_ratio[j] = static_cast<float>(mh[j]) / static_cast<float>(scor - min_[j][0]);
                }

                float s[5];
                for(int j = 0; j < 5; j++) {
                    s[j] = mh[j] / mh_ratio[j];
                }

                // OpenCV描画
                image_ = Scalar(255, 255, 255); // 白で初期化
                circle(image_, Point(250,250), static_cast<int>(35 + (65 * s[0])), Scalar(255, 0, 0), 2);
                circle(image_, Point(250,125), static_cast<int>(35 + (65 * s[1])), Scalar(255, 0, 0), 2);
                circle(image_, Point(250,375), static_cast<int>(35 + (65 * s[2])), Scalar(255, 0, 0), 2);
                circle(image_, Point(125,250), static_cast<int>(35 + (65 * s[3])), Scalar(255, 0, 0), 2);
                circle(image_, Point(375,250), static_cast<int>(35 + (65 * s[4])), Scalar(255, 0, 0), 2);

                int x = static_cast<int>((s[0]*250 + s[1]*250 + s[2]*250 + s[3]*25 + s[4]*475) / (s[0] + s[1] + s[2] + s[3] + s[4]));
                int y = static_cast<int>((s[0]*250 + s[1]*25 + s[2]*475 + s[3]*250 + s[4]*250) / (s[0] + s[1] + s[2] + s[3] + s[4]));

                circle(image_, Point(x, y), 10, Scalar(0, 0, 255), -1);

                // 速度計算
                const float MAX_LINEAR_VELOCITY = 1.0; // 適切な最大速度に設定
                float linear_speed = (static_cast<float>(x) / 500.0f) * MAX_LINEAR_VELOCITY; // スケール調整

                // cmd_vel メッセージの作成とパブリッシュ
                geometry_msgs::msg::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = linear_speed;
                cmd_vel_pub_->publish(cmd_vel_msg);

                RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x = %f", linear_speed);
            }

            // OpenCVウィンドウの更新
            imshow("img", image_);
            waitKey(1);
        }
    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    Mat image_;
    Point pos_;
    int oneces_;

    uint8_t g_[10];
    uint16_t bset_[5][1];      // bset1 ~ bset5
    uint16_t bset_prev_[5][1]; // bset12 ~ bset52
    uint16_t min_[5][1];        // min1 ~ min5
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorToCmdVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}