#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bitset>
#include <sstream>
#include <algorithm> // for std::min and std::max

using namespace cv;
using namespace std;

class SensorReader : public rclcpp::Node {
public:
    SensorReader() : Node("sensor_reader"),
                     serial_port_(-1),
                     image_(500, 500, CV_8UC3, Scalar(255, 255, 255)),
                     oneces_(0)
    {
        // パブリッシャーの初期化
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // タイマーの設定（100msごとに実行）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
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
        serial_port_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            return;
        }

        // シリアルポートの設定
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 bits per byte
        tty.c_iflag &= ~IGNBRK;                         // disable break processing
        tty.c_lflag = 0;                                // no signaling chars, no echo
        tty.c_oflag = 0;                                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;                            // read doesn't block
        tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xON/xOFF ctrl
        tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
        tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
        tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                        // no flow control

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port initialized.");
    }

    void timer_callback() {
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not initialized.");
            return;
        }

        uint8_t bytesWaiting;
        if (ioctl(serial_port_, FIONREAD, &bytesWaiting) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ioctl FIONREAD failed: %s", strerror(errno));
            return;
        }

        if (bytesWaiting > 25) {
            uint8_t read_buf[256];
            memset(read_buf, 0, sizeof(read_buf));
            int n = read(serial_port_, read_buf, sizeof(read_buf));

            if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
                return;
            }

            if (n == 0) {
                RCLCPP_WARN(this->get_logger(), "No data read from serial port.");
                return;
            }

            process_data(read_buf, n);
        }
    }

    void process_data(uint8_t* data, int size) {
        // ここでは最大10バイトをg_にコピー
        memset(g_, 0, sizeof(g_));
        memcpy(g_, data, std::min(size, static_cast<int>(sizeof(g_))));

        // センサー値の更新
        update_sensor_values();

        // 位置計算と速度指令値の生成
        calculate_and_publish_velocity();

        // 画像の更新と表示
        update_display();
    }

    void update_sensor_values() {
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
    }

    void calculate_and_publish_velocity() {
        int x = calculate_x_position();

        // 速度指令値の生成と発行
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = map_to_velocity(x);
        cmd_vel_pub_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x = %f", twist_msg.linear.x);
    }

    void update_display() {
        // 画像を白でクリア
        image_ = cv::Scalar(255, 255, 255);

        // センサー値に基づいて円を描画
        draw_sensor_circles();

        // OpenCVウィンドウの更新
        cv::imshow("img", image_);
        cv::waitKey(1);
    }

    int calculate_x_position() {
        float weighted_sum_x = 0.0f;
        float weight_sum = 0.0f;

        // センサー位置の定義
        const std::pair<int, int> sensor_positions[] = {
            {250, 250},  // センター
            {250, 125},  // 上
            {250, 375},  // 下
            {125, 250},  // 左
            {375, 250}   // 右
        };

        const uint16_t* sensor_values[] = {
            bset1_, bset2_, bset3_, bset4_, bset5_
        };

        for (int i = 0; i < 5; i++) {
            float weight = static_cast<float>(sensor_values[i][0]);
            weighted_sum_x += weight * sensor_positions[i].first;
            weight_sum += weight;
        }

        if (weight_sum > 0.0f) {
            return static_cast<int>(weighted_sum_x / weight_sum);
        }
        return 250; // デフォルト値（中央）
    }

    float map_to_velocity(int x) {
        const float MAX_VEL = 1.0f;  // 最大速度[m/s]
        // xを0-500の範囲から0-MAX_VELにスケール
        return (static_cast<float>(x) / 500.0f) * MAX_VEL;
    }

    void draw_sensor_circles() {
        const std::pair<int, int> positions[] = {
            {250, 250},  // センター
            {250, 125},  // 上
            {250, 375},  // 下
            {125, 250},  // 左
            {375, 250}   // 右
        };

        const uint16_t* values[] = {
            bset1_, bset2_, bset3_, bset4_, bset5_
        };

        // 各センサーの円を描画
        for (int i = 0; i < 5; i++) {
            // センサー値のスケーリングを調整（0-1000の範囲を想定）
            float normalized_value = static_cast<float>(values[i][0]) / 1000.0f;
            
            // 半径を計算（最小20、最大100）
            int radius = static_cast<int>(20 + normalized_value * 80);
            radius = std::max(20, std::min(100, radius));

            try {
                cv::circle(image_,
                          cv::Point(positions[i].first, positions[i].second),
                          radius,
                          cv::Scalar(255, 0, 0),  // 青色
                          2);  // 線の太さ
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Drawing circle %d: pos=(%d,%d), radius=%d", 
                    i, positions[i].first, positions[i].second, radius);
            }
            catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), 
                    "OpenCV error while drawing circle %d: %s", 
                    i, e.what());
            }
        }

        // 現在位置の表示（赤い点）
        try {
            int current_x = calculate_x_position();
            current_x = std::max(10, std::min(490, current_x));

            cv::circle(image_,
                      cv::Point(current_x, 250),
                      10,  // 固定半径
                      cv::Scalar(0, 0, 255),  // 赤色
                      -1);  // 塗りつぶし
        }
        catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "OpenCV error while drawing position marker: %s", 
                e.what());
        }
    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    cv::Mat image_;
    int oneces_;

    uint8_t g_[10];
    uint16_t bset1_[1], bset2_[1], bset3_[1], bset4_[1], bset5_[1];
    uint16_t min1_[1], min2_[1], min3_[1], min4_[1], min5_[1];
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}