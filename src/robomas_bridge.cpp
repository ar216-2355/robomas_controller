#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <cstring>

// カスタムメッセージ
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"

// 共通ヘッダ (STM32と同じもの)
#include "usb_packet.hpp" 

using namespace std::chrono_literals;

class RobomasBridge : public rclcpp::Node {
public:
    RobomasBridge() : Node("robomas_node") {
        // パラメータ読み込み
        this->declare_parameter("port_name", "/dev/ttyACM0");
        std::string port_name = this->get_parameter("port_name").as_string();
        
        // シリアルポートを開く
        if (!open_serial(port_name)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name.c_str());
            return; // エラー処理
        }

        // パラメータからPID設定を読み込む
        load_pid_params();

        // Publisher / Subscriber
        pub_feedback_ = this->create_publisher<robomas_interfaces::msg::RobomasFrame>("robomas/feedback", 10);
        
        sub_cmd_ = this->create_subscription<robomas_interfaces::msg::RobomasPacket>(
            "robomas/cmd", 10, std::bind(&RobomasBridge::cmd_callback, this, std::placeholders::_1));
            
        sub_can_ = this->create_subscription<robomas_interfaces::msg::CanFrame>(
            "robomas/can_tx", 10, std::bind(&RobomasBridge::can_callback, this, std::placeholders::_1));

        // タイマー
        // 1. 制御周期 (10ms): マイコンへ駆動指令を送る & 受信データチェック
        control_timer_ = this->create_wall_timer(10ms, std::bind(&RobomasBridge::control_loop, this));
        
        // 2. ターミナル表示 (100ms)
        display_timer_ = this->create_wall_timer(100ms, std::bind(&RobomasBridge::display_loop, this));

        RCLCPP_INFO(this->get_logger(), "Robomas Controller Started.");
    }

private:
    int serial_fd_ = -1;
    USBCtrlPacket tx_packet_; // 送信用バッファ
    MotorUnit current_targets_[16]; // 現在の目標値を保持

    // ハンドシェイク用
    std::vector<PIDConfig> pid_configs_;
    uint16_t current_pid_mask_ = 0;

    rclcpp::Publisher<robomas_interfaces::msg::RobomasFrame>::SharedPtr pub_feedback_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasPacket>::SharedPtr sub_cmd_;
    rclcpp::Subscription<robomas_interfaces::msg::CanFrame>::SharedPtr sub_can_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    
    // 最新のフィードバックデータ保持用
    USBFeedbackPacket last_feedback_;

    // -----------------------------------------------------------------
    // 初期化・パラメータ読み込み
    // -----------------------------------------------------------------
    bool open_serial(const std::string& port) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;
        
        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode
        tcsetattr(serial_fd_, TCSANOW, &options);
        return true;
    }

    void load_pid_params() {
        for(int i=0; i<16; i++) {
            std::string prefix = "motor" + std::to_string(i);
            // パラメータ宣言と取得 (エラー処理は省略)
            PIDConfig cfg;
            cfg.motor_id = i;
            
            auto get_p = [&](std::string name, float def) {
                this->declare_parameter(prefix + "." + name, def);
                return (float)this->get_parameter(prefix + "." + name).as_double();
            };

            cfg.speed_kp = get_p("speed_kp", 0.0);
            cfg.speed_ki = get_p("speed_ki", 0.0);
            cfg.speed_kd = get_p("speed_kd", 0.0);
            cfg.speed_i_limit = get_p("speed_i_limit", 0.0);
            cfg.speed_output_limit = get_p("speed_limit", 16000.0);
            
            cfg.position_kp = get_p("pos_kp", 0.0);
            // ... (他も同様に) ...
            
            pid_configs_.push_back(cfg);
        }
    }

    // -----------------------------------------------------------------
    // 受信コールバック (ROSトピック -> 内部バッファ)
    // -----------------------------------------------------------------
    void cmd_callback(const robomas_interfaces::msg::RobomasPacket::SharedPtr msg) {
        for(int i=0; i<16; i++) {
            current_targets_[i].mode = msg->motors[i].mode;
            current_targets_[i].target = msg->motors[i].target;
        }
    }

    void can_callback(const robomas_interfaces::msg::CanFrame::SharedPtr msg) {
        // CAN送信要求が来たら、即座にシリアルへ書き込む
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_SEND_CAN;
        pkt.payload.can_tx.can_id = msg->id;
        pkt.payload.can_tx.dlc = msg->dlc;
        memcpy(pkt.payload.can_tx.data, msg->data.data(), 8);
        pkt.checksum = calc_checksum(&pkt);
        
        write(serial_fd_, &pkt, sizeof(pkt));
    }

    // -----------------------------------------------------------------
    // メインループ (10ms)
    // -----------------------------------------------------------------
    void control_loop() {
        if (serial_fd_ < 0) return;

        // 1. シリアル受信処理
        // 本来はリングバッファでパケット解析すべきですが、簡易的に読み捨て例
        uint8_t buffer[256];
        int n = read(serial_fd_, buffer, sizeof(USBFeedbackPacket));
        if (n == sizeof(USBFeedbackPacket)) {
            USBFeedbackPacket* fb = (USBFeedbackPacket*)buffer;
            if (fb->header == 0x5A5A) { // チェックサム確認は省略
                last_feedback_ = *fb;
                current_pid_mask_ = fb->pid_configured_mask;
                publish_status(); // ROSへPublish
            }
        }

        // 2. ハンドシェイク処理 (PID設定)
        if (current_pid_mask_ != 0xFFFF) {
            // まだ設定されていないモーターがある場合、PID設定パケットを送る
            for (int i = 0; i < 16; i++) {
                if (!((current_pid_mask_ >> i) & 1)) {
                    send_pid_config(i);
                    // 一度に全部送ると詰まる可能性があるので、1ループ数個にするか
                    // ここではシンプルにループごとに未設定を全部送る
                }
            }
            return; // 設定中は駆動コマンドを送らない方が安全
        }

        // 3. 駆動指令送信
        send_drive_command();
    }

    // -----------------------------------------------------------------
    // 送信ヘルパー
    // -----------------------------------------------------------------
    void send_drive_command() {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_DRIVE_ALL;
        
        for(int i=0; i<16; i++) {
            pkt.payload.drive[i] = current_targets_[i];
        }
        pkt.checksum = calc_checksum(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }

    void send_pid_config(int id) {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_SET_PID;
        pkt.payload.pid = pid_configs_[id];
        pkt.checksum = calc_checksum(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }
    
    uint16_t calc_checksum(USBCtrlPacket* pkt) {
        uint16_t sum = 0;
        uint8_t* p = (uint8_t*)pkt;
        for(size_t i=0; i<sizeof(USBCtrlPacket)-2; i++) sum += p[i];
        return sum;
    }

    // -----------------------------------------------------------------
    // 表示・Publish関連
    // -----------------------------------------------------------------
    void publish_status() {
        auto msg = robomas_interfaces::msg::RobomasFrame();
        msg.header.stamp = this->now();
        msg.system_state = last_feedback_.system_state;
        msg.pid_configured_mask = last_feedback_.pid_configured_mask;
        
        for(int i=0; i<16; i++) {
            msg.angle[i] = last_feedback_.motors[i].angle;
            msg.velocity[i] = last_feedback_.motors[i].velocity;
            // ...
        }
        pub_feedback_->publish(msg);
    }

    void display_loop() {
        // ターミナル表示 (エスケープシーケンスで画面クリアすると綺麗)
        printf("\033[H\033[J"); // Clear Screen
        printf("=== Robomas Controller ===\n");
        printf("State: %d, PID Mask: %04X\n", last_feedback_.system_state, current_pid_mask_);
        printf("ID | Mode | Target |  FB Vel  |  FB Pos  \n");
        printf("---|------|--------|----------|----------\n");
        for(int i=0; i<16; i++) {
             printf("%2d |  %d   | %6.1f | %8.1f | %8.1f \n", 
                i, current_targets_[i].mode, current_targets_[i].target,
                last_feedback_.motors[i].velocity, last_feedback_.motors[i].angle);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobomasBridge>());
    rclcpp::shutdown();
    return 0;
}