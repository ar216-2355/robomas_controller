#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <cstring>
#include <algorithm> 
#include <cerrno> // ★追加: errno, EAGAIN などの定義を使うため

#include "std_msgs/msg/bool.hpp"
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"

// ★ここに提示されたヘッダーファイルの内容が入っている前提
#include "usb_packet.hpp" 

using namespace std::chrono_literals;

class RobomasBridge : public rclcpp::Node {
public:
    RobomasBridge() : Node("robomas_node") {
        RCLCPP_INFO(this->get_logger(), "Sizeof FB Packet: %lu", sizeof(USBFeedbackPacket));
        // --- パラメータ設定 ---
        this->declare_parameter("port_name", "/dev/ttyACM0");
        std::string port_name = this->get_parameter("port_name").as_string();
        
        if (!open_serial(port_name)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name.c_str());
            return; 
        }

        load_pid_params();

        param_subscriber_ = this->add_on_set_parameters_callback(
        std::bind(&RobomasBridge::param_callback, this, std::placeholders::_1));

        // --- Pub/Sub設定 ---
        pub_feedback_ = this->create_publisher<robomas_interfaces::msg::RobomasFrame>("robomas/feedback", 10);
        pub_can_rx_   = this->create_publisher<robomas_interfaces::msg::CanFrame>("robomas/can_rx", 10);
        
        sub_cmd_ = this->create_subscription<robomas_interfaces::msg::RobomasPacket>(
            "robomas/cmd", 10, std::bind(&RobomasBridge::cmd_callback, this, std::placeholders::_1));
            
        sub_can_ = this->create_subscription<robomas_interfaces::msg::CanFrame>(
            "robomas/can_tx", 10, std::bind(&RobomasBridge::can_callback, this, std::placeholders::_1));

        sub_emergency_ = this->create_subscription<std_msgs::msg::Bool>(
            "robomas/emergency", 10, std::bind(&RobomasBridge::emergency_callback, this, std::placeholders::_1));

        // --- タイマー設定 ---
        control_timer_ = this->create_wall_timer(10ms, std::bind(&RobomasBridge::control_loop, this));
        display_timer_ = this->create_wall_timer(200ms, std::bind(&RobomasBridge::display_loop, this));

        RCLCPP_INFO(this->get_logger(), "Robomas Controller Started (Full Spec).");
    }

private:
    int serial_fd_ = -1;
    std::vector<uint8_t> rx_buf_;
    
    MotorUnit current_targets_[16];
    std::vector<PIDConfig> pid_configs_;
    
    USBFeedbackPacket last_feedback_;
    uint16_t current_pid_mask_ = 0; // ハンドシェイク用

    rclcpp::Publisher<robomas_interfaces::msg::RobomasFrame>::SharedPtr pub_feedback_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr pub_can_rx_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasPacket>::SharedPtr sub_cmd_;
    rclcpp::Subscription<robomas_interfaces::msg::CanFrame>::SharedPtr sub_can_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;

    // -----------------------------------------------------------------
    // シリアルポート設定
    // -----------------------------------------------------------------
    bool open_serial(const std::string& port) {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;
        
        struct termios options;
        tcgetattr(serial_fd_, &options);

        // --- ここから修正 ---
        
        // 入力フラグ (Input Flags)
        // ICRNL: CR(0x0D) を NL(0x0A) に変換しない (重要！)
        // IXON/IXOFF: ソフトウェアフロー制御(Ctrl+S/Q)を無効化 (重要！)
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

        // 出力フラグ (Output Flags)
        // OPOST: 出力処理(改行変換など)を無効化
        options.c_oflag &= ~OPOST;

        // ローカルフラグ (Local Flags)
        // ICANON: カノニカルモード(行単位入力)を無効化
        // ECHO: エコーバック無効化
        // ISIG: シグナル文字(Ctrl+Cなど)を無効化
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

        // 制御フラグ (Control Flags)
        // CSIZE: 文字サイズマスククリア
        // PARENB: パリティなし
        // CSTOPB: ストップビット1
        // CS8: 8ビット通信
        options.c_cflag &= ~(CSIZE | PARENB);
        options.c_cflag |= CS8;
        
        // --- 修正ここまで ---

        // ボーレート設定 (USB CDCなので実は何でも良いが設定しておく)
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        // 設定を即時反映
        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIFLUSH);
        return true;
    }

    // -----------------------------------------------------------------
    // PIDパラメータ読み込み
    // -----------------------------------------------------------------
    void load_pid_params() {
        for(int i=0; i<16; i++) {
            std::string prefix = "motor" + std::to_string(i);
            PIDConfig cfg;
            cfg.motor_id = i + 1; // 1-16 (マイコン側実装に合わせる)
            
            auto get_p = [&](std::string name, float def) {
                this->declare_parameter(prefix + "." + name, def);
                return (float)this->get_parameter(prefix + "." + name).as_double();
            };

            cfg.speed_kp = get_p("speed_kp", 1.0);
            cfg.speed_ki = get_p("speed_ki", 0.0);
            cfg.speed_kd = get_p("speed_kd", 0.0);
            cfg.speed_i_limit = get_p("speed_i_limit", 1000.0);
            cfg.speed_output_limit = get_p("speed_limit", 16384.0);
            
            cfg.position_kp = get_p("pos_kp", 1.0);
            cfg.position_ki = get_p("pos_ki", 0.0);
            cfg.position_kd = get_p("pos_kd", 0.0);
            cfg.position_i_limit = get_p("pos_i_limit", 0.0);
            cfg.position_output_limit = get_p("pos_limit", 5000.0);
            
            pid_configs_.push_back(cfg);
            
            current_targets_[i].mode = 0; 
            current_targets_[i].target = 0.0f;
        }
    }

    // -----------------------------------------------------------------
    // メイン制御ループ (10ms)
    // -----------------------------------------------------------------
    void control_loop() {
        if (serial_fd_ < 0) return;

        process_serial_read();

        // ★追加: マイコンがEMERGENCYモード(0)の時は、PC側の目標値も強制的にリセット（脱力）
        if (last_feedback_.system_state == 0) {
            for (int i = 0; i < 16; i++) {
                current_targets_[i].mode = 3;     // 無効モード(脱力)
                current_targets_[i].target = 0.0; // 目標値0
            }
        }

        // ★ハンドシェイク: 全ビット(FFFF)が立つまではPID設定を送る
        if (current_pid_mask_ != 0xFFFF) {
            static int retry_count = 0;
            retry_count++;
            
            // if (retry_count % 100 == 0) { // 1秒に1回ログ
            //     RCLCPP_WARN(this->get_logger(), "Handshaking... Mask: %04X", current_pid_mask_);
            // }

            for (int i = 0; i < 16; i++) {
                if (!((current_pid_mask_ >> i) & 1)) {
                    send_pid_config(i);

                    break;
                }
            }
            return; // 設定完了まで駆動コマンドは送らない
        }

        // 通常駆動
        send_drive_command();
    }

    // -----------------------------------------------------------------
    // シリアル受信処理
    // -----------------------------------------------------------------
// -----------------------------------------------------------------
    // シリアル受信処理（修正版）
    // -----------------------------------------------------------------
    void process_serial_read() {
        uint8_t tmp_buf[256];
        int n = read(serial_fd_, tmp_buf, sizeof(tmp_buf));

        if (n > 0) {
            // 正常にデータを受信
            rx_buf_.insert(rx_buf_.end(), tmp_buf, tmp_buf + n);
        } 
        else if (n == 0) {
            // ★追加: USBケーブルが物理的に抜かれた (EOF検知)
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: USB disconnected! Shutting down node...");
            rclcpp::shutdown(); // ROS 2ノードを終了
            return;
        } 
        else if (n < 0) {
            // ★追加: エラー検知
            // O_NDELAY を設定しているため、単に「まだデータが来ていない」時は EAGAIN が返る。これは正常。
            // それ以外の致命的なハードウェアエラー(EIOなど)が起きたら終了する。
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "CRITICAL: Serial read error (%s). Shutting down...", strerror(errno));
                rclcpp::shutdown();
                return;
            }
        }

        while (rx_buf_.size() >= 2) {
            uint16_t header = (rx_buf_[1] << 8) | rx_buf_[0]; // Little Endian

            if (header == 0x5A5A) {
                // [A] フィードバックパケット (184 bytes)
                size_t pkt_size = sizeof(USBFeedbackPacket);
                
                // データが足りない場合は待つ
                if (rx_buf_.size() < pkt_size) break;

                USBFeedbackPacket pkt;
                memcpy(&pkt, rx_buf_.data(), pkt_size);
                
                // チェックサム確認
                if (calc_checksum_fb(&pkt) == pkt.checksum) {
                    // 成功：データを採用し、パケット分を消費する
                    last_feedback_ = pkt;
                    current_pid_mask_ = pkt.pid_configured_mask;
                    publish_feedback();
                    
                    rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + pkt_size);
                } else {
                    // 失敗：ヘッダーだと思ったが間違いだった（または破損）
                    // ★重要：パケット全部ではなく「1バイトだけ」進めて再検索する
                    // RCLCPP_WARN(this->get_logger(), "Checksum Error (FB) - Skipping 1 byte");
                    rx_buf_.erase(rx_buf_.begin());
                }
            } 
            else if (header == 0xCACA) {
                // [B] CAN転送パケット (例: 15 bytes)
                size_t pkt_size = sizeof(USBCanForwardPacket);
                
                if (rx_buf_.size() < pkt_size) break;

                USBCanForwardPacket pkt;
                memcpy(&pkt, rx_buf_.data(), pkt_size);
                
                // CAN転送はチェックサムなし、または同様にチェックするならここに記述
                auto msg = robomas_interfaces::msg::CanFrame();
                msg.id = pkt.can_id;
                msg.dlc = pkt.dlc;
                for(int i=0; i<8; i++) msg.data[i] = pkt.data[i];
                pub_can_rx_->publish(msg);

                // 成功として消費
                rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + pkt_size);
            }
            else {
                // ヘッダー不一致：1バイト読み捨てて次を探す
                rx_buf_.erase(rx_buf_.begin()); 
            }
        }
        
        // バッファ溢れ防止
        if (rx_buf_.size() > 4096) rx_buf_.clear();
    }
    // -----------------------------------------------------------------
    // 送信コマンド作成
    // -----------------------------------------------------------------
    void send_drive_command() {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_DRIVE_ALL; // 0x01
        
        for(int i=0; i<16; i++) {
            pkt.payload.drive[i] = current_targets_[i];
        }
        
        pkt.checksum = calc_checksum_ctrl(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }

    void send_pid_config(int index) {
        if (index < 0 || index >= 16) return;
        
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_SET_PID; // 0x02
        pkt.payload.pid = pid_configs_[index];
        
        pkt.checksum = calc_checksum_ctrl(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }
    
    // -----------------------------------------------------------------
    // チェックサム計算
    // -----------------------------------------------------------------
    uint16_t calc_checksum_ctrl(USBCtrlPacket* pkt) {
        uint16_t sum = 0;
        uint8_t* p = (uint8_t*)pkt;
        // 末尾のchecksum(2byte)を除いて加算
        for(size_t i=0; i < sizeof(USBCtrlPacket) - 2; i++) {
            sum += p[i]; // uint8として足す
        }
        return sum;
    }

    uint16_t calc_checksum_fb(USBFeedbackPacket* pkt) {
        uint16_t sum = 0;
        uint8_t* p = (uint8_t*)pkt;
        // 末尾のchecksum(2byte)を除いて加算
        for(size_t i=0; i < sizeof(USBFeedbackPacket) - 2; i++) {
            sum += p[i]; // uint8として足す
        }
        return sum;
    }

    // -----------------------------------------------------------------
    // 動的パラメータ変更時のコールバック（修正版）
    // -----------------------------------------------------------------
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            std::string name = param.get_name();
            
            // name は "motor0.speed_kp" のような形式
            if (name.find("motor") == 0) {
                size_t dot_pos = name.find('.');
                if (dot_pos != std::string::npos) {
                    try {
                        int idx = std::stoi(name.substr(5, dot_pos - 5)); // "motor"の後の数字
                        std::string p_name = name.substr(dot_pos + 1);    // "."の後の文字列

                        if (idx >= 0 && idx < 16) {
                            // 1. パラメータを構造体に上書き
                            bool is_updated = false; // 更新があったかどうかのフラグ

                            if (p_name == "speed_kp") { pid_configs_[idx].speed_kp = param.as_double(); is_updated = true; }
                            else if (p_name == "speed_ki") { pid_configs_[idx].speed_ki = param.as_double(); is_updated = true; }
                            else if (p_name == "speed_kd") { pid_configs_[idx].speed_kd = param.as_double(); is_updated = true; }
                            else if (p_name == "speed_i_limit") { pid_configs_[idx].speed_i_limit = param.as_double(); is_updated = true; }
                            else if (p_name == "speed_limit") { pid_configs_[idx].speed_output_limit = param.as_double(); is_updated = true; }
                            else if (p_name == "pos_kp") { pid_configs_[idx].position_kp = param.as_double(); is_updated = true; }
                            else if (p_name == "pos_ki") { pid_configs_[idx].position_ki = param.as_double(); is_updated = true; }
                            else if (p_name == "pos_kd") { pid_configs_[idx].position_kd = param.as_double(); is_updated = true; }
                            else if (p_name == "pos_i_limit") { pid_configs_[idx].position_i_limit = param.as_double(); is_updated = true; }
                            else if (p_name == "pos_limit") { pid_configs_[idx].position_output_limit = param.as_double(); is_updated = true; }

                            // 2. ★修正ポイント！
                            // マスク操作をやめて、ここで直接送信関数を呼ぶ！
                            if (is_updated) {
                                send_pid_config(idx);
                                RCLCPP_INFO(this->get_logger(), "Updated & Sent PID for Motor %d", idx + 1);
                            }
                        }
                    } catch (...) {
                        // 変換エラー等は無視
                    }
                }
            }
        }
        return result;
    }

    // -----------------------------------------------------------------
    // ROS コールバック & Publish
    // -----------------------------------------------------------------
    void cmd_callback(const robomas_interfaces::msg::RobomasPacket::SharedPtr msg) {
        for (const auto& cmd : msg->motors) {
            if (cmd.motor_id >= 1 && cmd.motor_id <= 16) {
                int index = cmd.motor_id - 1; 
                current_targets_[index].mode = cmd.mode;
                current_targets_[index].target = cmd.target;
            }
        }
    }

    void can_callback(const robomas_interfaces::msg::CanFrame::SharedPtr msg) {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_SEND_CAN; // 0x03
        
        pkt.payload.can_tx.can_id = msg->id;
        pkt.payload.can_tx.dlc = msg->dlc;
        memcpy(pkt.payload.can_tx.data, msg->data.data(), 8);
        
        pkt.checksum = calc_checksum_ctrl(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }
    
    void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        
        if (msg->data) {
            pkt.command_id = CMD_EMERGENCY_STOP; // 0x00
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP SENT");
        } else {
            pkt.command_id = CMD_RESET_EMERGENCY; // 0x04 (定義あり！)
            RCLCPP_INFO(this->get_logger(), "EMERGENCY RESET SENT");
        }
        
        pkt.checksum = calc_checksum_ctrl(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }

    void publish_feedback() {
        auto msg = robomas_interfaces::msg::RobomasFrame();
        msg.header.stamp = this->now();
        
        msg.system_state = last_feedback_.system_state;       // 定義あり
        msg.pid_configured_mask = last_feedback_.pid_configured_mask; // 定義あり
        
        for(int i=0; i<16; i++) {
            msg.angle[i]    = last_feedback_.motors[i].angle;
            msg.velocity[i] = last_feedback_.motors[i].velocity;
            msg.torque[i]   = (float)last_feedback_.motors[i].torque; 
            msg.temp[i]     = (float)last_feedback_.motors[i].temp;   
        }
        pub_feedback_->publish(msg);
    }

    void display_loop() {
        printf("\033[H\033[J");
        printf("=== Robomas Controller (State: %d) ===\n", last_feedback_.system_state);
        printf("PID Mask: %04X (Target: FFFF)\n", current_pid_mask_);
        printf("ID | Mode | Target |  FB Vel  |  FB Pos  | Torque | Temp \n");
        printf("---|------|--------|----------|----------|--------|------\n");
        for(int i=0; i<16; i++) {
             printf("%2d |  %d   | %6.1f | %8.1f | %8.1f | %6d | %3d \n", 
                i+1, 
                current_targets_[i].mode, 
                current_targets_[i].target,
                last_feedback_.motors[i].velocity, 
                last_feedback_.motors[i].angle,
                last_feedback_.motors[i].torque,
                last_feedback_.motors[i].temp
            );
        }

        fflush(stdout);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobomasBridge>());
    rclcpp::shutdown();
    return 0;
}