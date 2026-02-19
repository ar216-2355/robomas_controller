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
#include <cerrno>

#include "std_msgs/msg/bool.hpp"
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/robomas_frame.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"

// ヘッダーファイルの内容が入っている前提
#include "usb_packet.hpp" 

using namespace std::chrono_literals;

class RobomasBridge : public rclcpp::Node {
public:
    RobomasBridge() : Node("robomas_node") {
        RCLCPP_INFO(this->get_logger(), "Sizeof FB Packet: %lu", sizeof(USBFeedbackPacket));

        // --- カウンタの初期化 ---
        packets_received_ = 0;
        checksum_errors_ = 0;
        last_packet_time_ = this->now();
        
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

        RCLCPP_INFO(this->get_logger(), "Robomas Controller Started (Full Spec / M3508 Scaled).");
    }

private:
    // --- 追加のメンバ変数 ---
    uint64_t packets_received_;   // 正常に受信できた数
    uint64_t checksum_errors_;    // チェックサムで弾かれた数
    rclcpp::Time last_packet_time_; // 最後にパケットを受信した時刻

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

        // 入力フラグ (Input Flags)
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        // 出力フラグ (Output Flags)
        options.c_oflag &= ~OPOST;
        // ローカルフラグ (Local Flags)
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        // 制御フラグ (Control Flags)
        options.c_cflag &= ~(CSIZE | PARENB);
        options.c_cflag |= CS8;
        
        // ボーレート設定
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

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
            cfg.motor_id = i + 1;
            
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

        // マイコンがEMERGENCYモード(0)の時は、PC側の目標値も強制的にリセット
        if (last_feedback_.system_state == 0) {
            for (int i = 0; i < 16; i++) {
                current_targets_[i].mode = 3;     // 無効モード(脱力)
                current_targets_[i].target = 0.0; // 目標値0
            }
        }

        // ハンドシェイク
        if (current_pid_mask_ != 0xFFFF) {
            static int retry_count = 0;
            retry_count++;
            
            for (int i = 0; i < 16; i++) {
                if (!((current_pid_mask_ >> i) & 1)) {
                    send_pid_config(i);
                    break;
                }
            }
            return;
        }

        // 通常駆動
        send_drive_command();
    }

    // -----------------------------------------------------------------
    // シリアル受信処理
    // -----------------------------------------------------------------
    void process_serial_read() {
        uint8_t tmp_buf[256];
        int n = read(serial_fd_, tmp_buf, sizeof(tmp_buf));

        if (n > 0) {
            rx_buf_.insert(rx_buf_.end(), tmp_buf, tmp_buf + n);
        } 
        else if (n == 0) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: USB disconnected! Shutting down node...");
            rclcpp::shutdown();
            return;
        } 
        else if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "CRITICAL: Serial read error (%s). Shutting down...", strerror(errno));
                rclcpp::shutdown();
                return;
            }
        }

        while (rx_buf_.size() >= 2) {
            uint16_t header = (rx_buf_[1] << 8) | rx_buf_[0];

            if (header == 0x5A5A) {
                // [A] フィードバックパケット
                size_t pkt_size = sizeof(USBFeedbackPacket);
                if (rx_buf_.size() < pkt_size) break;

                USBFeedbackPacket pkt;
                memcpy(&pkt, rx_buf_.data(), pkt_size);
                
                if (calc_checksum_fb(&pkt) == pkt.checksum) {
                    last_feedback_ = pkt;
                    current_pid_mask_ = pkt.pid_configured_mask;
                    publish_feedback();
                    
                    // 成功カウントと時刻更新
                    packets_received_++;
                    last_packet_time_ = this->now();
                    
                    rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + pkt_size);
                } else {
                    // チェックサムエラー
                    checksum_errors_++;
                    rx_buf_.erase(rx_buf_.begin());
                }
            } 
            else if (header == 0xCACA) {
                // [B] CAN転送パケット
                size_t pkt_size = sizeof(USBCanForwardPacket);
                if (rx_buf_.size() < pkt_size) break;

                USBCanForwardPacket pkt;
                memcpy(&pkt, rx_buf_.data(), pkt_size);
                
                auto msg = robomas_interfaces::msg::CanFrame();
                msg.id = pkt.can_id;
                msg.dlc = pkt.dlc;
                for(int i=0; i<8; i++) msg.data[i] = pkt.data[i];
                pub_can_rx_->publish(msg);

                rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + pkt_size);
            }
            else {
                rx_buf_.erase(rx_buf_.begin()); 
            }
        }
        if (rx_buf_.size() > 4096) rx_buf_.clear();
    }

    // -----------------------------------------------------------------
    // 送信コマンド作成（★M3508スケール変換の実装箇所）
    // -----------------------------------------------------------------
    void send_drive_command() {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_DRIVE_ALL; // 0x01
        
        for(int i=0; i<16; i++) {
            // 基本はROSからの指令値をそのままコピー
            pkt.payload.drive[i] = current_targets_[i];

            // ★電流制御モード(0)の場合、温度を見てM3508ならスケール変換する
            // M2006 (temp == 0): 10000mA -> 10000 (変換なし)
            // M3508 (temp != 0): 20000mA -> 16384 (変換あり)
            if (current_targets_[i].mode == 0) {
                // ROSから来るのは mA単位 (例: 20000.0)
                float ros_target_ma = current_targets_[i].target;
                uint8_t temp = last_feedback_.motors[i].temp;

                if (temp != 0) {
                    // M3508の場合
                    // 20000mA を 16384 に変換 (係数: 0.8192)
                    pkt.payload.drive[i].target = ros_target_ma * (16384.0f / 20000.0f);
                } 
                // M2006 (temp==0) の場合は 10000mA = 10000 なのでそのまま
            }
        }
        
        pkt.checksum = calc_checksum_ctrl(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }

    void send_pid_config(int index) {
        if (index < 0 || index >= 16) return;
        
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_SET_PID;
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
        for(size_t i=0; i < sizeof(USBCtrlPacket) - 2; i++) {
            sum += p[i];
        }
        return sum;
    }

    uint16_t calc_checksum_fb(USBFeedbackPacket* pkt) {
        uint16_t sum = 0;
        uint8_t* p = (uint8_t*)pkt;
        for(size_t i=0; i < sizeof(USBFeedbackPacket) - 2; i++) {
            sum += p[i];
        }
        return sum;
    }

    // -----------------------------------------------------------------
    // 動的パラメータ変更時のコールバック
    // -----------------------------------------------------------------
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            std::string name = param.get_name();
            if (name.find("motor") == 0) {
                size_t dot_pos = name.find('.');
                if (dot_pos != std::string::npos) {
                    try {
                        int idx = std::stoi(name.substr(5, dot_pos - 5));
                        std::string p_name = name.substr(dot_pos + 1);

                        if (idx >= 0 && idx < 16) {
                            bool is_updated = false;

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

                            if (is_updated) {
                                send_pid_config(idx);
                                RCLCPP_INFO(this->get_logger(), "Updated & Sent PID for Motor %d", idx + 1);
                            }
                        }
                    } catch (...) {}
                }
            }
        }
        return result;
    }

    // -----------------------------------------------------------------
    // ROS コールバック
    // -----------------------------------------------------------------
    void cmd_callback(const robomas_interfaces::msg::RobomasPacket::SharedPtr msg) {
        for (const auto& cmd : msg->motors) {
            if (cmd.motor_id >= 1 && cmd.motor_id <= 16) {
                int index = cmd.motor_id - 1; 
                current_targets_[index].mode = cmd.mode;
                current_targets_[index].target = cmd.target; // ここにはmA (20000.0) が入る
            }
        }
    }

    void can_callback(const robomas_interfaces::msg::CanFrame::SharedPtr msg) {
        USBCtrlPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.header = 0xA5A5;
        pkt.command_id = CMD_SEND_CAN;
        
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
            pkt.command_id = CMD_EMERGENCY_STOP;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP SENT");
        } else {
            pkt.command_id = CMD_RESET_EMERGENCY;
            RCLCPP_INFO(this->get_logger(), "EMERGENCY RESET SENT");
        }
        pkt.checksum = calc_checksum_ctrl(&pkt);
        write(serial_fd_, &pkt, sizeof(pkt));
    }

    // -----------------------------------------------------------------
    // Publish Feedback（★M3508スケール変換の実装箇所）
    // -----------------------------------------------------------------
    void publish_feedback() {
        auto msg = robomas_interfaces::msg::RobomasFrame();
        msg.header.stamp = this->now();
        
        msg.system_state = last_feedback_.system_state;
        msg.pid_configured_mask = last_feedback_.pid_configured_mask;
        
        for(int i=0; i<16; i++) {
            msg.angle[i]    = last_feedback_.motors[i].angle;
            msg.velocity[i] = last_feedback_.motors[i].velocity;
            msg.temp[i]     = (float)last_feedback_.motors[i].temp;

            // ★トルク値の変換
            // M2006 (temp == 0): 生値 (±10000) = mA
            // M3508 (temp != 0): 生値 (±16384) -> ±20000 mA に変換
            float raw_torque = (float)last_feedback_.motors[i].torque;
            uint8_t temp = last_feedback_.motors[i].temp;

            if (temp != 0) {
                // M3508: 16384 を 20000 に引き伸ばす (係数: 1.2207)
                msg.torque[i] = raw_torque * (20000.0f / 16384.0f);
            } else {
                // M2006: そのまま
                msg.torque[i] = raw_torque;
            }
        }
        pub_feedback_->publish(msg);
    }

    // -----------------------------------------------------------------
    // Display Loop（未接続判定ロジックを追加）
    // -----------------------------------------------------------------
    void display_loop() {
        printf("\033[H\033[J");

        // 通信状態の計算
        double dt = (this->now() - last_packet_time_).seconds();
        const char* conn_status = (dt < 0.1) ? "\033[1;32mONLINE\033[0m" : "\033[1;31mOFFLINE (Timeout)\033[0m";
        double error_rate = (packets_received_ + checksum_errors_ > 0) 
            ? (double)checksum_errors_ / (packets_received_ + checksum_errors_) * 100.0 : 0.0;


        printf("=== Robomas Controller Stats ===\n");
        printf("Status: %s  |  Last Packet: %.3f sec ago\n", conn_status, dt);
        printf("Received: %lu  |  Checksum Errors: %lu (\033[1;33m%.2f%%\033[0m)\n", 
                packets_received_, checksum_errors_, error_rate);
        printf("-------------------------------------------------------------\n");
        
        printf("System State: %d | PID Mask: %04X\n", last_feedback_.system_state, current_pid_mask_);
        
        printf("ID | Type  | Mode | Target |  FB Vel  |  FB Pos  | Torque(mA) | Temp \n");
        printf("---|-------|------|--------|----------|----------|------------|------\n");
        
        for(int i=0; i<16; i++) {
            float raw_torque = (float)last_feedback_.motors[i].torque;
            uint8_t temp = last_feedback_.motors[i].temp;
            float velocity = last_feedback_.motors[i].velocity;
            float angle = last_feedback_.motors[i].angle;
            
            float display_torque = raw_torque;
            
            // ★判定ロジック
            const char* model_name;

            // データが全て完全に0なら「未接続」とみなす
            // (M2006でも手で少し回せば angle が0以外になるので認識されます)
            bool is_disconnected = (temp == 0 && raw_torque == 0 && velocity == 0.0f && angle == 0.0f);

            if (temp > 0) {
                model_name = "M3508"; // 温度があれば絶対M3508
                // M3508のみトルクをmA換算
                display_torque = raw_torque * (20000.0f / 16384.0f);
            } 
            else if (!is_disconnected) {
                model_name = "M2006"; // 温度0だけど、何かデータが来てるならM2006
            } 
            else {
                model_name = " --- "; // 全部0なら未接続扱い
            }

            // 表示 (未接続なら数値も薄くしたいところですが、まずは名前だけで)
             printf("%2d | %s |  %d   | %6.1f | %8.1f | %8.1f | %10.0f | %3d \n", 
                i+1, 
                model_name, 
                current_targets_[i].mode, 
                current_targets_[i].target,
                velocity, 
                angle,
                display_torque, 
                temp
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