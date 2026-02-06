#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 1. コマンドID定義 (PC -> マイコン)
// ============================================================================
enum CommandID {
    CMD_EMERGENCY_STOP  = 0x00, // 緊急停止 (ペイロードなし/無視)
    CMD_DRIVE_ALL       = 0x01, // 16台一括制御 (MotorUnit[16])
    CMD_SET_PID         = 0x02, // PIDゲイン設定 (PIDConfig)
    CMD_SEND_CAN        = 0x03, // 任意CAN送信 (CantTxRequest)
    CMD_RESET_EMERGENCY = 0x04, // 緊急停止解除 (ペイロードなし)
    CMD_SAVE_SETTINGS   = 0x05  // [Option] Flash保存 (ペイロードなし)
};


// ============================================================================
// 2. ペイロード構造体定義 (共用体の中身)
// ============================================================================

// [A] 全モーター駆動用 (頻繁に送る: 1msごと)
// ModeとTargetは必ずペアで扱います。
typedef struct {
    uint8_t mode;       // 0:電流, 1:速度, 2:位置, 3:無効(脱力)
    float   target;     // 目標値 (A, rpm, degree)
} __attribute__((packed)) MotorUnit;

// [B] PIDゲイン設定用 (調整時に送る)
// データ量が多いため、1パケットで「1台分」の設定を送ります。
typedef struct {
    uint8_t motor_id;   // 設定するモーター番号 (0-15)

    // 速度制御用
    float   speed_kp;
    float   speed_ki;
    float   speed_kd;
    float   speed_i_limit;
    float   speed_output_limit;  // 出力電流最大値

    // 位置制御用
    float   position_kp;
    float   position_ki;
    float   position_kd;
    float   position_i_limit;
    float   position_output_limit;  // 出力速度最大値
} __attribute__((packed)) PIDConfig;

// [C] 任意のCANフレーム送信要求 (PCから任意のデータを送りたい時)
typedef struct {
    uint32_t can_id;    // CAN ID (標準/拡張はIDの大きさで自動判別推奨)
    uint8_t  dlc;       // データ長 (0-8)
    uint8_t  data[8];   // データ本体
} __attribute__((packed)) CantTxRequest;


// ============================================================================
// 3. USB受信パケット (PC -> マイコン)
// ============================================================================
// 全体サイズ: 2(Head) + 1(Cmd) + 80(Payload) + 2(Sum) = 85 bytes
typedef struct {
    // [ヘッダー部]
    uint16_t header;      // 0xA5A5 (パケット開始識別)
    uint8_t  command_id;  // CommandID参照

    // [ペイロード部 (共用体)]
    // メモリ領域を共有します。最大サイズ(drive: 80byte)に合わせて確保されます。
    // ※データが短いコマンドを送る際も、必ずパディングして85byteで送ること。
    union {
        uint8_t       dummy[80];    // 領域確保 & パディング用
        MotorUnit     drive[16];    // ID: 0x01
        PIDConfig     pid;          // ID: 0x02
        CantTxRequest can_tx;       // ID: 0x03
    } payload;

    // [フッター部]
    uint16_t checksum;    // 単純加算チェックサム (header ～ payload末尾まで)

} __attribute__((packed)) USBCtrlPacket;



///////////////////////////////// マイコン → PC ///////////////////////////////////


// ============================================================================
// 4. フィードバックデータ構造
// ============================================================================

// ロボマス1台分のフィードバック情報 (11 byte)
typedef struct {
    float   angle;        // 累積角度 (度数法)
    float   velocity;     // 現在速度 (rpm)
    int16_t torque;       // 現在トルク電流 (生値)
    uint8_t temp;         // 温度 (℃)
} __attribute__((packed)) MotorFeedbackUnit;


// [パケットA] ロボマス状態 (定期送信: 1ms〜10msごと)
// ヘッダー: 0x5A5A
typedef struct {
    uint16_t header;       // 0x5A5A

    // システム状態 (0:非常停止, 1:準備, 2:駆動)
    uint8_t  system_state;

    // PID設定完了確認用ビットマスク
    // bit0=1ならMotor0設定済み, bit15=1ならMotor15設定済み
    // 全ビットが1(0xFFFF)になるまで、PCはPID設定を再送する
    uint16_t pid_configured_mask;

    uint8_t  reserved;     // アライメント調整/予備

    // 16台分のフィードバック
    MotorFeedbackUnit motors[16];

    uint16_t checksum;     // チェックサム
} __attribute__((packed)) USBFeedbackPacket;


// [パケットB] 任意CAN転送 (イベント送信: 受信時のみ)
// ヘッダー: 0xCACA
typedef struct {
    uint16_t header;      // 0xCACA (Can Can)
    uint32_t can_id;      // CAN ID
    uint8_t  dlc;         // データ長
    uint8_t  data[8];     // データ本体
    uint16_t checksum;
} __attribute__((packed)) USBCanForwardPacket;


// ============================================================================
// 5. 関数プロトタイプ宣言
// ============================================================================
void Prepare_Motor_Packet(USBFeedbackPacket* pkt);


#ifdef __cplusplus
}
#endif
