#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ★★★ 最重要：強制的に1バイト境界にする呪文 (開始) ★★★
#pragma pack(push, 1)

// ============================================================================
// 1. コマンドID定義
// ============================================================================
enum CommandID {
    CMD_EMERGENCY_STOP  = 0x00,
    CMD_DRIVE_ALL       = 0x01,
    CMD_SET_PID         = 0x02,
    CMD_SEND_CAN        = 0x03,
    CMD_RESET_EMERGENCY = 0x04,
    CMD_SAVE_SETTINGS   = 0x05
};

// ============================================================================
// 2. ペイロード構造体
// ============================================================================
typedef struct {
    uint8_t mode;
    float   target;
} MotorUnit; // 5 bytes

typedef struct {
    uint8_t motor_id;
    float   speed_kp;
    float   speed_ki;
    float   speed_kd;
    float   speed_i_limit;
    float   speed_output_limit;
    float   position_kp;
    float   position_ki;
    float   position_kd;
    float   position_i_limit;
    float   position_output_limit;
} PIDConfig; // 41 bytes

typedef struct {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CantTxRequest; // 13 bytes

// ============================================================================
// 3. USB受信パケット (PC -> マイコン)
// ============================================================================
typedef struct {
    uint16_t header;     // 0xA5A5
    uint8_t  command_id;
    union {
        uint8_t       dummy[80];
        MotorUnit     drive[16];
        PIDConfig     pid;
        CantTxRequest can_tx;
    } payload;
    uint16_t checksum;
} USBCtrlPacket; // Total: 85 bytes

// ============================================================================
// 4. フィードバックデータ構造 (マイコン -> PC)
// ============================================================================
typedef struct {
    float   angle;
    float   velocity;
    int16_t torque;
    uint8_t temp;
} MotorFeedbackUnit; // 11 bytes (これがズレの原因でした！)

typedef struct {
    uint16_t header;              // 0x5A5A
    uint8_t  system_state;
    uint16_t pid_configured_mask;
    uint8_t  reserved;
    MotorFeedbackUnit motors[16]; // 11 * 16 = 176 bytes
    uint16_t checksum;
} USBFeedbackPacket; // Total: 184 bytes

typedef struct {
    uint16_t header; // 0xCACA
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint16_t checksum;
} USBCanForwardPacket;

// ★★★ 呪文の解除 (終了) ★★★
#pragma pack(pop)

#ifdef __cplusplus
}
#endif