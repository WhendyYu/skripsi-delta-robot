#include <sys/_stdint.h>
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

// ============================================================
// Debug Serial
// ============================================================

#define DebugSerial Serial

// ============================================================
// Global Configuration
// ============================================================

constexpr uint8_t NUM_MOTORS = 3;

// ============================================================
// UART
// ============================================================

constexpr uint32_t UART_BAUDRATE = 115200;

constexpr uint16_t UART_BUFFER_SIZE = 512;
constexpr uint16_t UART_PAYLOAD_MAX = 480;

// ============================================================
// Protocol Markers
// ============================================================

constexpr uint8_t START_BYTE = 0xAA;
constexpr uint8_t END_BYTE   = 0x55;

// ============================================================
// Encoder
// ============================================================

constexpr uint16_t ENCODER_CPR = 4096;

constexpr float ENCODER_TO_DEG = 360.0f / (float)ENCODER_CPR;

// ============================================================
// Stepper Configuration
// ============================================================

constexpr uint16_t STEPS_PER_REV = 200;

constexpr uint8_t MICROSTEPS = 16;

constexpr float STEPS_PER_DEG = (STEPS_PER_REV * MICROSTEPS) / 360.0f;

// ============================================================
// Trajectory Limits
// ============================================================

constexpr uint8_t MAX_TRAJECTORY_POINTS = 16;

// ============================================================
// Motion Limits
// ============================================================

constexpr float MAX_JOINT_VEL = 180.0f; // deg/sec

constexpr float MAX_JOINT_ACCEL = 720.0f; // deg/sec²

constexpr float JOINT_MIN_DEG = -65.0f;

constexpr float JOINT_MAX_DEG = 65.0f;

// ============================================================
// Control Loop
// ============================================================

constexpr uint16_t TRAJECTORY_CONTROL_HZ = 1000;

// ============================================================
// Robot State
// ============================================================

enum class RobotState : uint8_t
{
    IDLE            = 0,
    HOMING          = 1,
    RUNNING         = 2,
    ERROR           = 3,
    EMERGENCY_STOP  = 4
};

// ============================================================
// Command Types
// ============================================================

enum class CommandType : uint8_t
{
    HOME            = 0x01,
    EMERGENCY_STOP  = 0x02,
    ENABLE_MOTORS   = 0x03,
    DISABLE_MOTORS  = 0x04,
    SET_VACUUM      = 0x05,
    SET_SERVO       = 0x06,
    TRAJECTORY      = 0x10,
    GET_STATUS      = 0x20,
    GET_ENCODERS    = 0x21,
    INIT            = 0x22
};

// ============================================================
// Response Types
// ============================================================

enum class ResponseType : uint8_t
{
    ACK                  = 0x80,
    STATUS               = 0x90,
    TRAJECTORY_COMPLETE  = 0x92,
    ENCODER_DATA         = 0x93
};

// ============================================================
// Error Codes
// ============================================================

enum class ErrorCode : uint8_t
{
    NONE                  = 0,
    INVALID_COMMAND       = 1,
    INVALID_PAYLOAD       = 2,
    TRAJECTORY_TOO_LONG   = 3,
    MOTOR_FAULT           = 4,
    HOME_FAILED           = 5,
    CRC_MISMATCH          = 6,
    UART_TIMEOUT          = 7,
    ENCODER_READ          = 8,
    I2C_ERROR             = 9
};

// ============================================================
// Status Flags
// ============================================================

constexpr uint8_t FLAG_MOTORS_ENABLED = 0x01;

constexpr uint8_t FLAG_VACUUM_ON = 0x02;

constexpr uint8_t FLAG_TRAJECTORY_ACTIVE = 0x04;

constexpr uint8_t FLAG_ENCODER_VALID = 0x08;

constexpr uint8_t FLAG_SYNCHRONIZED =  0x10;

// ============================================================
// Encoder Snapshot
// ============================================================

struct EncoderSnapshot
{
    uint16_t raw[NUM_MOTORS];

    float degrees[NUM_MOTORS];

    bool valid[NUM_MOTORS];

    uint32_t timestamp_ms;
};

// ============================================================
// Trajectory Point
// ============================================================

struct TrajectoryPoint
{
    float joint_angles[NUM_MOTORS];

    uint32_t time_ms;

    uint8_t vacuum;

    uint8_t servo_angle;

    uint16_t reserved;
};

// ============================================================
// Trajectory Command
// ============================================================

struct TrajectoryCommand
{
    uint16_t trajectory_id;

    uint16_t num_points;

    TrajectoryPoint points[MAX_TRAJECTORY_POINTS];
};

// ============================================================
// Runtime Segment
// ============================================================

struct TrajectorySegment
{
    float start_deg[NUM_MOTORS];

    float end_deg[NUM_MOTORS];

    float duration_ms;

    uint32_t start_time_ms;

    uint32_t end_time_ms;
};

// ============================================================
// UART Command Packet
// ============================================================

struct CommandPacket
{
    uint8_t command;

    uint16_t length;

    uint8_t payload[UART_PAYLOAD_MAX];
};

// ============================================================
// Status Packet
// ============================================================

struct StatusPacket
{
    RobotState state;

    ErrorCode error_code;

    uint8_t error_motor;

    float current_angles[NUM_MOTORS];

    int32_t motor_steps[NUM_MOTORS];

    uint16_t encoder_raw[NUM_MOTORS];

    uint8_t flags;

    uint16_t trajectory_id;

    uint8_t trajectory_progress;
};

// ============================================================
// UART RX Parser State
// ============================================================

enum class RxState : uint8_t
{
    WAIT_START,
    WAIT_LENGTH,
    WAIT_COMMAND,
    WAIT_PAYLOAD,
    WAIT_CRC,
    WAIT_END
};

// ============================================================
// Callback Types
// ============================================================

typedef void (*CommandCallback)(const CommandPacket& packet);

typedef void (*TrajectoryCompleteCallback)(uint16_t trajectory_id);

// ============================================================
// Utility Functions
// ============================================================

static inline float normalize_angle(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }

    while (degrees <= -180.0f)
    {
        degrees += 360.0f;
    }

    return degrees;
}

static inline int32_t degrees_to_steps(float degrees)
{
    return lroundf(
        degrees * STEPS_PER_DEG);
}

static inline float steps_to_degrees(int32_t steps)
{
    return (float)steps / STEPS_PER_DEG;
}

static inline float encoder_to_degrees(uint16_t raw)
{
    return (float)raw * ENCODER_TO_DEG;
}

static inline uint16_t degrees_to_encoder(float degrees)
{
    while (degrees < 0.0f)
    {
        degrees += 360.0f;
    }

    while (degrees >= 360.0f)
    {
        degrees -= 360.0f;
    }

    return (uint16_t)(degrees / ENCODER_TO_DEG);
}