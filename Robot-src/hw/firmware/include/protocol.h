#pragma once

#include <Arduino.h>
#include <math.h>

// ============================================================
// Debug
// ============================================================

#include <HardwareSerial.h>

extern HardwareSerial DebugSerial;

// ============================================================
// Protocol Bytes
// ============================================================

#define START_BYTE 0xAA
#define END_BYTE   0x55

// ============================================================
// UART
// ============================================================

#define BAUD_RATE 115200

// ============================================================
// Trajectory Limits
// ============================================================

#define MAX_TRAJECTORY_POINTS 32
#define MAX_GRIP_ACTIONS      8

// ============================================================
// Motor Configuration
// ============================================================

#define NUM_MOTORS        3

#define STEPS_PER_REV     200
#define MICROSTEPS        16

#define MAX_SPEED         200
#define MAX_ACCEL         4000

#define HOMING_SPEED      500
#define HOMING_ACCEL      100

// ============================================================
// Encoder
// ============================================================

#define ENCODER_CPR         4096
#define ENCODER_TO_DEGREES  (360.0f / ENCODER_CPR)

// ============================================================
// Status Flags
// ============================================================

#define FLAG_MOTORS_ENABLED    0x01
#define FLAG_VACUUM_ON         0x02
#define FLAG_TRAJECTORY_ACTIVE 0x04
#define FLAG_SERVO_ACTIVE      0x08
#define FLAG_ENCODER_VALID     0x10

// ============================================================
// Robot State
// ============================================================

enum RobotState
{
    STATE_IDLE = 0,
    STATE_HOMING = 1,
    STATE_RUNNING = 2,
    STATE_ERROR = 3,
    STATE_EMERGENCY_STOP = 4,
};

// ============================================================
// Utility
// ============================================================

static inline float normalize_angle(
    float degrees)
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

static inline int32_t degrees_to_steps(
    float degrees)
{
    float steps_per_degree =
        (STEPS_PER_REV * MICROSTEPS) /
        360.0f;

    return lroundf(
        degrees * steps_per_degree);
}

static inline float steps_to_degrees(
    int32_t steps)
{
    float steps_per_degree =
        (STEPS_PER_REV * MICROSTEPS) /
        360.0f;

    return (float)steps /
           steps_per_degree;
}

static inline float encoder_to_degrees(
    uint16_t raw)
{
    return (float)raw *
           ENCODER_TO_DEGREES;
}

static inline uint16_t degrees_to_encoder(
    float degrees)
{
    while (degrees < 0)
    {
        degrees += 360.0f;
    }

    while (degrees >= 360.0f)
    {
        degrees -= 360.0f;
    }

    return (uint16_t)(
        degrees /
        ENCODER_TO_DEGREES);
}