#pragma once

#include "protocol.h"

// ============================================================
// Initialization
// ============================================================

bool motor_control_init();

// ============================================================
// Enable / Disable
// ============================================================

void motor_control_enable(
    bool enable);

bool motor_control_is_enabled();

// ============================================================
// Velocity Control (legacy/debug only)
// ============================================================

void motor_control_run_forward(
    uint8_t motor,
    float speed_hz);

void motor_control_run_backward(
    uint8_t motor,
    float speed_hz);

void motor_control_set_velocity(
    uint8_t motor,
    float velocity_deg_sec);

void motor_control_stop(
    uint8_t motor);

void motor_control_stop_all();

// ============================================================
// Position Motion (NEW)
// ============================================================

bool motor_control_move_to_steps(
    uint8_t motor,
    int32_t target_steps,
    float speed_deg_sec);

bool motor_control_move_to_degrees(
    uint8_t motor,
    float target_deg,
    float speed_deg_sec);

bool motor_control_is_running(
    uint8_t motor);

bool motor_control_all_complete();

// ============================================================
// Position Access
// ============================================================

int32_t motor_control_get_position_steps(
    uint8_t motor);

float motor_control_get_position_degrees(
    uint8_t motor);

void motor_control_set_position_steps(
    uint8_t motor,
    int32_t steps);

void motor_control_set_position_degrees(
    uint8_t motor,
    float degrees);

// ============================================================
// Outputs
// ============================================================

void motor_control_set_vacuum(
    bool enabled);

void motor_control_set_servo(
    uint8_t position);

// ============================================================
// Emergency Stop
// ============================================================

void motor_control_emergency_stop();