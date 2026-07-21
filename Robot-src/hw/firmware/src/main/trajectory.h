#pragma once

#include "protocol.h"

// ============================================================
// Initialization
// ============================================================

bool trajectory_init();

// ============================================================
// Control
// ============================================================

bool trajectory_start(
    const TrajectoryCommand& trajectory);

void trajectory_stop();

void trajectory_update();

// ============================================================
// Status
// ============================================================

bool trajectory_is_active();

bool trajectory_is_complete();

uint8_t trajectory_get_progress();

uint16_t trajectory_get_id();

// ============================================================
// Callback
// ============================================================

void trajectory_set_complete_callback(
    TrajectoryCompleteCallback callback);