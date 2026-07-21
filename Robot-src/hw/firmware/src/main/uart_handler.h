#pragma once

#include "protocol.h"

// ============================================================
// Initialization
// ============================================================

bool uart_handler_init(
    Stream& serial_port);

// ============================================================
// Update
// ============================================================

void uart_handler_update();

// ============================================================
// Command Callback
// ============================================================

void uart_set_command_callback(
    CommandCallback callback);

// ============================================================
// TX Functions
// ============================================================

void uart_send_ack(
    uint8_t command,
    bool success,
    ErrorCode error);

void uart_send_status(
    const StatusPacket& status);

void uart_send_trajectory_complete(
    uint16_t trajectory_id);

void uart_send_encoder_data(
    const EncoderSnapshot& snapshot);

// ============================================================
// Utilities
// ============================================================

uint8_t uart_calculate_crc8(
    const uint8_t* data,
    size_t length);