#pragma once

#include "protocol.h"

// ============================================================
// Initialization
// ============================================================

bool i2c_handler_init();

// ============================================================
// Encoder API
// ============================================================

bool i2c_read_encoders(
    EncoderSnapshot& snapshot);

bool i2c_read_encoder(
    uint8_t encoder_index,
    uint16_t& raw_angle);

// ============================================================
// Encoder Offsets
// ============================================================

void i2c_set_encoder_offset(
    uint8_t encoder,
    int16_t offset);

int16_t i2c_get_encoder_offset(
    uint8_t encoder);