#include "i2c_handler.h"

#include <Wire.h>

// ============================================================
// Hardware Configuration
// ============================================================

constexpr uint8_t I2C_SDA_PIN = 8;

constexpr uint8_t I2C_SCL_PIN = 9;

constexpr uint32_t I2C_FREQUENCY =
    100000;

// ============================================================
// TCA9548A
// ============================================================

constexpr uint8_t MUX_ADDRESS =
    0x70;

// ============================================================
// AS5600
// ============================================================

constexpr uint8_t AS5600_ADDRESS =
    0x36;

constexpr uint8_t AS5600_RAW_ANGLE =
    0x0C;

// ============================================================
// Encoder Channel Mapping
// ============================================================

static const uint8_t
encoder_channel_map[NUM_MOTORS] =
{
    0,
    1,
    2
};

// ============================================================
// Runtime State
// ============================================================

static int16_t
encoder_offset[NUM_MOTORS] =
{
    4048,
    2925,
    3566,
};

// ============================================================
// Internal
// ============================================================

static bool i2c_select_mux_channel(
    uint8_t channel);

static bool i2c_read_as5600_raw(
    uint8_t channel,
    uint16_t& raw);

// ============================================================
// Initialization
// ============================================================

bool i2c_handler_init()
{
    Wire.begin(
        I2C_SDA_PIN,
        I2C_SCL_PIN);

    Wire.setClock(
        I2C_FREQUENCY);

    DebugSerial.println(
        "I2C initialized");

    return true;
}

// ============================================================
// Read All Encoders
// ============================================================

bool i2c_read_encoders(
    EncoderSnapshot& snapshot)
{
    bool any_valid = false;

    for (uint8_t i = 0;
         i < NUM_MOTORS;
         i++)
    {
        uint16_t raw = 0;

        bool success =
            i2c_read_encoder(
                i,
                raw);

        snapshot.raw[i] = raw;

        snapshot.valid[i] =
            success;

        if (!success)
        {
            snapshot.degrees[i] =
                0.0f;

            continue;
        }

        any_valid = true;

        // ====================================================
        // Simple Single-Turn Angle
        // ====================================================

        float degrees =
            (float)raw
            * ENCODER_TO_DEG;

        float offset_deg =
            encoder_offset[i]
            * ENCODER_TO_DEG;

        degrees -= offset_deg;

        // ====================================================
        // Normalize
        // ====================================================

        while (degrees > 180.0f)
        {
            degrees -= 360.0f;
        }

        while (degrees <= -180.0f)
        {
            degrees += 360.0f;
        }

        snapshot.degrees[i] =
            degrees;
    }

    snapshot.timestamp_ms =
        millis();

    return any_valid;
}

// ============================================================
// Read Single Encoder
// ============================================================

bool i2c_read_encoder(
    uint8_t encoder_index,
    uint16_t& raw_angle)
{
    if (encoder_index >=
        NUM_MOTORS)
    {
        return false;
    }

    return i2c_read_as5600_raw(
        encoder_channel_map[
            encoder_index],
        raw_angle);
}

// ============================================================
// Offset Access
// ============================================================

void i2c_set_encoder_offset(
    uint8_t encoder,
    uint16_t offset)
{
    if (encoder >= NUM_MOTORS)
    {
        return;
    }

    encoder_offset[encoder] =
        offset;
}

int16_t i2c_get_encoder_offset(
    uint8_t encoder)
{
    if (encoder >= NUM_MOTORS)
    {
        return 0;
    }

    return encoder_offset[
        encoder];
}

// ============================================================
// Select MUX Channel
// ============================================================

static bool i2c_select_mux_channel(
    uint8_t channel)
{
    Wire.beginTransmission(
        MUX_ADDRESS);

    Wire.write(
        1 << channel);

    if (Wire.endTransmission()
        != 0)
    {
        return false;
    }

    delayMicroseconds(50);

    return true;
}

// ============================================================
// Read AS5600 Raw
// ============================================================

static bool i2c_read_as5600_raw(
    uint8_t channel,
    uint16_t& raw)
{
    if (!i2c_select_mux_channel(
            channel))
    {
        return false;
    }

    Wire.beginTransmission(
        AS5600_ADDRESS);

    Wire.write(
        AS5600_RAW_ANGLE);

    if (Wire.endTransmission(false)
        != 0)
    {
        return false;
    }

    uint8_t bytes_received =
        Wire.requestFrom(
            AS5600_ADDRESS,
            (uint8_t)2);

    if (bytes_received != 2)
    {
        return false;
    }

    raw =
        ((uint16_t)Wire.read()
         << 8)
        |
        Wire.read();

    raw &= 0x0FFF;

    return true;
}