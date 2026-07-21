#include "uart_handler.h"

// ============================================================
// UART Stream
// ============================================================

static Stream* uart = nullptr;

// ============================================================
// RX State
// ============================================================

static RxState rx_state =
    RxState::WAIT_START;

static uint8_t rx_length = 0;

static uint8_t rx_command = 0;

static uint16_t rx_index = 0;

static uint8_t rx_crc = 0;

static uint8_t rx_payload[
    UART_PAYLOAD_MAX];

// ============================================================
// Callback
// ============================================================

static CommandCallback
command_callback = nullptr;

// ============================================================
// Internal
// ============================================================

static void uart_reset_parser();

static void uart_parse_byte(
    uint8_t byte);

static void uart_send_packet(
    uint8_t type,
    const uint8_t* payload,
    uint16_t length);

// ============================================================
// Initialization
// ============================================================

bool uart_handler_init(
    Stream& serial_port)
{
    uart = &serial_port;

    uart_reset_parser();

    return true;
}

// ============================================================
// Update
// ============================================================

void uart_handler_update()
{
    if (uart == nullptr)
    {
        return;
    }

    while (uart->available())
    {
        uint8_t byte =
            uart->read();

        uart_parse_byte(byte);
    }
}

// ============================================================
// Callback
// ============================================================

void uart_set_command_callback(
    CommandCallback callback)
{
    command_callback =
        callback;
}

// ============================================================
// ACK
// ============================================================

void uart_send_ack(
    uint8_t command,
    bool success,
    ErrorCode error)
{
    uint8_t payload[3];

    payload[0] = command;

    payload[1] =
        success ? 1 : 0;

    payload[2] =
        static_cast<uint8_t>(
            error);

    uart_send_packet(
        static_cast<uint8_t>(
            ResponseType::ACK),
        payload,
        sizeof(payload));
}

// ============================================================
// Status
// ============================================================

void uart_send_status(
    const StatusPacket& status)
{
    uint8_t payload[64];

    uint16_t offset = 0;

    payload[offset++] =
        static_cast<uint8_t>(
            status.state);

    payload[offset++] =
        static_cast<uint8_t>(
            status.error_code);

    payload[offset++] =
        status.error_motor;

    memcpy(
        &payload[offset],
        status.current_angles,
        sizeof(status.current_angles));

    offset +=
        sizeof(status.current_angles);

    memcpy(
        &payload[offset],
        status.motor_steps,
        sizeof(status.motor_steps));

    offset +=
        sizeof(status.motor_steps);

    memcpy(
        &payload[offset],
        status.encoder_raw,
        sizeof(status.encoder_raw));

    offset +=
        sizeof(status.encoder_raw);

    payload[offset++] =
        status.flags;

    payload[offset++] =
        status.trajectory_id & 0xFF;

    payload[offset++] =
        (status.trajectory_id >> 8)
        & 0xFF;

    payload[offset++] =
        status.trajectory_progress;

    uart_send_packet(
        static_cast<uint8_t>(
            ResponseType::STATUS),
        payload,
        offset);
}

// ============================================================
// Trajectory Complete
// ============================================================

void uart_send_trajectory_complete(
    uint16_t trajectory_id)
{
    uint8_t payload[2];

    payload[0] =
        trajectory_id & 0xFF;

    payload[1] =
        (trajectory_id >> 8)
        & 0xFF;

    uart_send_packet(
        static_cast<uint8_t>(
            ResponseType::TRAJECTORY_COMPLETE),
        payload,
        sizeof(payload));
}

// ============================================================
// Encoder Data
// ============================================================

void uart_send_encoder_data(
    const EncoderSnapshot& snapshot)
{
    uint8_t payload[16];

    uint16_t offset = 0;

    for (uint8_t i = 0;
         i < NUM_MOTORS;
         i++)
    {
        payload[offset++] =
            snapshot.raw[i] & 0xFF;

        payload[offset++] =
            (snapshot.raw[i] >> 8)
            & 0xFF;
    }

    uint8_t valid_flags = 0;

    for (uint8_t i = 0;
         i < NUM_MOTORS;
         i++)
    {
        if (snapshot.valid[i])
        {
            valid_flags |=
                (1 << i);
        }
    }

    payload[offset++] =
        valid_flags;

    uart_send_packet(
        static_cast<uint8_t>(
            ResponseType::ENCODER_DATA),
        payload,
        offset);
}

// ============================================================
// CRC8
// ============================================================

uint8_t uart_calculate_crc8(
    const uint8_t* data,
    size_t length)
{
    uint8_t crc = 0;

    for (size_t i = 0;
         i < length;
         i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0;
             j < 8;
             j++)
        {
            if (crc & 0x80)
            {
                crc =
                    (crc << 1)
                    ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// ============================================================
// Send Packet
// ============================================================

static void uart_send_packet(
    uint8_t type,
    const uint8_t* payload,
    uint16_t length)
{
    if (uart == nullptr)
    {
        return;
    }

    uint8_t packet[
        UART_BUFFER_SIZE];

    uint16_t offset = 0;

    packet[offset++] =
        START_BYTE;

    uint16_t total_length =
        length + 1;

    packet[offset++] =
    total_length;

    packet[offset++] =
        type;

    memcpy(
        &packet[offset],
        payload,
        length);

    offset += length;

    packet[offset++] =
        uart_calculate_crc8(
            &packet[1],
            total_length + 1);

    packet[offset++] =
        END_BYTE;

    uart->write(
        packet,
        offset);
}

// ============================================================
// Parser Reset
// ============================================================

static void uart_reset_parser()
{
    rx_state =
        RxState::WAIT_START;

    rx_length = 0;

    rx_command = 0;

    rx_index = 0;

    rx_crc = 0;
}

// ============================================================
// Parse Byte
// ============================================================

static void uart_parse_byte(
    uint8_t byte)
{
    switch (rx_state)
    {
        case RxState::WAIT_START:
        {
            if (byte == START_BYTE)
            {
                rx_state =
                    RxState::WAIT_LENGTH;
            }
        }
        break;

        case RxState::WAIT_LENGTH:
        {
            rx_length = byte;

            if (rx_length >
                UART_PAYLOAD_MAX + 1)
            {
                uart_reset_parser();
                return;
            }

            rx_index = 0;

            rx_state =
                RxState::WAIT_COMMAND;
        }
        break;

        case RxState::WAIT_COMMAND:
        {
            rx_command = byte;

            if (rx_length == 1)
            {
                rx_state =
                    RxState::WAIT_CRC;
            }
            else
            {
                rx_state =
                    RxState::WAIT_PAYLOAD;
            }
        }
        break;

        case RxState::WAIT_PAYLOAD:
        {
            rx_payload[
                rx_index++] = byte;

            if (rx_index >=
                (rx_length - 1))
            {
                rx_state =
                    RxState::WAIT_CRC;
            }
        }
        break;

        case RxState::WAIT_CRC:
        {
            rx_crc = byte;

            rx_state =
                RxState::WAIT_END;
        }
        break;

        case RxState::WAIT_END:
        {
            if (byte == END_BYTE)
            {
                uint8_t crc_buffer[
                    UART_BUFFER_SIZE];

                crc_buffer[0] =
                    rx_length;

                crc_buffer[1] =
                    rx_command;

                memcpy(
                    &crc_buffer[2],
                    rx_payload,
                    rx_length - 1);

                uint8_t crc =
                    uart_calculate_crc8(
                        crc_buffer,
                        rx_length + 1);

                if (crc == rx_crc)
                {
                    if (command_callback)
                    {
                        CommandPacket pkt;

                        pkt.command =
                            rx_command;

                        pkt.length =
                            rx_length - 1;

                        memcpy(
                            pkt.payload,
                            rx_payload,
                            pkt.length);

                        command_callback(
                            pkt);
                    }
                }
                else
                {
                    DebugSerial.println(
                        "CRC mismatch");
                }
            }

            uart_reset_parser();
        }
        break;
    }
}