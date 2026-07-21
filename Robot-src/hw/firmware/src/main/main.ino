#include "protocol.h"

#include "uart_handler.h"
#include "motor_control.h"
#include "trajectory.h"
#include "i2c_handler.h"

// ============================================================
// Robot State
// ============================================================

static RobotState robot_state = RobotState::IDLE;

static ErrorCode robot_error = ErrorCode::NONE;

static bool synchronized = false;

// ============================================================
// Encoder Snapshot
// ============================================================

static EncoderSnapshot encoder_snapshot;

// ============================================================
// Forward Declarations
// ============================================================

static void handle_command(const CommandPacket& packet);

static void handle_trajectory_complete(uint16_t trajectory_id);

static void send_status();

// ============================================================
// Setup
// ============================================================

void setup()
{
    // ========================================================
    // Serial Comm
    // ========================================================

    // ROS communication
    Serial0.begin(UART_BAUDRATE);

    // Debug UART
    DebugSerial.begin(115200);

    delay(1000);

    DebugSerial.println();
    
    DebugSerial.println("DELTA ROBOT START");

    // ========================================================
    // UART Transport
    // ========================================================

    uart_handler_init(Serial0);

    uart_set_command_callback(handle_command);

    // ========================================================
    // Motor Control
    // ========================================================

    if (!motor_control_init())
    {
        robot_state = RobotState::ERROR;

        robot_error = ErrorCode::MOTOR_FAULT;

        DebugSerial.println("Motor init failed");
    }

    // ========================================================
    // I2C + Encoders
    // ========================================================

    if (!i2c_handler_init())
    {
        robot_state = RobotState::ERROR;

        robot_error = ErrorCode::I2C_ERROR;

        DebugSerial.println("I2C init failed");
    }

    // ========================================================
    // Trajectory
    // ========================================================

    trajectory_init();

    trajectory_set_complete_callback(handle_trajectory_complete);

    DebugSerial.println("System ready");
}

// ============================================================
// Main Loop
// ============================================================

void loop()
{
    // ========================================================
    // UART
    // ========================================================

    uart_handler_update();

    // ========================================================
    // Trajectory
    // ========================================================

    trajectory_update();

    // ========================================================
    // Encoder Refresh
    // ========================================================

    i2c_read_encoders(encoder_snapshot);

    // ========================================================
    // State Tracking
    // ========================================================

    if (trajectory_is_active())
    {
        robot_state = RobotState::RUNNING;
    }
    else
    {
        if (robot_state != RobotState::EMERGENCY_STOP)
        {
            robot_state = RobotState::IDLE;
        }
    }

    // ========================================================
    // Periodic Status
    // ========================================================

    static uint32_t last_status_ms = 0;

    uint32_t now = millis();

    if ((now - last_status_ms) >= 100)
    {
        last_status_ms = now;

        send_status();
    }
}

// ============================================================
// Command Handler
// ============================================================

static void handle_command(const CommandPacket& packet)
{
    CommandType command = static_cast<CommandType>(packet.command);

    switch (command)
    {
        // ====================================================
        // INIT
        // ====================================================
        
        case CommandType::INIT:
        {
            trajectory_stop();

            EncoderSnapshot snapshot;

            if (!i2c_read_encoders(snapshot))
            {
                uart_send_ack(packet.command, false, ErrorCode::ENCODER_READ);

                break;
            }

            DebugSerial.println("=== INIT SYNC START ===");

            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                DebugSerial.print("Encoder ");
                DebugSerial.print(i);
                DebugSerial.print(" degrees = ");
                DebugSerial.println(snapshot.degrees[i]);
            }

            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                motor_control_set_position_degrees(i, snapshot.degrees[i]);
            }

            DebugSerial.println("=== AFTER POSITION INJECTION ===");

            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                DebugSerial.print("Motor ");
                DebugSerial.print(i);
                DebugSerial.print(" readback = ");
                DebugSerial.println(
                    motor_control_get_position_degrees(i)
                );
            }

            synchronized = true;

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // HOME
        // ====================================================

        case CommandType::HOME:
        {
            DebugSerial.println("HOME");

            trajectory_stop();

            motor_control_stop_all();

            motor_control_set_position_degrees(0, 0.0f);

            motor_control_set_position_degrees(1, 0.0f);

            motor_control_set_position_degrees(2, 0.0f);

            robot_state = RobotState::IDLE;

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // ESTOP
        // ====================================================

        case CommandType::EMERGENCY_STOP:
        {
            DebugSerial.println("EMERGENCY STOP");

            trajectory_stop();

            motor_control_emergency_stop();

            robot_state = RobotState::EMERGENCY_STOP;

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // ENABLE
        // ====================================================

        case CommandType::ENABLE_MOTORS:
        {
            motor_control_enable(true);

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // DISABLE
        // ====================================================

        case CommandType::DISABLE_MOTORS:
        {
            motor_control_enable(false);

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // VACUUM
        // ====================================================

        case CommandType::SET_VACUUM:
        {
            if (packet.length < 1)
            {
                uart_send_ack(packet.command, false, ErrorCode::INVALID_PAYLOAD);

                return;
            }

            bool enable =
                packet.payload[0];

            motor_control_set_vacuum(enable);

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // SERVO
        // ====================================================

        case CommandType::SET_SERVO:
        {
            if (packet.length < 1)
            {
                uart_send_ack(packet.command, false, ErrorCode::INVALID_PAYLOAD);

                return;
            }

            motor_control_set_servo(packet.payload[0]);

            uart_send_ack(packet.command, true, ErrorCode::NONE);
        }
        break;

        // ====================================================
        // GET STATUS
        // ====================================================

        case CommandType::GET_STATUS:
        {
            send_status();
        }
        break;

        // ====================================================
        // GET ENCODERS
        // ====================================================

        case CommandType::GET_ENCODERS:
        {
            i2c_read_encoders(encoder_snapshot);

            uart_send_encoder_data(encoder_snapshot);
        }
        break;

        // ====================================================
        // TRAJECTORY
        // ====================================================

        case CommandType::TRAJECTORY:
        {
            if (packet.length < 4)
            {
                uart_send_ack(
                    packet.command,
                    false,
                    ErrorCode::INVALID_PAYLOAD);

                return;
            }

            TrajectoryCommand traj;

            memset(
                &traj,
                0,
                sizeof(traj));

            uint16_t offset = 0;

            // =================================================
            // Header
            // =================================================

            traj.trajectory_id =
                packet.payload[offset]
                |
                (packet.payload[offset + 1] << 8);

            offset += 2;

            traj.num_points =
                packet.payload[offset]
                |
                (packet.payload[offset + 1] << 8);

            offset += 2;

            // =================================================
            // Validation
            // =================================================

            if (traj.num_points >
                MAX_TRAJECTORY_POINTS)
            {
                uart_send_ack(
                    packet.command,
                    false,
                    ErrorCode::TRAJECTORY_TOO_LONG);

                return;
            }

            // =================================================
            // Expected Payload Size
            // =================================================

            constexpr uint16_t POINT_SIZE = 20;

            uint16_t expected_size =
                4 +
                (traj.num_points * POINT_SIZE);

            if (packet.length != expected_size)
            {
                uart_send_ack(
                    packet.command,
                    false,
                    ErrorCode::INVALID_PAYLOAD);

                return;
            }

            // =================================================
            // Points
            // =================================================

            for (uint16_t i = 0;
                i < traj.num_points;
                i++)
            {
                // joint_angles[0]
                memcpy(
                    &traj.points[i].joint_angles[0],
                    &packet.payload[offset],
                    4);

                offset += 4;

                // joint_angles[1]
                memcpy(
                    &traj.points[i].joint_angles[1],
                    &packet.payload[offset],
                    4);

                offset += 4;

                // joint_angles[2]
                memcpy(
                    &traj.points[i].joint_angles[2],
                    &packet.payload[offset],
                    4);

                offset += 4;

                // time_ms
                memcpy(
                    &traj.points[i].time_ms,
                    &packet.payload[offset],
                    4);

                offset += 4;

                // vacuum
                traj.points[i].vacuum =
                    packet.payload[offset++];

                // servo_angle
                traj.points[i].servo_angle =
                    packet.payload[offset++];

                // reserved
                traj.points[i].reserved =
                    packet.payload[offset]
                    |
                    (packet.payload[offset + 1] << 8);

                offset += 2;
            }

            // =================================================
            // Start Trajectory
            // =================================================

            bool success =
                trajectory_start(traj);

            uart_send_ack(
                packet.command,
                success,
                success
                    ? ErrorCode::NONE
                    : ErrorCode::INVALID_PAYLOAD);
        }
        break;

        // ====================================================
        // INVALID
        // ====================================================

        default:
        {
            uart_send_ack(packet.command, false, ErrorCode::INVALID_COMMAND);
        }
        break;
    }
}

// ============================================================
// Trajectory Complete Callback
// ============================================================

static void handle_trajectory_complete(uint16_t trajectory_id)
{
    uart_send_trajectory_complete(trajectory_id);
}

// ============================================================
// Status Packet
// ============================================================

static void send_status()
{
    StatusPacket status;

    memset(&status, 0, sizeof(status));

    status.state = robot_state;

    status.error_code = robot_error;

    // ========================================================
    // Motor Positions
    // ========================================================

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        status.current_angles[i] = motor_control_get_position_degrees(i);

        status.motor_steps[i] = motor_control_get_position_steps(i);

        status.encoder_raw[i] = encoder_snapshot.raw[i];
    }

    // ========================================================
    // Flags
    // ========================================================

    if (synchronized)
    {
        status.flags |= FLAG_SYNCHRONIZED;
    }
    
    if (motor_control_is_enabled())
    {
        status.flags |= FLAG_MOTORS_ENABLED;
    }

    if (trajectory_is_active())
    {
        status.flags |= FLAG_TRAJECTORY_ACTIVE;
    }

    bool encoders_valid = true;

    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        if (!encoder_snapshot.valid[i])
        {
            encoders_valid = false;
            break;
        }
    }

    if (encoders_valid)
    {
        status.flags |= FLAG_ENCODER_VALID;
    }

    // ========================================================
    // Trajectory
    // ========================================================

    status.trajectory_id = trajectory_get_id();

    status.trajectory_progress = trajectory_get_progress();

    // ========================================================
    // Send
    // ========================================================

    uart_send_status(status);
}