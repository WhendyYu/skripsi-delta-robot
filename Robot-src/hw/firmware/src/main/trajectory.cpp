#include "trajectory.h"

#include "motor_control.h"

// ============================================================
// Runtime State
// ============================================================

static bool trajectory_active = false;

static bool trajectory_complete = false;

static bool segment_started = false;

static uint16_t trajectory_id = 0;

static uint8_t current_segment = 0;

// ============================================================
// Trajectory Data
// ============================================================

static TrajectoryCommand
active_trajectory;

// ============================================================
// Completion Callback
// ============================================================

static TrajectoryCompleteCallback
complete_callback = nullptr;

// ============================================================
// Internal
// ============================================================

static float nearest_equivalent(
    float target,
    float current);

static float clampf(
    float value,
    float min_v,
    float max_v);

// ============================================================
// Initialization
// ============================================================

bool trajectory_init()
{
    memset(
        &active_trajectory,
        0,
        sizeof(TrajectoryCommand));

    trajectory_active = false;

    trajectory_complete = false;

    segment_started = false;

    current_segment = 0;

    DebugSerial.println(
        "Trajectory initialized");

    return true;
}

// ============================================================
// Start
// ============================================================

bool trajectory_start(
    const TrajectoryCommand& trajectory)
{
    if (trajectory.num_points < 2)
    {
        DebugSerial.println(
            "Trajectory rejected");

        return false;
    }

    if (trajectory.num_points >
        MAX_TRAJECTORY_POINTS)
    {
        DebugSerial.println(
            "Trajectory too large");

        return false;
    }

    memcpy(
        &active_trajectory,
        &trajectory,
        sizeof(TrajectoryCommand));

    trajectory_id =
        trajectory.trajectory_id;

    current_segment = 0;

    trajectory_active = true;

    trajectory_complete = false;

    segment_started = false;

    DebugSerial.print(
        "Trajectory start: ");

    DebugSerial.println(
        trajectory_id);

    return true;
}

// ============================================================
// Stop
// ============================================================

void trajectory_stop()
{
    trajectory_active = false;

    trajectory_complete = false;

    segment_started = false;

    motor_control_stop_all();

    DebugSerial.println(
        "Trajectory stopped");
}

// ============================================================
// Update
// ============================================================

void trajectory_update()
{
    if (!trajectory_active)
    {
        return;
    }

    // ========================================================
    // Start Segment
    // ========================================================

    if (!segment_started)
    {
        TrajectoryPoint* p0 =
            &active_trajectory.points[
                current_segment];

        TrajectoryPoint* p1 =
            &active_trajectory.points[
                current_segment + 1];

        motor_control_set_vacuum(
            p1->vacuum != 0);

        motor_control_set_servo(
            p1->servo_angle);

        float duration_sec =
            (p1->time_ms -
             p0->time_ms)
            * 0.001f;

        if (duration_sec <= 0.0f)
        {
            duration_sec = 0.001f;
        }

        DebugSerial.print(
            "Segment ");

        DebugSerial.print(
            current_segment);

        DebugSerial.print(
            " duration: ");

        DebugSerial.println(
            duration_sec,
            4);

        // ====================================================
        // Start Coordinated Motion
        // ====================================================

        for (uint8_t i = 0;
             i < NUM_MOTORS;
             i++)
        {
            float q0 =
                p0->joint_angles[i];

            float q1 =
                nearest_equivalent(
                    p1->joint_angles[i],
                    q0);

            float delta_deg =
                fabs(q1 - q0);

            float speed_deg_sec =
                delta_deg /
                duration_sec;

            speed_deg_sec =
                clampf(
                    speed_deg_sec,
                    1.0f,
                    MAX_JOINT_VEL);

            DebugSerial.print(
                "M");

            DebugSerial.print(i);

            DebugSerial.print(
                " target=");

            DebugSerial.print(
                q1,
                3);

            DebugSerial.print(
                " speed=");

            DebugSerial.println(
                speed_deg_sec,
                3);

            motor_control_move_to_degrees(
                i,
                q1,
                speed_deg_sec);
        }

        segment_started = true;
    }

    // ========================================================
    // Wait For Completion
    // ========================================================

    if (!motor_control_all_complete())
    {
        return;
    }

    // ========================================================
    // Advance Segment
    // ========================================================

    current_segment++;

    segment_started = false;

    // ========================================================
    // Entire Trajectory Complete
    // ========================================================

    if (current_segment >=
        (active_trajectory.num_points - 1))
    {
        trajectory_active = false;

        trajectory_complete = true;

        DebugSerial.println(
            "Trajectory complete");

        if (complete_callback)
        {
            complete_callback(
                trajectory_id);
        }
    }
}

// ============================================================
// Status
// ============================================================

bool trajectory_is_active()
{
    return trajectory_active;
}

bool trajectory_is_complete()
{
    return trajectory_complete;
}

uint8_t trajectory_get_progress()
{
    if (!trajectory_active)
    {
        return trajectory_complete
            ? 100
            : 0;
    }

    return (
        (current_segment + 1)
        * 100)
        / (active_trajectory.num_points - 1);
}

uint16_t trajectory_get_id()
{
    return trajectory_id;
}

// ============================================================
// Callback
// ============================================================

void trajectory_set_complete_callback(
    TrajectoryCompleteCallback callback)
{
    complete_callback =
        callback;
}

// ============================================================
// Helper
// ============================================================

static float clampf(
    float value,
    float min_v,
    float max_v)
{
    if (value < min_v)
    {
        return min_v;
    }

    if (value > max_v)
    {
        return max_v;
    }

    return value;
}

static float nearest_equivalent(
    float target,
    float current)
{
    float best = target;

    float best_error =
        fabs(target - current);

    for (int k = -3;
         k <= 3;
         k++)
    {
        float candidate =
            target +
            (360.0f * k);

        float error =
            fabs(candidate - current);

        if (error < best_error)
        {
            best = candidate;
            best_error = error;
        }
    }

    return best;
}