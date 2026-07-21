#include "motor_control.h"

#include <FastAccelStepper.h>
#include <TMCStepper.h>

// ============================================================
// Pin Configuration
// ============================================================

// Motor 1
constexpr uint8_t M1_STEP_PIN = 6;
constexpr uint8_t M1_DIR_PIN  = 5;

// Motor 2
constexpr uint8_t M2_STEP_PIN = 16;
constexpr uint8_t M2_DIR_PIN  = 15;

// Motor 3
constexpr uint8_t M3_STEP_PIN = 11;
constexpr uint8_t M3_DIR_PIN  = 10;

// Shared enable
constexpr uint8_t MOTOR_ENABLE_PIN = 7;

// TMC UART
constexpr uint8_t TMC_UART_RX = 18;
constexpr uint8_t TMC_UART_TX = 17;

constexpr uint32_t TMC_UART_BAUD = 115200;

// Vacuum + Servo
constexpr uint8_t VACUUM_PIN = 12;

constexpr uint8_t SERVO_PIN = 13;

// ============================================================
// Motion Limits
// ============================================================

constexpr float MAX_SPEED_HZ =
    25000.0f;

constexpr float MAX_ACCEL_HZ =
    80000.0f;

// ============================================================
// TMC2209
// ============================================================

constexpr float R_SENSE =
    0.11f;

// ============================================================
// Motor Direction Signs
// ============================================================

static const int8_t
motor_sign[NUM_MOTORS] =
{
    1,
    1,
    1
};

// ============================================================
// UART
// ============================================================

static HardwareSerial&
tmc_serial =
    Serial1;

// ============================================================
// FastAccelStepper
// ============================================================

static FastAccelStepperEngine
engine;

static FastAccelStepper*
stepper[NUM_MOTORS] =
{
    nullptr,
    nullptr,
    nullptr
};

// ============================================================
// TMC2209 Drivers
// ============================================================

static TMC2209Stepper
driver0(
    &tmc_serial,
    R_SENSE,
    0);

static TMC2209Stepper
driver1(
    &tmc_serial,
    R_SENSE,
    1);

static TMC2209Stepper
driver2(
    &tmc_serial,
    R_SENSE,
    2);

// ============================================================
// Runtime State
// ============================================================

static bool motors_enabled =
    false;

static bool vacuum_enabled =
    false;

// ============================================================
// Internal
// ============================================================

static void configure_driver(
    TMC2209Stepper& driver);

// ============================================================
// Initialization
// ============================================================

bool motor_control_init()
{
    pinMode(
        MOTOR_ENABLE_PIN,
        OUTPUT);

    digitalWrite(
        MOTOR_ENABLE_PIN,
        HIGH);

    // ========================================================
    // TMC UART
    // ========================================================

    tmc_serial.begin(
        TMC_UART_BAUD,
        SERIAL_8N1,
        TMC_UART_RX,
        TMC_UART_TX);

    delay(100);

    configure_driver(
        driver0);

    configure_driver(
        driver1);

    configure_driver(
        driver2);

    // ========================================================
    // Stepper Engine
    // ========================================================

    engine.init();

    stepper[0] =
        engine.stepperConnectToPin(
            M1_STEP_PIN);

    stepper[1] =
        engine.stepperConnectToPin(
            M2_STEP_PIN);

    stepper[2] =
        engine.stepperConnectToPin(
            M3_STEP_PIN);

    const uint8_t dir_pins[] =
    {
        M1_DIR_PIN,
        M2_DIR_PIN,
        M3_DIR_PIN
    };

    for (uint8_t i = 0;
         i < NUM_MOTORS;
         i++)
    {
        if (stepper[i] == nullptr)
        {
            DebugSerial.println(
                "Stepper init failed");

            return false;
        }

        stepper[i]
            ->setDirectionPin(
                dir_pins[i]);

        stepper[i]
            ->setEnablePin(
                MOTOR_ENABLE_PIN);

        stepper[i]
            ->setAutoEnable(false);

        stepper[i]
            ->setAcceleration(
                MAX_ACCEL_HZ);

        stepper[i]
            ->setSpeedInHz(1000);

        stepper[i]
            ->setCurrentPosition(0);
    }

    // ========================================================
    // Outputs
    // ========================================================

    pinMode(
        VACUUM_PIN,
        OUTPUT);

    digitalWrite(
        VACUUM_PIN,
        LOW);

    pinMode(
        SERVO_PIN,
        OUTPUT);

    DebugSerial.println(
        "Motor control initialized");

    return true;
}

// ============================================================
// Driver Configuration
// ============================================================

static void configure_driver(
    TMC2209Stepper& driver)
{
    driver.begin();

    driver.toff(5);

    driver.blank_time(24);

    driver.rms_current(1100);

    driver.microsteps(
        MICROSTEPS);

    driver.pdn_disable(true);

    driver.I_scale_analog(false);

    driver.en_spreadCycle(true);

    driver.internal_Rsense(false);

    driver.TPWMTHRS(0xFFFFF);

    driver.semin(5);

    driver.semax(2);

    driver.sedn(0b01);
}

// ============================================================
// Enable / Disable
// ============================================================

void motor_control_enable(
    bool enable)
{
    motors_enabled = enable;

    digitalWrite(
        MOTOR_ENABLE_PIN,
        enable ? LOW : HIGH);

    DebugSerial.print(
        "Motors ");

    DebugSerial.println(
        enable
        ? "ENABLED"
        : "DISABLED");
}

bool motor_control_is_enabled()
{
    return motors_enabled;
}

// ============================================================
// Legacy Velocity Functions
// ============================================================

void motor_control_run_forward(
    uint8_t motor,
    float speed_hz)
{
    if (motor >= NUM_MOTORS)
    {
        return;
    }

    if (!motors_enabled)
    {
        return;
    }

    speed_hz = fabs(speed_hz);

    if (speed_hz > MAX_SPEED_HZ)
    {
        speed_hz = MAX_SPEED_HZ;
    }

    bool physical_forward =
        (motor_sign[motor] > 0);

    stepper[motor]
        ->setSpeedInHz(speed_hz);

    if (physical_forward)
    {
        stepper[motor]
            ->runForward();
    }
    else
    {
        stepper[motor]
            ->runBackward();
    }
}

void motor_control_run_backward(
    uint8_t motor,
    float speed_hz)
{
    if (motor >= NUM_MOTORS)
    {
        return;
    }

    if (!motors_enabled)
    {
        return;
    }

    speed_hz = fabs(speed_hz);

    if (speed_hz > MAX_SPEED_HZ)
    {
        speed_hz = MAX_SPEED_HZ;
    }

    bool physical_forward =
        (motor_sign[motor] < 0);

    stepper[motor]
        ->setSpeedInHz(speed_hz);

    if (physical_forward)
    {
        stepper[motor]
            ->runForward();
    }
    else
    {
        stepper[motor]
            ->runBackward();
    }
}

void motor_control_set_velocity(
    uint8_t motor,
    float velocity_deg_sec)
{
    float speed_hz =
        fabs(
            velocity_deg_sec)
        * STEPS_PER_DEG;

    if (velocity_deg_sec >= 0.0f)
    {
        motor_control_run_forward(
            motor,
            speed_hz);
    }
    else
    {
        motor_control_run_backward(
            motor,
            speed_hz);
    }
}

// ============================================================
// Position Motion (NEW)
// ============================================================

bool motor_control_move_to_steps(
    uint8_t motor,
    int32_t target_steps,
    float speed_deg_sec)
{
    if (motor >= NUM_MOTORS)
    {
        return false;
    }

    if (!motors_enabled)
    {
        return false;
    }

    float speed_hz =
        fabs(speed_deg_sec)
        * STEPS_PER_DEG;

    if (speed_hz < 1.0f)
    {
        speed_hz = 1.0f;
    }

    if (speed_hz > MAX_SPEED_HZ)
    {
        speed_hz = MAX_SPEED_HZ;
    }

    stepper[motor]
        ->setSpeedInHz(
            speed_hz);

    stepper[motor]
        ->moveTo(
            target_steps
            * motor_sign[motor]);

    return true;
}

bool motor_control_move_to_degrees(
    uint8_t motor,
    float target_deg,
    float speed_deg_sec)
{
    return motor_control_move_to_steps(
        motor,
        degrees_to_steps(
            target_deg),
        speed_deg_sec);
}

bool motor_control_is_running(
    uint8_t motor)
{
    if (motor >= NUM_MOTORS)
    {
        return false;
    }

    return stepper[motor]
        ->isRunning();
}

bool motor_control_all_complete()
{
    for (uint8_t i = 0;
         i < NUM_MOTORS;
         i++)
    {
        if (motor_control_is_running(i))
        {
            return false;
        }
    }

    return true;
}

// ============================================================
// Stop
// ============================================================

void motor_control_stop(
    uint8_t motor)
{
    if (motor >= NUM_MOTORS)
    {
        return;
    }

    stepper[motor]
        ->forceStop();
}

void motor_control_stop_all()
{
    for (uint8_t i = 0;
         i < NUM_MOTORS;
         i++)
    {
        motor_control_stop(i);
    }
}

// ============================================================
// Position Access
// ============================================================

int32_t motor_control_get_position_steps(
    uint8_t motor)
{
    if (motor >= NUM_MOTORS)
    {
        return 0;
    }

    int32_t raw =
        stepper[motor]
            ->getCurrentPosition();

    return raw *
           motor_sign[motor];
}

float motor_control_get_position_degrees(
    uint8_t motor)
{
    return steps_to_degrees(
        motor_control_get_position_steps(
            motor));
}

void motor_control_set_position_steps(
    uint8_t motor,
    int32_t steps)
{
    if (motor >= NUM_MOTORS)
    {
        return;
    }

    stepper[motor]
        ->setCurrentPosition(
            steps
            * motor_sign[motor]);
}

void motor_control_set_position_degrees(
    uint8_t motor,
    float degrees)
{
    motor_control_set_position_steps(
        motor,
        degrees_to_steps(
            degrees));
}

// ============================================================
// Vacuum
// ============================================================

void motor_control_set_vacuum(
    bool enabled)
{
    vacuum_enabled =
        enabled;

    digitalWrite(
        VACUUM_PIN,
        enabled
        ? HIGH
        : LOW);
}

// ============================================================
// Servo
// ============================================================

void motor_control_set_servo(
    uint8_t position)
{
    // Placeholder
}

// ============================================================
// Emergency Stop
// ============================================================

void motor_control_emergency_stop()
{
    motor_control_stop_all();

    digitalWrite(
        MOTOR_ENABLE_PIN,
        HIGH);

    motors_enabled =
        false;

    DebugSerial.println(
        "EMERGENCY STOP");
}