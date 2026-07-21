std_msgs/Header header

uint8 state                     # Current robot state
uint8 error_code                # Error code (0 = OK)
float32[3] current_angles       # Current joint angles (deg)
float32[3] encoder_values       # Raw encoder values (0-4095)
bool motors_enabled             # Motor flag
bool vacuum_on                  # Gripper state
bool synchronized               # Motor Encoder sync flag
float32 servo_position          # Current servo angle
uint16 trajectory_id            # Current executed trajectory
float32 trajectory_progress     # Progress (0.0 - 1.0)