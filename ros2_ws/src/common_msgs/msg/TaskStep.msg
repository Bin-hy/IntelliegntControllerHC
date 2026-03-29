string name
string type
string device_sn
float64[] arm_pos
int32[] hand_pos
string[] camera_type
# IO control fields (type=io, device=duco arm)
int8 io_type        # 0=standard(1-16), 1=tool(1-2)
int8 io_port        # IO port number
bool io_value       # true=HIGH, false=LOW
int32 delay_ms      # Delay after this step in milliseconds
# Lift platform fields (type=lift)
string lift_command      # move_up/move_down/stop/position_move/position_next/position_stop
int32 lift_speed_rpm     # Speed in RPM (speed mode) or 0.1rpm (position mode)
int32 lift_target_pulses # Target position in pulses (position mode)
int32 lift_accel_ms      # Acceleration time ms (position mode, default 1000)
int32 lift_decel_ms      # Deceleration time ms (position mode, default 1000)
