string name
string type
string device_sn
# Arm step fields (type=arm)
string arm_command       # "movej" or "movel" (default: "movej")
float64[] arm_pos        # Joint angles in degrees (MoveJ: 6 joints)
float64[] arm_cart_pos   # Cartesian target: X,Y,Z in mm; RX,RY,RZ in degrees
float64 arm_velocity     # MoveJ: deg/s; MoveL: mm/s
float64 arm_accel        # MoveJ: deg/s²; MoveL: mm/s²
int32[] hand_pos
string[] camera_type
# IO control fields (type=io, device=duco arm)
int8 io_type        # 0=standard(1-16), 1=tool(1-2)
int8 io_port        # IO port number
bool io_value       # true=HIGH, false=LOW
int32 delay_ms      # Delay after this step in milliseconds
# Lift platform fields (type=lift)
string lift_command      # move_up/move_down/stop/position_move/position_next/position_stop
int32 lift_speed_rpm     # Speed in RPM
int32 lift_target_pulses # Target position in pulses (position mode)
int32 lift_accel_ms      # Acceleration time ms (position mode, default 1000)
int32 lift_decel_ms      # Deceleration time ms (position mode, default 1000)
# Device control fields (type=control)
# control_target: arm / lhand / rhand / lift
# control_command: poweron / enable / disable / poweroff / home
string control_target
string control_command
