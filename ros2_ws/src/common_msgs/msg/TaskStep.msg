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
