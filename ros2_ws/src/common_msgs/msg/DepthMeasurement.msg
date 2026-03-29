# Depth camera measurement result (ear/target position)
float64 depth_mm          # measured depth in millimeters
float64 pos_x             # 3D position X in camera frame (meters)
float64 pos_y             # 3D position Y in camera frame (meters)
float64 pos_z             # 3D position Z in camera frame (meters)
int32 roi_center_u        # measurement center pixel U
int32 roi_center_v        # measurement center pixel V
int32 valid_points        # valid depth pixels used for measurement
string status             # measuring | no_depth | no_camera_info | idle
string camera_ns          # source camera namespace
