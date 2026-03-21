string task_name
int32 task_id
int64 created_time
int32 exec_rounds
string collision_camera_sn   # Camera SN (or topic prefix) used for collision detection during this task. Empty = use default.
TaskDeviceCheck[] device_checks
TaskStep[] task_seqs
