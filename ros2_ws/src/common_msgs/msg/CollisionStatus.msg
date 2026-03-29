# Vision-based collision detection status
string status          # safe | caution | warning | emergency | sensor_unavailable | tf_unavailable
float64 min_distance   # meters, nearest obstacle to nearest robot link surface (999 = no obstacle)
string nearest_link    # robot link name closest to the obstacle
int32 obstacle_points  # number of external points within caution zone
string message
