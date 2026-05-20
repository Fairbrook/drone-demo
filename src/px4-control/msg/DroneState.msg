std_msgs/Header header

# Pose (NED, meters / radians)
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw

# Body-frame angular velocity (rad/s)
float64 wx
float64 wy
float64 wz

# Battery
float32 battery_voltage
float32 battery_remaining
uint8   battery_warning

# Status
uint8 arming_state
uint8 nav_state
bool  failsafe
bool  offboard_active

# Active goto target (only meaningful when goto_active=true)
bool   goto_active
float64 goto_x
float64 goto_y
float64 goto_z
float64 goto_yaw
