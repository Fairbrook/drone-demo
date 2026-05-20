std_msgs/Header header

uint8 KIND_TAKEOFF   = 1
uint8 KIND_LAND      = 2
uint8 KIND_EMERGENCY = 3
uint8 KIND_GOTO      = 4
uint8 KIND_HOLD      = 5

uint8 kind

# Goto target (NED, meters / rad). For KIND_GOTO only.
float64 x
float64 y
float64 z
float64 yaw

# Takeoff altitude AGL in meters (positive up). For KIND_TAKEOFF only.
float32 takeoff_altitude
