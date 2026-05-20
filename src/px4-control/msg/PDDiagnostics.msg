std_msgs/Header header

# Reference + measurement (NED world, m and rad)
float64[3] position
float64[3] velocity
float64[3] position_target
float64 yaw
float64 yaw_target

# PD errors
float64[3] position_error     # p_des - p
float64[3] velocity_error     # -v (target velocity is zero)
float64[3] attitude_error     # vee(R_des^T R - R^T R_des) / 2
float64[3] rate_error         # body angular velocity (target rate is zero)

# Intermediate / outputs
float64[3] desired_accel      # PD output before gravity compensation removed
float64[3] thrust_body        # normalized FRD [-1,1]
float64[3] torque_body        # normalized FRD [-1,1]
