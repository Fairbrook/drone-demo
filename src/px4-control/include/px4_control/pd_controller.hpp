#pragma once

#include <Eigen/Dense>

namespace px4_control {

struct PDGains {
  Eigen::Vector3d kp_pos{1.5, 1.5, 3.0};
  Eigen::Vector3d kd_pos{1.2, 1.2, 2.5};
  Eigen::Vector3d kp_att{6.0, 6.0, 2.0};
  Eigen::Vector3d kd_att{0.8, 0.8, 0.4};
  double hover_thrust = 0.5;
  double max_tilt_rad = 0.5;
  double max_accel_xy = 6.0;
  Eigen::Vector3d max_torque{0.5, 0.5, 0.2};
};

struct ControllerInputs {
  Eigen::Vector3d p;          // NED position [m]
  Eigen::Quaterniond q;       // attitude, body FRD -> NED, Hamilton w,x,y,z
  Eigen::Vector3d v;          // NED velocity [m/s]
  Eigen::Vector3d omega;      // body FRD angular velocity [rad/s]
  Eigen::Vector3d p_des;      // NED target position [m]
  double yaw_des = 0.0;       // NED yaw [rad]
};

struct ControllerOutputs {
  // Commands to PX4 (normalized [-1,1]^3, FRD body axes).
  Eigen::Vector3d thrust_body;
  Eigen::Vector3d torque_body;

  // Diagnostics: errors and intermediates, in NED world unless noted.
  Eigen::Vector3d position_error{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity_error{0.0, 0.0, 0.0};
  Eigen::Vector3d attitude_error{0.0, 0.0, 0.0};   // e_R, body frame
  Eigen::Vector3d rate_error{0.0, 0.0, 0.0};       // e_omega, body frame
  Eigen::Vector3d desired_accel{0.0, 0.0, 0.0};    // a_des before normalization
};

class PDController {
 public:
  explicit PDController(const PDGains &gains = {}) : gains_(gains) {}

  void setGains(const PDGains &gains) { gains_ = gains; }
  const PDGains &gains() const { return gains_; }

  ControllerOutputs compute(const ControllerInputs &in) const;

 private:
  PDGains gains_;
};

}  // namespace px4_control
