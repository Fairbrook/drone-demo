#include "px4_control/pd_controller.hpp"

#include <algorithm>
#include <cmath>

namespace px4_control {

namespace {

constexpr double kGravity = 9.80665;

double clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

Eigen::Vector3d vee(const Eigen::Matrix3d &m) {
  return {m(2, 1), m(0, 2), m(1, 0)};
}

}  // namespace

ControllerOutputs PDController::compute(const ControllerInputs &in) const {
  ControllerOutputs out;

  // 1) Position PD -> desired acceleration in NED world frame.
  //    Gravity in NED acts as +g on z; we cancel it by commanding -g.
  Eigen::Vector3d e_p = in.p_des - in.p;
  Eigen::Vector3d e_v = -in.v;  // target velocity is zero
  out.position_error = e_p;
  out.velocity_error = e_v;
  Eigen::Vector3d a_des = gains_.kp_pos.cwiseProduct(e_p)
                          + gains_.kd_pos.cwiseProduct(e_v);
  a_des.z() += -kGravity;  // gravity compensation

  // Limit horizontal acceleration to bound tilt angle.
  Eigen::Vector2d a_xy(a_des.x(), a_des.y());
  double a_xy_norm = a_xy.norm();
  if (a_xy_norm > gains_.max_accel_xy) {
    a_xy *= gains_.max_accel_xy / a_xy_norm;
    a_des.x() = a_xy.x();
    a_des.y() = a_xy.y();
  }

  out.desired_accel = a_des;

  // 2) Desired body-frame z-axis (FRD: body +z is down, so thrust direction is -z).
  //    Desired thrust in world points opposite to a_des (drone accelerates up by
  //    pushing -z in body, which means -z_body must align with a_des direction).
  //    => z_body_des (world) = -a_des / |a_des|.
  Eigen::Vector3d z_b_des = -a_des;
  double a_norm = z_b_des.norm();
  if (a_norm < 1e-6) {
    // Degenerate: command neutral (still report the errors we computed).
    out.thrust_body.setZero();
    out.torque_body.setZero();
    return out;
  }
  z_b_des /= a_norm;

  // Enforce max-tilt: clamp angle between z_b_des and world -z (up) to max_tilt_rad.
  // world "up" in NED is -z = (0,0,-1).
//  Eigen::Vector3d up(0.0, 0.0, -1.0);
//  double cos_tilt = clamp(z_b_des.dot(up), -1.0, 1.0);
//  double tilt = std::acos(cos_tilt);
//  if (tilt > gains_.max_tilt_rad) {
//    Eigen::Vector3d axis = up.cross(z_b_des);
//    double axis_norm = axis.norm();
//    if (axis_norm > 1e-6) {
//      axis /= axis_norm;
//      Eigen::AngleAxisd aa(gains_.max_tilt_rad, axis);
//      z_b_des = aa * up;
//    }
//  }

  // 3) Build desired rotation matrix R_des using yaw_des as a heading constraint.
  Eigen::Vector3d x_c(std::cos(in.yaw_des), std::sin(in.yaw_des), 0.0);
  Eigen::Vector3d y_b_des = z_b_des.cross(x_c);
  double y_norm = y_b_des.norm();
  if (y_norm < 1e-6) {
    // x_c parallel to z_b_des -> pick arbitrary perpendicular.
    y_b_des = z_b_des.cross(Eigen::Vector3d::UnitY());
    y_norm = y_b_des.norm();
  }
  y_b_des /= y_norm;
  Eigen::Vector3d x_b_des = y_b_des.cross(z_b_des);

  Eigen::Matrix3d R_des;
  R_des.col(0) = x_b_des;
  R_des.col(1) = y_b_des;
  R_des.col(2) = z_b_des;

  // 4) Thrust magnitude: project a_des onto current body -z axis in world frame.
  //    R is body-FRD to NED-world; column 2 is body +z direction in world.
  Eigen::Matrix3d R = in.q.toRotationMatrix();
  Eigen::Vector3d z_body_world = R.col(2);
  double T_scalar = -a_des.dot(z_body_world);
  // Normalize by hover_thrust producing hover acceleration (g).
  double thrust_norm = clamp(T_scalar * (gains_.hover_thrust / kGravity), 0.0, 1.0);
  Eigen::Vector3d thrust_body(0.0, 0.0, thrust_norm);  // FRD: thrust along -z

  // 5) Attitude error via vee map of (R_des^T R - R^T R_des) / 2.
  Eigen::Matrix3d e_R_mat = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d e_R = vee(e_R_mat);

  // 6) Body-rate error (desired body rate = 0 for goto-to-point).
  Eigen::Vector3d e_w = in.omega;

  out.attitude_error = e_R;
  out.rate_error = e_w;

  // 7) Torque (un-normalized, in body axes).
  Eigen::Vector3d tau = -gains_.kp_att.cwiseProduct(e_R)
                        - gains_.kd_att.cwiseProduct(e_w);

  // Normalize per-axis into [-1, 1].
  Eigen::Vector3d torque_body{
      clamp(tau.x() / gains_.max_torque.x(), -1.0, 1.0),
      clamp(tau.y() / gains_.max_torque.y(), -1.0, 1.0),
      clamp(tau.z() / gains_.max_torque.z(), -1.0, 1.0)};

  out.thrust_body = thrust_body;
  out.torque_body = torque_body;
  return out;
}

}  // namespace px4_control
