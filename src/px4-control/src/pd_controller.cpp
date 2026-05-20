#include "px4_control/pd_controller.hpp"

#include <algorithm>
#include <cmath>

namespace px4_control {

namespace {

constexpr double kGravity = 9.80665;

double clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

// Log map of a unit quaternion, matching the convention used in the
// forerunner2 low-level controller: r = (vec(q)/|vec(q)|) * acos(w).
// For q = (cos(θ/2), sin(θ/2)·n̂), this yields (θ/2)·n̂.
Eigen::Vector3d quatLog(const Eigen::Quaterniond &q_in) {
  Eigen::Quaterniond q(q_in);
  q.normalize();
  const Eigen::Vector3d v(q.x(), q.y(), q.z());
  const double n = v.norm();
  if (n < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return (v / n) * std::acos(clamp(q.w(), -1.0, 1.0));
}

// Quaternion that rotates body (0,0,1) onto the unit vector h, expressed in
// the world frame. Standard half-angle construction; axis = nz × h.
Eigen::Quaterniond quatFromZ(const Eigen::Vector3d &h) {
  static const Eigen::Vector3d nz(0.0, 0.0, 1.0);
  const double dot = clamp(nz.dot(h), -1.0, 1.0);
  const Eigen::Vector3d axis_uvec = nz.cross(h);
  const double axis_norm = axis_uvec.norm();
  if (axis_norm < 1e-9) {
    if (dot > 0.0) return Eigen::Quaterniond::Identity();
    // h is anti-parallel to nz — 180° flip about body x.
    return Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
  }
  const double w = std::sqrt(std::max(0.0, (1.0 + dot)) * 0.5);
  const Eigen::Vector3d xyz =
      (axis_uvec / axis_norm) * std::sqrt(std::max(0.0, (1.0 - dot)) * 0.5);
  return Eigen::Quaterniond(w, xyz.x(), xyz.y(), xyz.z());
}

}  // namespace

ControllerOutputs PDController::compute(const ControllerInputs &in) const {
  ControllerOutputs out;

  // 1) Position PD → desired acceleration in NED world frame.
  //    a_des = -kp·(p - p_des) - kd·v − g_NED  (gravity comp; NED g = (0,0,+g)).
  const Eigen::Vector3d e_p = in.p - in.p_des;
  const Eigen::Vector3d e_v = in.v;  // target velocity is zero
  out.position_error = -e_p;         // diagnostics carry (p_des − p)
  out.velocity_error = -e_v;

  Eigen::Vector3d a_des = -gains_.kp_pos.cwiseProduct(e_p)
                          - gains_.kd_pos.cwiseProduct(e_v);
  a_des.z() += -kGravity;  // (0,0,-g): cancels NED gravity (points "up").

  // Bound horizontal acceleration so tilt stays sane.
  Eigen::Vector2d a_xy(a_des.x(), a_des.y());
  const double a_xy_norm = a_xy.norm();
  if (a_xy_norm > gains_.max_accel_xy) {
    a_xy *= gains_.max_accel_xy / a_xy_norm;
    a_des.x() = a_xy.x();
    a_des.y() = a_xy.y();
  }
  out.desired_accel = a_des;

  const Eigen::Matrix3d R = in.q.toRotationMatrix();

  // 2) Thrust magnitude: project required acceleration onto body −z (in world).
  //    PX4 FRD: thrust is applied along −z_body. R.col(2) = z_body in world.
  const double T_scalar = -a_des.dot(R.col(2));
  const double T_norm =
      clamp(T_scalar * (gains_.hover_thrust / kGravity), 0.0, 1.0);
  // Match existing px4_control_node sign convention.
  Eigen::Vector3d thrust_body(0.0, 0.0, -T_norm);

  // 3) Build desired attitude quaternion.
  //    Drone applies acceleration along −z_body. Want −z_body_world = a_des/|a_des|.
  //    ⇒ desired z_body in world = h = −a_des/|a_des|.
  const double a_norm = a_des.norm();
  if (a_norm < 1e-6) {
    // Degenerate: keep current attitude, no torque.
    out.thrust_body = thrust_body;
    out.torque_body.setZero();
    return out;
  }
  const Eigen::Vector3d h = -a_des / a_norm;
  const Eigen::Quaterniond q_align = quatFromZ(h);

  // Yaw about world z (NED +yaw = clockwise viewed from above).
  const Eigen::Quaterniond q_yaw(std::cos(in.yaw_des * 0.5), 0.0, 0.0,
                                 std::sin(in.yaw_des * 0.5));
  Eigen::Quaterniond q_target = q_align * q_yaw;
  q_target.normalize();

  // 4) Attitude error (body frame): q_e = q_target⁻¹ ⊗ q_current. Take short path.
  Eigen::Quaterniond q_e = q_target.conjugate() * in.q;
  q_e.normalize();
  if (q_e.w() < 0.0) {
    q_e.coeffs() *= -1.0;
  }

  const Eigen::Vector3d e_R = quatLog(q_e);
  out.attitude_error = e_R;
  out.rate_error = in.omega;

  // 5) PD → angular-acceleration command, with tanh saturation on magnitude.
  Eigen::Vector3d u_r = -gains_.kp_att.cwiseProduct(e_R)
                        - gains_.kd_att.cwiseProduct(in.omega);
  const double u_norm = u_r.norm();
  if (u_norm > 1e-9 && gains_.krmax > 0.0) {
    u_r = gains_.krmax * std::tanh(u_norm / gains_.krmax) * (u_r / u_norm);
  }

  // 6) Torque in body axes: τ = J·u + ω × (J·ω).
  const Eigen::Vector3d Jw = gains_.inertia.cwiseProduct(in.omega);
  const Eigen::Vector3d tau = gains_.inertia.cwiseProduct(u_r) + in.omega.cross(Jw);

  // Per-axis normalize to PX4 [-1, 1] body-frame torque setpoint.
  Eigen::Vector3d torque_body{
      clamp(tau.x() / gains_.max_torque.x(), -1.0, 1.0),
      clamp(tau.y() / gains_.max_torque.y(), -1.0, 1.0),
      clamp(tau.z() / gains_.max_torque.z(), -1.0, 1.0)};

  out.thrust_body = thrust_body;
  out.torque_body = torque_body;
  return out;
}

}  // namespace px4_control
