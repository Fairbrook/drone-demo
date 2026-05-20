#include "px4_control/px4_control_node.hpp"

#include <chrono>
#include <cmath>

namespace px4_control {

using namespace std::chrono_literals;

namespace {

Eigen::Quaterniond px4QuatToEigen(const std::array<float, 4> &q) {
  // PX4 stores Hamilton w,x,y,z.
  return {q[0], q[1], q[2], q[3]};
}

void quatToRpy(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();
  roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  double sinp = 2.0 * (w * y - z * x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

Px4ControlNode::Px4ControlNode() : Node("px4_control_node") {
  loadParameters();

  // Publishers.
  offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4PubQos());
  thrust_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", px4PubQos());
  torque_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", px4PubQos());
  command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4PubQos());
  state_pub_ = create_publisher<px4_control::msg::DroneState>(
      "/tui/state", rclcpp::QoS(10).reliable());
  diag_pub_ = create_publisher<px4_control::msg::PDDiagnostics>(
      "/px4_control/pd_diagnostics", rclcpp::QoS(10).best_effort());

  // Subscribers.
  odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4SubQos(),
      std::bind(&Px4ControlNode::onOdometry, this, std::placeholders::_1));
  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4SubQos(),
      std::bind(&Px4ControlNode::onStatus, this, std::placeholders::_1));
  battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", px4SubQos(),
      std::bind(&Px4ControlNode::onBattery, this, std::placeholders::_1));
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4SubQos(),
      std::bind(&Px4ControlNode::onAttitude, this, std::placeholders::_1));
  cmd_sub_ = create_subscription<px4_control::msg::DroneCommand>(
      "/tui/cmd", rclcpp::QoS(10).reliable(),
      std::bind(&Px4ControlNode::onCommand, this, std::placeholders::_1));

  // Timers.
  control_timer_ = create_wall_timer(
      5ms, std::bind(&Px4ControlNode::controlTick, this));  // 200 Hz
  state_timer_ = create_wall_timer(
      100ms, std::bind(&Px4ControlNode::statePublishTick, this));  // 10 Hz

  RCLCPP_INFO(get_logger(), "px4_control_node started");
}

void Px4ControlNode::loadParameters() {
  PDGains g = pd_.gains();
  auto declare_vec3 = [&](const std::string &name, Eigen::Vector3d &dst) {
    std::vector<double> def{dst.x(), dst.y(), dst.z()};
    auto v = declare_parameter<std::vector<double>>(name, def);
    if (v.size() == 3) {
      dst = {v[0], v[1], v[2]};
    }
  };
  declare_vec3("kp_pos", g.kp_pos);
  declare_vec3("kd_pos", g.kd_pos);
  declare_vec3("kp_att", g.kp_att);
  declare_vec3("kd_att", g.kd_att);
  declare_vec3("max_torque", g.max_torque);
  g.hover_thrust = declare_parameter<double>("hover_thrust", g.hover_thrust);
  g.max_tilt_rad = declare_parameter<double>("max_tilt_rad", g.max_tilt_rad);
  g.max_accel_xy = declare_parameter<double>("max_accel_xy", g.max_accel_xy);
  pd_.setGains(g);
}

rclcpp::QoS Px4ControlNode::px4SubQos() const {
  // uXRCE-DDS publishes BEST_EFFORT, VOLATILE.
  return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
}

rclcpp::QoS Px4ControlNode::px4PubQos() const {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

uint64_t Px4ControlNode::nowUs() {
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

void Px4ControlNode::onOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  last_odom_ = *msg;
  last_odom_stamp_ = now();
}

void Px4ControlNode::onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  last_status_ = *msg;
}

void Px4ControlNode::onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
  last_battery_ = *msg;
}

void Px4ControlNode::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  last_attitude_ = *msg;
}

void Px4ControlNode::onCommand(const px4_control::msg::DroneCommand::SharedPtr msg) {
  using DC = px4_control::msg::DroneCommand;
  switch (msg->kind) {
    case DC::KIND_TAKEOFF:
      RCLCPP_INFO(get_logger(), "Takeoff to %.2f m AGL", msg->takeoff_altitude);
      takeoff_altitude_agl_ = msg->takeoff_altitude;
      saw_auto_takeoff_ = false;
      requestArm();
      requestTakeoff(msg->takeoff_altitude);
      flight_state_ = FlightState::TAKING_OFF;
      break;
    case DC::KIND_LAND:
      RCLCPP_INFO(get_logger(), "Land");
      requestLand();
      flight_state_ = FlightState::LANDING;
      break;
    case DC::KIND_EMERGENCY:
      RCLCPP_WARN(get_logger(), "EMERGENCY - disarming");
      requestDisarm();
      flight_state_ = FlightState::EMERGENCY;
      break;
    case DC::KIND_GOTO: {
      goto_target_ = {msg->x, msg->y, msg->z};
      goto_yaw_ = msg->yaw;
      RCLCPP_INFO(get_logger(), "Goto (%.2f, %.2f, %.2f) yaw=%.2f",
                  msg->x, msg->y, msg->z, msg->yaw);
      // Only re-warmup if not already in an active offboard PD state.
      if (flight_state_ != FlightState::GOTO && flight_state_ != FlightState::HOLD) {
        offboard_warmup_ticks_ = 0;
      }
      flight_state_ = FlightState::GOTO;
      break;
    }
    case DC::KIND_HOLD:
      if (last_odom_) {
        goto_target_ = {last_odom_->position[0], last_odom_->position[1],
                        last_odom_->position[2]};
        Eigen::Quaterniond q = px4QuatToEigen(last_odom_->q);
        double r, p, y;
        quatToRpy(q, r, p, y);
        goto_yaw_ = y;
        if (flight_state_ != FlightState::GOTO && flight_state_ != FlightState::HOLD) {
          offboard_warmup_ticks_ = 0;
        }
        flight_state_ = FlightState::HOLD;
        RCLCPP_INFO(get_logger(), "Hold at current position");
      }
      break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown command kind %u", msg->kind);
      break;
  }
}

void Px4ControlNode::controlTick() {
  // Stream OffboardControlMode unconditionally so PX4 will accept setpoints.
  publishOffboardControlMode();

  const bool odom_fresh =
      last_odom_ && (now() - last_odom_stamp_).seconds() < kOdomStaleSec;

  if (!odom_fresh) {
    publishNeutralSetpoints();
    return;
  }

  // Detect when PX4 finishes a takeoff so we can take back control via HOLD.
  // PX4 can transition out of AUTO_TAKEOFF before the vehicle has actually
  // climbed (and may have already been in AUTO_LOITER before takeoff was
  // commanded), so we only accept the handover once we have *seen* the
  // AUTO_TAKEOFF nav state AND the vehicle is near the requested altitude.
  if (flight_state_ == FlightState::TAKING_OFF && last_status_) {
    const auto nav = last_status_->nav_state;
    if (nav == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF) {
      saw_auto_takeoff_ = true;
    }
    // PX4 NED: +z is down, so AGL altitude = -position.z.
    const double current_alt_agl = -last_odom_->position[2];
    const bool altitude_reached =
        current_alt_agl >=
        static_cast<double>(takeoff_altitude_agl_) - kTakeoffAltitudeTolM;
    const bool px4_left_takeoff =
        nav == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER ||
        nav == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
    if (saw_auto_takeoff_ && altitude_reached && px4_left_takeoff) {
      // Hold at the *requested* altitude, not the (possibly still-climbing)
      // current z, to avoid handing the PD controller a low setpoint that
      // triggers PX4's land detector.
      goto_target_ = {last_odom_->position[0], last_odom_->position[1],
                      -static_cast<double>(takeoff_altitude_agl_)};
      Eigen::Quaterniond q = px4QuatToEigen(last_odom_->q);
      double r, p, yy;
      quatToRpy(q, r, p, yy);
      goto_yaw_ = yy;
      offboard_warmup_ticks_ = 0;
      flight_state_ = FlightState::HOLD;
      RCLCPP_INFO(get_logger(), "Takeoff complete (alt=%.2f m) -> HOLD",
                  current_alt_agl);
    }
  }

  switch (flight_state_) {
    case FlightState::GOTO:
    case FlightState::HOLD: {
      if (offboard_warmup_ticks_ < kOffboardWarmupTicks) {
        offboard_warmup_ticks_++;
        publishNeutralSetpoints();
        if (offboard_warmup_ticks_ == kOffboardWarmupTicks) {
          // Just finished warmup — request OFFBOARD + arm once.
          requestOffboardMode();
          requestArm();
        }
        break;
      }

      ControllerInputs in;
      in.p = {last_odom_->position[0], last_odom_->position[1],
              last_odom_->position[2]};
      in.q = px4QuatToEigen(last_odom_->q);
      in.v = {last_odom_->velocity[0], last_odom_->velocity[1],
              last_odom_->velocity[2]};
      in.omega = {last_odom_->angular_velocity[0],
                  last_odom_->angular_velocity[1],
                  last_odom_->angular_velocity[2]};
      in.p_des = goto_target_;
      in.yaw_des = goto_yaw_;
      auto out = pd_.compute(in);
      publishThrustTorque(out.thrust_body, out.torque_body);
      publishDiagnostics(in, out);
      break;
    }
    case FlightState::TAKING_OFF:
    case FlightState::LANDING:
    case FlightState::EMERGENCY:
    case FlightState::IDLE:
    default:
      publishNeutralSetpoints();
      break;
  }
}

void Px4ControlNode::publishOffboardControlMode() {
  px4_msgs::msg::OffboardControlMode m{};
  m.timestamp = nowUs();
  m.position = false;
  m.velocity = false;
  m.acceleration = false;
  m.attitude = false;
  m.body_rate = false;
  m.thrust_and_torque = true;
  m.direct_actuator = false;
  offboard_pub_->publish(m);
}

void Px4ControlNode::publishThrustTorque(const Eigen::Vector3d &thrust,
                                         const Eigen::Vector3d &torque) {
  const uint64_t t = nowUs();
  px4_msgs::msg::VehicleThrustSetpoint th{};
  th.timestamp = t;
  th.timestamp_sample = t;
  th.xyz[0] = static_cast<float>(thrust.x());
  th.xyz[1] = static_cast<float>(thrust.y());
  th.xyz[2] = static_cast<float>(thrust.z());
  thrust_pub_->publish(th);

  px4_msgs::msg::VehicleTorqueSetpoint tq{};
  tq.timestamp = t;
  tq.timestamp_sample = t;
  tq.xyz[0] = static_cast<float>(torque.x());
  tq.xyz[1] = static_cast<float>(torque.y());
  tq.xyz[2] = static_cast<float>(torque.z());
  torque_pub_->publish(tq);
}

void Px4ControlNode::publishNeutralSetpoints() {
  publishThrustTorque(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void Px4ControlNode::publishDiagnostics(const ControllerInputs &in,
                                        const ControllerOutputs &out) {
  px4_control::msg::PDDiagnostics d{};
  d.header.stamp = now();
  d.header.frame_id = "ned";
  auto fill = [](std::array<double, 3> &dst, const Eigen::Vector3d &src) {
    dst[0] = src.x();
    dst[1] = src.y();
    dst[2] = src.z();
  };
  fill(d.position, in.p);
  fill(d.velocity, in.v);
  fill(d.position_target, in.p_des);
  d.yaw_target = in.yaw_des;
  double r, p, y;
  quatToRpy(in.q, r, p, y);
  d.yaw = y;
  fill(d.position_error, out.position_error);
  fill(d.velocity_error, out.velocity_error);
  fill(d.attitude_error, out.attitude_error);
  fill(d.rate_error, out.rate_error);
  fill(d.desired_accel, out.desired_accel);
  fill(d.thrust_body, out.thrust_body);
  fill(d.torque_body, out.torque_body);
  diag_pub_->publish(d);
}

void Px4ControlNode::sendVehicleCommand(uint32_t command, float p1, float p2,
                                        float p3, float p4, double p5, double p6,
                                        float p7) {
  px4_msgs::msg::VehicleCommand c{};
  c.timestamp = nowUs();
  c.command = command;
  c.param1 = p1;
  c.param2 = p2;
  c.param3 = p3;
  c.param4 = p4;
  c.param5 = p5;
  c.param6 = p6;
  c.param7 = p7;
  c.target_system = 1;
  c.target_component = 1;
  c.source_system = 255;
  c.source_component = 1;
  c.from_external = true;
  command_pub_->publish(c);
}

void Px4ControlNode::requestOffboardMode() {
  // VEHICLE_CMD_DO_SET_MODE: base_mode=1 (custom), custom_main_mode=6 (OFFBOARD).
  sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                     1.0f, 6.0f, 0.0f);
}

void Px4ControlNode::requestArm() {
  sendVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void Px4ControlNode::requestDisarm() {
  // param2 = 21196 = force-disarm magic (per MAVLink spec).
  sendVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f,
      21196.0f);
}

void Px4ControlNode::requestTakeoff(float altitude_agl) {
  // Without GPS coordinates PX4 will use NAN -> current position.
  const float nan = std::nanf("");
  px4_msgs::msg::VehicleCommand c{};
  c.timestamp = nowUs();
  c.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  c.param1 = nan;
  c.param2 = nan;
  c.param3 = nan;
  c.param4 = nan;  // yaw
  c.param5 = std::nan("");
  c.param6 = std::nan("");
  c.param7 = altitude_agl;  // relative altitude (PX4 takes AGL if no GPS frame)
  c.target_system = 1;
  c.target_component = 1;
  c.source_system = 255;
  c.source_component = 1;
  c.from_external = true;
  command_pub_->publish(c);
}

void Px4ControlNode::requestLand() {
  sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

void Px4ControlNode::statePublishTick() {
  px4_control::msg::DroneState s{};
  s.header.stamp = now();
  s.header.frame_id = "ned";

  if (last_odom_) {
    s.x = last_odom_->position[0];
    s.y = last_odom_->position[1];
    s.z = last_odom_->position[2];
    Eigen::Quaterniond q = px4QuatToEigen(last_odom_->q);
    double r, p, y;
    quatToRpy(q, r, p, y);
    s.roll = r;
    s.pitch = p;
    s.yaw = y;
    s.wx = last_odom_->angular_velocity[0];
    s.wy = last_odom_->angular_velocity[1];
    s.wz = last_odom_->angular_velocity[2];
  }
  if (last_battery_) {
    s.battery_voltage = last_battery_->voltage_v;
    s.battery_remaining = last_battery_->remaining;
    s.battery_warning = last_battery_->warning;
  }
  if (last_status_) {
    s.arming_state = last_status_->arming_state;
    s.nav_state = last_status_->nav_state;
    s.failsafe = last_status_->failsafe;
    s.offboard_active = (last_status_->nav_state ==
                         px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
  }
  s.goto_active = (flight_state_ == FlightState::GOTO ||
                   flight_state_ == FlightState::HOLD);
  s.goto_x = goto_target_.x();
  s.goto_y = goto_target_.y();
  s.goto_z = goto_target_.z();
  s.goto_yaw = goto_yaw_;
  state_pub_->publish(s);
}

}  // namespace px4_control
