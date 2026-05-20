#pragma once

#include <atomic>
#include <memory>
#include <optional>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include "px4_control/msg/drone_command.hpp"
#include "px4_control/msg/drone_state.hpp"
#include "px4_control/msg/pd_diagnostics.hpp"
#include "px4_control/pd_controller.hpp"

namespace px4_control {

enum class FlightState {
  IDLE,
  TAKING_OFF,
  HOLD,
  GOTO,
  LANDING,
  EMERGENCY,
};

class Px4ControlNode : public rclcpp::Node {
 public:
  Px4ControlNode();

 private:
  // Subscriber callbacks.
  void onOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void onCommand(const px4_control::msg::DroneCommand::SharedPtr msg);

  // Timers.
  void controlTick();
  void statePublishTick();

  // PX4 helpers.
  uint64_t nowUs();
  void publishOffboardControlMode();
  void publishThrustTorque(const Eigen::Vector3d &thrust,
                           const Eigen::Vector3d &torque);
  void publishNeutralSetpoints();
  void publishDiagnostics(const ControllerInputs &in, const ControllerOutputs &out);
  void sendVehicleCommand(uint32_t command, float p1 = 0.0f, float p2 = 0.0f,
                          float p3 = 0.0f, float p4 = 0.0f, double p5 = 0.0,
                          double p6 = 0.0, float p7 = 0.0f);
  void requestOffboardMode();
  void requestArm();
  void requestDisarm();
  void requestTakeoff(float altitude_agl);
  void requestLand();

  // Gain loading from ROS params.
  void loadParameters();

  // QoS profiles.
  rclcpp::QoS px4SubQos() const;
  rclcpp::QoS px4PubQos() const;

  // Subscribers.
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_control::msg::DroneCommand>::SharedPtr cmd_sub_;

  // Publishers.
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<px4_control::msg::DroneState>::SharedPtr state_pub_;
  rclcpp::Publisher<px4_control::msg::PDDiagnostics>::SharedPtr diag_pub_;

  // Timers.
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // Cached PX4 state.
  std::optional<px4_msgs::msg::VehicleOdometry> last_odom_;
  rclcpp::Time last_odom_stamp_;
  std::optional<px4_msgs::msg::VehicleStatus> last_status_;
  std::optional<px4_msgs::msg::BatteryStatus> last_battery_;
  std::optional<px4_msgs::msg::VehicleAttitude> last_attitude_;

  // Flight state machine.
  FlightState flight_state_ = FlightState::IDLE;
  Eigen::Vector3d goto_target_{0.0, 0.0, 0.0};
  double goto_yaw_ = 0.0;
  uint64_t offboard_warmup_ticks_ = 0;
  float takeoff_altitude_agl_ = 0.0f;
  bool saw_auto_takeoff_ = false;
  static constexpr uint64_t kOffboardWarmupTicks = 200;  // ~1 s @ 200 Hz
  static constexpr double kOdomStaleSec = 0.2;
  // Treat takeoff as complete once we are within this many meters of target AGL.
  static constexpr double kTakeoffAltitudeTolM = 0.3;

  // Controller.
  PDController pd_;
};

}  // namespace px4_control
