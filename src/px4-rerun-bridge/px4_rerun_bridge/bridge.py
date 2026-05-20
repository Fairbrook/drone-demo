"""Bridges px4_control diagnostics + state into the Rerun viewer.

Subscribes:
    /px4_control/pd_diagnostics  (px4_control/PDDiagnostics)  - 200 Hz when active
    /tui/state                   (px4_control/DroneState)     - 10 Hz

Run:
    # spawns the viewer locally
    ros2 run px4_rerun_bridge rerun_bridge
    ros2 run px4_rerun_bridge rerun_bridge --ros-args -p save_path:=/tmp/flight.rrd
                                                            # writes a .rrd file instead
"""
import math

import rclpy
import rerun as rr
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_control.msg import DroneState, PDDiagnostics


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RerunBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_rerun_bridge")

        # Parameters.
        self.declare_parameter("app_id", "px4_control")
        self.declare_parameter("spawn_viewer", True)
        self.declare_parameter("save_path", "")
        app_id = self.get_parameter(
            "app_id").get_parameter_value().string_value
        spawn = self.get_parameter(
            "spawn_viewer").get_parameter_value().bool_value
        save_path = self.get_parameter(
            "save_path").get_parameter_value().string_value

        # Initialize Rerun. If save_path is set we record to file instead of opening a viewer.
        if save_path:
            rr.init(app_id, spawn=False)
            rr.save(save_path)
            self.get_logger().info(f"Rerun recording to {save_path}")
        else:
            rr.init(app_id, spawn=spawn)
            self.get_logger().info(
                "Rerun viewer " +
                ("spawned" if spawn else "init only — connect manually")
            )

        # NED world view: X=North forward, Y=East right, Z=Down. FRD matches.
        rr.log("world", rr.ViewCoordinates.FRD, static=True)

        # Static markers for the world origin so the viewer has scale.
        rr.log(
            "world/origin",
            rr.Points3D([[0.0, 0.0, 0.0]], radii=[0.05],
                        colors=[[150, 150, 150]]),
            static=True,
        )

        # QoS — diagnostics are BEST_EFFORT from the controller, DroneState is RELIABLE.
        be_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        rel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            PDDiagnostics, "/px4_control/pd_diagnostics", self._on_diag, be_qos
        )
        self.create_subscription(
            DroneState, "/tui/state", self._on_state, rel_qos)

        self.get_logger().info("px4_rerun_bridge started")

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_diag(self, msg: PDDiagnostics) -> None:
        #        rr.set_time_seconds("ros_time", _stamp_to_seconds(msg.header.stamp))

        # 3D scene: drone + target + desired-accel arrow.
        p = list(msg.position)
        p_tgt = list(msg.position_target)
        a_des = list(msg.desired_accel)

        rr.log(
            "world/drone",
            rr.Points3D([p], radii=[0.12], colors=[
                        [60, 200, 80]], labels=["drone"]),
        )
        rr.log(
            "world/target",
            rr.Points3D([p_tgt], radii=[0.12], colors=[
                        [230, 60, 60]], labels=["target"]),
        )
        rr.log(
            "world/path",
            rr.LineStrips3D([[p, p_tgt]], colors=[[140, 140, 140]]),
        )
        # Scale desired accel for visualization (clamp to 5 m visual length).
        a_norm = math.sqrt(sum(c * c for c in a_des))
        if a_norm > 1e-3:
            scale = min(1.0, 5.0 / a_norm)
            arrow = [c * scale for c in a_des]
            rr.log(
                "world/drone/desired_accel",
                rr.Arrows3D(vectors=[arrow], origins=[
                            p], colors=[[80, 160, 240]]),
            )

        # Per-axis scalar streams. Group under one entity each so Rerun
        # auto-plots the three components together.
        self._log_vec3("errors/position", msg.position_error, ("x", "y", "z"))
        self._log_vec3("errors/velocity", msg.velocity_error, ("x", "y", "z"))
        self._log_vec3("errors/attitude", msg.attitude_error,
                       ("roll", "pitch", "yaw"))
        self._log_vec3("errors/rate", msg.rate_error, ("p", "q", "r"))
        self._log_vec3("cmd/thrust", msg.thrust_body, ("x", "y", "z"))
        self._log_vec3("cmd/torque", msg.torque_body, ("roll", "pitch", "yaw"))
        self._log_vec3("ref/desired_accel", msg.desired_accel, ("x", "y", "z"))

        # Yaw vs yaw target as degrees on the same plot.
        rr.log("ref/yaw/measured", rr.Scalars([math.degrees(msg.yaw)]))
        rr.log("ref/yaw/target", rr.Scalars([math.degrees(msg.yaw_target)]))

        # Position vs target per axis (for tracking-error plots).
        for i, axis in enumerate(("x", "y", "z")):
            rr.log(f"ref/position_{axis}/measured",
                   rr.Scalars([float(msg.position[i])]))
            rr.log(f"ref/position_{axis}/target",
                   rr.Scalars([float(msg.position_target[i])]))

    def _on_state(self, msg: DroneState) -> None:
        # rr.set_time_seconds("ros_time", _stamp_to_seconds(msg.header.stamp))

        # Battery + arming + mode as time-aligned scalars / text.
        rr.log("status/battery_voltage_v",
               rr.Scalars([float(msg.battery_voltage)]))
        if msg.battery_remaining >= 0.0:
            rr.log(
                "status/battery_remaining_pct",
                rr.Scalars([float(msg.battery_remaining) * 100.0]),
            )
        rr.log("status/armed",
               rr.Scalars([1.0 if msg.arming_state == 2 else 0.0]))
        rr.log("status/failsafe", rr.Scalars([1.0 if msg.failsafe else 0.0]))
        rr.log("status/offboard_active",
               rr.Scalars([1.0 if msg.offboard_active else 0.0]))
        rr.log("status/nav_state", rr.Scalars([float(msg.nav_state)]))
        rr.log(
            "status/mode_text",
            rr.TextLog(
                f"nav_state={msg.nav_state} arming={msg.arming_state} "
                f"failsafe={msg.failsafe} offboard={msg.offboard_active}"
            ),
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _log_vec3(path: str, vec, axis_names) -> None:
        for value, axis in zip(vec, axis_names):
            rr.log(f"{path}/{axis}", rr.Scalars([float(value)]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RerunBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
