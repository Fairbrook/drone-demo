import math
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_control.msg import DroneCommand, DroneState

from px4_tui.app import TuiApp


NAV_STATE_NAMES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    6: "POSITION_SLOW",
    8: "ALT_CRUISE",
    10: "ACRO",
    12: "DESCEND",
    13: "TERMINATION",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
    19: "AUTO_FOLLOW",
    20: "AUTO_PRECLAND",
    21: "ORBIT",
    22: "AUTO_VTOL_TAKEOFF",
}

ARMING_STATE_NAMES = {1: "DISARMED", 2: "ARMED"}

BATTERY_WARNING_NAMES = {
    0: "NONE",
    1: "LOW",
    2: "CRITICAL",
    3: "EMERGENCY",
    4: "FAILED",
}


class TuiNode(Node):
    """rclpy node that owns the DroneState subscription and DroneCommand publisher."""

    def __init__(self):
        super().__init__("px4_tui")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub = self.create_publisher(DroneCommand, "/tui/cmd", qos)
        self.state_sub = self.create_subscription(
            DroneState, "/tui/state", self._on_state, qos
        )
        self.latest_state: DroneState | None = None

    def _on_state(self, msg: DroneState) -> None:
        self.latest_state = msg

    def send_takeoff(self, altitude_agl: float) -> None:
        msg = DroneCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kind = DroneCommand.KIND_TAKEOFF
        msg.takeoff_altitude = float(altitude_agl)
        self.cmd_pub.publish(msg)

    def send_land(self) -> None:
        msg = DroneCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kind = DroneCommand.KIND_LAND
        self.cmd_pub.publish(msg)

    def send_emergency(self) -> None:
        msg = DroneCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kind = DroneCommand.KIND_EMERGENCY
        self.cmd_pub.publish(msg)

    def send_hold(self) -> None:
        msg = DroneCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kind = DroneCommand.KIND_HOLD
        self.cmd_pub.publish(msg)

    def send_goto(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        msg = DroneCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.kind = DroneCommand.KIND_GOTO
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = math.radians(float(yaw_deg))
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TuiNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = TuiApp(node=node, nav_states=NAV_STATE_NAMES,
                 arming_states=ARMING_STATE_NAMES,
                 battery_warnings=BATTERY_WARNING_NAMES)
    try:
        app.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
