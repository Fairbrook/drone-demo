from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("kp_pos", default_value="[1.5, 1.5, 4.0]"),
        DeclareLaunchArgument("kd_pos", default_value="[1.2, 1.2, 2.5]"),
        DeclareLaunchArgument("kp_att", default_value="[12.0, 12.0, 4.0]"),
        DeclareLaunchArgument("kd_att", default_value="[1.6, 1.6, 0.8]"),
        DeclareLaunchArgument("inertia", default_value="[0.029, 0.029, 0.055]"),
        DeclareLaunchArgument("krmax", default_value="5000.0"),
        DeclareLaunchArgument("max_torque", default_value="[0.5, 0.5, 0.2]"),
        DeclareLaunchArgument("hover_thrust", default_value="1.0"),
        DeclareLaunchArgument("max_tilt_rad", default_value="0.5"),
        DeclareLaunchArgument("max_accel_xy", default_value="6.0"),
    ]

    node = Node(
        package="px4_control",
        executable="px4_control_node",
        name="px4_control_node",
        output="screen",
        parameters=[{
            "kp_pos": LaunchConfiguration("kp_pos"),
            "kd_pos": LaunchConfiguration("kd_pos"),
            "kp_att": LaunchConfiguration("kp_att"),
            "kd_att": LaunchConfiguration("kd_att"),
            "inertia": LaunchConfiguration("inertia"),
            "krmax": LaunchConfiguration("krmax"),
            "max_torque": LaunchConfiguration("max_torque"),
            "hover_thrust": LaunchConfiguration("hover_thrust"),
            "max_tilt_rad": LaunchConfiguration("max_tilt_rad"),
            "max_accel_xy": LaunchConfiguration("max_accel_xy"),
        }],
    )

    return LaunchDescription([*args, node])
