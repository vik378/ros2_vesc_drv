from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_vesc_drv",
                executable="vesc_diff_drv",
                name="vesc_diff_drv",
            ),
            Node(package="joy", executable="joy_node", name="joy_node"),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[
                    {"axis_angular": 2},
                    {"axis_linear": {"x": 3}},
                    {"require_enable_button": False},
                    {"scale_linear": {"x": 0.1}},
                    {"scale_angular": {"yaw": 0.1}},
                ],
            ),
            Node(
                package="ros2_vesc_drv",
                executable="cmdv_mapper",
                name="vesc_diff_cmdv_mapper",
            ),
        ]
    )
