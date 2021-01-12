from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="joy", executable="joy_node", name="joy_node"),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[
                    {"axis_angular": 2},
                    {"axis_linear": {"x": 3}},
                    {"require_enable_button": False},
                    {"scale_linear": {"x": 0.2}},
                    {"scale_angular": {"yaw": 0.2}},
                ],
            ),
        ]
    )
