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
            Node(
                package="ros2_vesc_drv",
                executable="cmdv_mapper",
                name="vesc_diff_cmdv_mapper",
            ),
        ]
    )
