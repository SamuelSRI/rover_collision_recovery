from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory(
        "rover_collision_recovery"
    )
    params_file = os.path.join(
        package_share,
        "config",
        "collision_recovery.yaml",
    )

    return LaunchDescription([
        Node(
            package="rover_collision_recovery",
            executable="collision_recovery_node",
            name="collision_recovery",
            output="screen",
            parameters=[params_file],
        )
    ])
