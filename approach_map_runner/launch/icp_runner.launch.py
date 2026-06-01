import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_runner_share = get_package_share_directory("approach_map_runner")

    use_sim_time = LaunchConfiguration("use_sim_time")
    icp_config = LaunchConfiguration("icp_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for the ICP runner node.",
        ),
        DeclareLaunchArgument(
            "icp_config",
            default_value=os.path.join(map_runner_share, "config", "icp_config.yaml"),
            description="ROS parameter file for node_icp_cuda.",
        ),
        Node(
            package="approach_map_runner",
            executable="node_icp_cuda",
            output="screen",
            parameters=[
                icp_config,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
