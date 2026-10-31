import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_runner_share = get_package_share_directory("approach_map_runner")
    mapping_share = get_package_share_directory("approach_mapping")
    cost_share = get_package_share_directory("approach_cost")

    use_sim_time = LaunchConfiguration("use_sim_time")
    runner_config = LaunchConfiguration("runner_config")
    cost_runner_config = LaunchConfiguration("cost_runner_config")
    map_config = LaunchConfiguration("map_config")
    cost_config = LaunchConfiguration("cost_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for all map runner nodes.",
        ),
        DeclareLaunchArgument(
            "runner_config",
            default_value=os.path.join(map_runner_share, "config", "runner_config.yaml"),
            description="ROS parameter file for approach_map_runner_node.",
        ),
        DeclareLaunchArgument(
            "cost_runner_config",
            default_value=os.path.join(map_runner_share, "config", "cost_runner_config.yaml"),
            description="ROS parameter file for approach_cost_runner_node.",
        ),
        DeclareLaunchArgument(
            "map_config",
            default_value=os.path.join(mapping_share, "config", "map_config.yaml"),
            description="Algorithm configuration file for approach_mapping.",
        ),
        DeclareLaunchArgument(
            "cost_config",
            default_value=os.path.join(cost_share, "config", "cost_config.yaml"),
            description="Algorithm configuration file for approach_cost.",
        ),
        Node(
            package="approach_map_runner",
            executable="approach_map_runner_node",
            output="screen",
            parameters=[
                runner_config,
                {
                    "use_sim_time": use_sim_time,
                    "map_config_path": map_config,
                },
            ],
        ),
        Node(
            package="approach_map_runner",
            executable="approach_cost_runner_node",
            output="screen",
            parameters=[
                cost_runner_config,
                {
                    "use_sim_time": use_sim_time,
                    "cost_config_path": cost_config,
                },
            ],
        ),
        # Node(
        #     package="approach_map_runner",
        #     executable="approach_planning_runner_node",
        #     output="screen",
        #     parameters=[{"use_sim_time": use_sim_time}],
        # ),
    ])
