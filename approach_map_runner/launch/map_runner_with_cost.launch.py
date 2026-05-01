from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for all map runner nodes.",
        ),
        Node(
            package="approach_map_runner",
            executable="approach_map_runner_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="approach_map_runner",
            executable="approach_cost_runner_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="approach_map_runner",
            executable="approach_waiting",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # Node(
        #     package="approach_map_runner",
        #     executable="approach_planning_runner_node",
        #     output="screen",
        #     parameters=[{"use_sim_time": use_sim_time}],
        # ),
    ])
