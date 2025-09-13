from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("my_car_description")

    rviz_config_file = os.path.join(pkg_share, "rviz", "my_car.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
        
    ])
