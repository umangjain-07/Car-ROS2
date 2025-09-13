from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg = get_package_share_directory("my_car_description")

    gazebo_launch = os.path.join(description_pkg, "launch", "my_car_gazebo.launch.py")
    display_launch = os.path.join(description_pkg, "launch", "display.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
        ),
    ])
