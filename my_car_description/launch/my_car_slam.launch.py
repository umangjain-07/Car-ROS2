from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arg for sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to your saved map
    map_file = '/home/umang/ros2_ws/src/car/my_car_description/maps/car_map.yaml'

    # Path to Nav2 bringup's navigation launch
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated clock'
        ),

        # SLAM Toolbox in localization mode
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'localization',
                'map_file': map_file
            }],
        ),

        # Include Nav2 bringup navigation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
