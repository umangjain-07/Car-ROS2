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
    map_file = '/home/umang/ros2_ws/src/car/my_car_description/maps/my_car_map.yaml'

    # Path to Nav2 bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    # Path to your custom nav2 params
    description_pkg = get_package_share_directory('my_car_description')
    nav2_params = os.path.join(description_pkg, 'config', 'nav2_params.yaml')

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

        # Include Nav2 navigation with your custom params
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # 'params_file': nav2_params
            }.items(),
        ),
    ])
