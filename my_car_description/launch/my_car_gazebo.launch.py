from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("my_car_description")

    # World file
    world_file = os.path.join(pkg_share, "worlds", "my_world.world")

    # Process URDF
    xacro_file = os.path.join(pkg_share, "urdf", "my_car.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true"
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),

        # Push robot_description to parameter server
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": LaunchConfiguration("use_sim_time")}]
        ),

        Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher",
    parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    output="screen"
),


        # Spawn robot in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "my_car"],
            output="screen"
        ),
    ])
