from turtle import screensize
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    car_screen_node = Node(
        package="articubot_one_ui",
        executable="ui_node",
        name="car_screen_node",
        output="screen",
        # parameters=[{'cmd_vel':'/cmd_vel'}],
    )
    ld.add_action(car_screen_node)
    return ld