from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robotic_bird",
            executable="flight_controller",
            name="flight_controller",
            output="screen"),
        Node(
            package="robotic_bird",
            executable="navigator",
            name="navigator",
            output="screen"),
    ])