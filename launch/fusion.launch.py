import os
from launch import LaunchDescription
from launch_ros.actions import Node

CFG = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "config", "ekf_lio_wheel.yaml")


def generate_launch_description():
    return LaunchDescription([
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             output="screen", parameters=[CFG],
             remappings=[("odometry/filtered", "/fusion/odom")]),
    ])
