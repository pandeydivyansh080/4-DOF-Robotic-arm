"""
Starts robot_state_publisher (reads arm.urdf) and RViz.

Real arm (also starts the serial bridge):
    ros2 launch arm_view.launch.py port:=COM3

Demo, no hardware. The launch only opens RViz; run the keyboard-controlled
bridge yourself in a second ROS 2 window:
    ros2 launch arm_view.launch.py demo:=true
    python arm_serial_bridge.py --demo
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    with open(os.path.join(HERE, "arm.urdf"), "r") as f:
        urdf = f.read()

    bridge = os.path.join(HERE, "arm_serial_bridge.py")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="COM3"),
        DeclareLaunchArgument("demo", default_value="false"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),

        Node(package="rviz2", executable="rviz2"),

        ExecuteProcess(
            cmd=["python", bridge, "--port", LaunchConfiguration("port")],
            output="screen", condition=UnlessCondition(LaunchConfiguration("demo"))),

        LogInfo(
            msg="Demo mode: now open a second ROS 2 window and run: "
                "python arm_serial_bridge.py --demo",
            condition=IfCondition(LaunchConfiguration("demo"))),
    ])
