# cd ~/ros2_farm_ws
# ros2 launch farm_sim farm.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farm_sim',
            executable='environment',
            name='environment_node',
            output='screen'
        ),
        Node(
            package='farm_sim',
            executable='robot',
            name='robot_node',
            output='screen'
        ),
        Node(
            package='farm_sim',
            executable='visualizer',
            name='visualizer_node',
            output='screen'
        ),
        Node(
            package='farm_sim',
            executable='rviz_publisher_node',
            name='rviz_publisher_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/busra/ros2_farm_ws/src/farm_sim/farm_robot.rviz'],
            output='screen'
        ),
    ])