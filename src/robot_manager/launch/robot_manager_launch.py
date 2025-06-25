import os
from launch import LaunchDescription
from launch_ros.actions import Node  # 注意这里的导入路径

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_manager',  # 指定 ROS 2 包名
            executable='motor_manager',  # 可执行文件名
            name='motor_manager_node',  # 节点名称
            output='screen',  # 输出到屏幕
        ),
        
        Node(
            package='joy',  # 指定 ROS 2 包名
            executable='joy_node',  # 可执行文件名
            name='joystick_node',  # 节点名称
            output='screen',  # 输出到屏幕
        ),
    ])

