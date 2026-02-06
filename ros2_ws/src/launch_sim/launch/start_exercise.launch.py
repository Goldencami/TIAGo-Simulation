from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tiago_controllers = Node(
        package='tiago_arm_control',
        executable='TIAGOController',
        name='TIAGOController',
        output='screen'
    )

    tiago_base_node = Node(
        package='tiago_base_control',
        executable='base_control',
        name='base_control',
        output='screen'
    )

    return LaunchDescription([
        tiago_controllers,
        tiago_base_node
    ])
