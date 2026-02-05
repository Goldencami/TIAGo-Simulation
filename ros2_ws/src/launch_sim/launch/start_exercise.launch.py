from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tiago_lift_control_node = Node(
        package='tiago_arm_control',
        executable='lift_arm_control',
        name='lift_arm_control',
        output='screen'
    )

    tiago_retract_control_node = Node(
        package='tiago_arm_control',
        executable='retract_arm_control',
        name='retract_arm_control',
        output='screen'
    )

    tiago_gripper_control_node = Node(
        package='tiago_arm_control',
        executable='gripper_control',
        name='gripper_control',
        output='screen'
    )

    tiago_base_node = Node(
        package='tiago_base_control',
        executable='base_control',
        name='base_control',
        output='screen'
    )

    return LaunchDescription([
        tiago_lift_control_node,
        tiago_retract_control_node,
        tiago_gripper_control_node,
        tiago_base_node
    ])
