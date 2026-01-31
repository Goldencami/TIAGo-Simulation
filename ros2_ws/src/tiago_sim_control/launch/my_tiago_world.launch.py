import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    package_dir = get_package_share_directory('tiago_sim_control')
    tiago_webots_pkg = get_package_share_directory('webots_ros2_tiago')

    tiago_moveit_pkg = get_package_share_directory('tiago_moveit_config')
    tiago_description_pkg = get_package_share_directory('tiago_description')
    robot_description_path = os.path.join(tiago_webots_pkg, 'resource', 'tiago_webots.urdf')

    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    # custom action that allows you to start a Webots simulation instance
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'tiago_test.wbt'),
        ros2_supervisor=True
    )

    # interface that connects the controller plugin to the target robot
    my_robot_driver = WebotsController(
        robot_name='TIAGo',
        parameters=[{
            'robot_description': robot_description,
            'update_rate': 31 
        }],
        remappings=[('/cmd_vel', '/cmd_vel')],
        respawn=True
    )

    tiago_base_driver = Node(
        package='tiago_sim_control',
        executable='tiago_base_driver',
        output='screen'
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=[tiago_base_driver]
    )

    # move group node from MoveIt2
    move_group = Node(
        package='tiago_moveit_config',
        executable='move_group',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # This action will kill all nodes once the Webots simulation has exited
    return LaunchDescription([
        webots,
        my_robot_driver,
        waiting_nodes,
        move_group,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])