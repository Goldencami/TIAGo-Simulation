import os
import yaml
import subprocess
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

# run xacro and return the robot description as string
def load_xacro(path, **kwargs):
    cmd = ['xacro', path] + [f'{k}:={v}' for k, v in kwargs.items()]
    return subprocess.check_output(cmd).decode('utf-8')

def generate_launch_description():
    package_dir = get_package_share_directory('tiago_sim_control')
    tiago_desc_pkg = get_package_share_directory('tiago_description')
    tiago_moveit_pkg = get_package_share_directory('tiago_moveit_config')

    urdf_xacro = os.path.join(tiago_desc_pkg, 'robots', 'tiago.urdf.xacro')
    srdf_xacro = os.path.join(tiago_moveit_pkg, 'config', 'srdf', 'tiago.srdf.xacro')

    controllers_yaml_file = os.path.join(tiago_moveit_pkg, 'config', 'controllers', 'controllers_webots.yaml')
    with open(controllers_yaml_file, 'r') as f:
        controllers_yaml = yaml.safe_load(f)

    robot_description = load_xacro(
        urdf_xacro,
        base_type='pmb2',
        arm_type='tiago-arm',
        ft_sensor='schunk-ft',
        end_effector='pal-gripper'
    )

    robot_description_semantic = load_xacro(
        srdf_xacro,
        base_type='pmb2',
        arm_type='tiago-arm',
        ft_sensor='schunk-ft',
        end_effector='pal-gripper'
    )

    # custom action that allows you to start a Webots simulation instance
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'tiago_test.wbt'),
        ros2_supervisor=True
    )

    # interface that connects the controller plugin to the target robot
    my_robot_driver = WebotsController(
        robot_name='TIAGo',
        parameters=[{
            'robot_description': urdf_xacro,
            'update_rate': 31,
        }],
        remappings=[('/cmd_vel', '/cmd_vel')],
        respawn=True,
    )

    tiago_base_driver = Node(
        package='tiago_sim_control',
        executable='tiago_base_driver',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }]
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=my_robot_driver,
        nodes_to_start=[tiago_base_driver]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # move group node from MoveIt2
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': True},
            controllers_yaml
        ]
    )

    # This action will kill all nodes once the Webots simulation has exited
    return LaunchDescription([
        webots,
        my_robot_driver,
        waiting_nodes,
        robot_state_publisher,
        move_group,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])