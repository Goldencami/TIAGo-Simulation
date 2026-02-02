import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='sim_world',
        description='Name of the world in my package/worlds (without .world)'
    )

    def spawn_tiago(context, *args, **kwargs):
        world_name = LaunchConfiguration('world_name').perform(context)
        world_path = os.path.join(
            get_package_share_directory('my_tiago_sim'),
            'worlds',
            world_name + '.world'
        )

        # Start Gazebo server
        gzserver = ExecuteProcess(
            cmd=['gzserver', world_path, '--verbose'],
            output='screen'
        )

        # Start Gazebo client
        gzclient = ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        )

        # Spawn TIAGo URDF
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'tiago',
                '-file', os.path.join(get_package_share_directory('tiago_robot'),
                                      'robots', 'tiago.urdf.xacro'),
                '-x', '0', '-y', '0', '-z', '0.0'
            ],
            output='screen'
        )

        return [gzserver, gzclient, spawn_robot]

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=spawn_tiago)
    ])
