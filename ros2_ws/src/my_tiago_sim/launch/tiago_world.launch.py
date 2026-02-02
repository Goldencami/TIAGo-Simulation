import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_pal.include_utils import include_scoped_launch_py_description

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_tiago_sim')
    world_path = os.path.join(pkg_dir, 'worlds', 'sim_world.sdf')

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value=world_path,
        description='Path to the Ignition world file'
    )

    # launch Ignition Gazebo in VM
    # ign_gazebo_cmd = ExecuteProcess(
    #     cmd=[
    #         'ign', 'gazebo',
    #         LaunchConfiguration('world'),
    #         '-r',
    #         '-v', '4',
    #         '--render-engine', 'ogre'
    #     ],
    #     output='screen'
    # )

    # spawn_tiago = Node(
    #     package='ros_ign_gazebo',
    #     executable='create',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-name', 'tiago',
    #         '-x', '3', '-y', '3', '-z', '0.0'
    #     ],
    #     output='screen'
    # )

    # Include the official Tiago Gazebo launch
    tiago_gazebo_launch = include_scoped_launch_py_description(
        pkg_name='tiago_gazebo',
        paths=['launch', 'tiago_gazebo.launch.py'],
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'navigation': 'False',
            'moveit': 'False',
            'rviz': 'False',
            'gzclient': 'True',
            'is_public_sim': 'False'
        }
    )

    return LaunchDescription([
        world_arg,
        tiago_gazebo_launch
    ])