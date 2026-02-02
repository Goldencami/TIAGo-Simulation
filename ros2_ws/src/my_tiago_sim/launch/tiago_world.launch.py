import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_pal.include_utils import include_scoped_launch_py_description

def generate_launch_description():
    # Path to your package
    pkg_dir = get_package_share_directory('my_tiago_sim')
    world_path = os.path.join(pkg_dir, 'worlds', 'sim_world.world')

    # Declare a launch argument for flexibility
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to the Gazebo world file'
    )

    # Include the TIAGo Gazebo launch file
    tiago_launch = include_scoped_launch_py_description(
        os.path.join(
            get_package_share_directory('tiago_gazebo'),
            'launch',
            'tiago_gazebo.launch.py'
        ),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': LaunchConfiguration('world')
        }.items()
    )

    return LaunchDescription([
        world_arg,
        tiago_launch
    ])
