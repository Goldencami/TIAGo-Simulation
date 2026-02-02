import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    is_public_sim = LaunchConfiguration('is_public_sim')

    declare_is_public_sim = DeclareLaunchArgument(
        'is_public_sim',
        default_value='True',
        description="Required by PAL's public TIAGo simulation"
    )

    world = os.path.join(get_package_share_directory('my_tiago_sim'), 'worlds', 'sim_world.world')

    tiago_desc = get_package_share_directory('tiago_description')
    tiago_xacro_file = os.path.join(tiago_desc, 'robots', 'tiago.urdf.xacro')
    robot_description_raw = xacro.process_file(tiago_xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world, 'is_public_sim': is_public_sim}.items()
    )

    # Spawn TIAGo at the corner of the room
    spawn_tiago = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tiago'
            # '-x', '-2.0',
            # '-y', '-2.0',
            # '-Y', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_is_public_sim,
        gazebo,
        robot_state_publisher_node,
        spawn_tiago
    ])