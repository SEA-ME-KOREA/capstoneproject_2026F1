from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

import os


def generate_launch_description():
    limo_car_launch = os.path.join(
        get_package_share_directory('limo_car'),
        'launch',
        'ackermann.launch.py',
    )
    parking_world = PathJoinSubstitution([
        get_package_share_directory('limo_gazebosim'),
        'worlds',
        'parking_lot_scaled_vehicles.world',
    ])
    package_models = os.path.join(
        get_package_share_directory('limo_gazebosim'),
        'models',
    )
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py',
    )
    default_rviz_config = os.path.join(
        get_package_share_directory('limo_car'),
        'rviz',
        'gazebo.rviz',
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=default_rviz_config,
        description='Absolute path to the RViz config file.',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=parking_world,
        description='Full path to the Gazebo world file.',
    )
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            os.pathsep,
            os.path.expanduser('~/.gazebo/models'),
            os.pathsep,
            package_models,
        ],
    )
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(limo_car_launch),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        rviz_arg,
        world_arg,
        gazebo_model_path,
        robot_description,
        gazebo,
        spawn_entity,
        rviz_node,
    ])
