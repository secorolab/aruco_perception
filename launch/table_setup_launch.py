import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_perception')
    default_config_path = os.path.join(pkg_share, 'config', 'table_setup.yml')

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Path to the configuration file'
    )
    launch_arguments = {'config_path': LaunchConfiguration('config_path')}.items()

    world_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'world_pose_launch.py')
        ),
        launch_arguments=launch_arguments,
    )

    object_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'object_pose_launch.py')
        ),
        launch_arguments=launch_arguments,
    )

    return LaunchDescription([
        config_path_arg,
        world_pose_launch,
        object_pose_launch,
    ])
