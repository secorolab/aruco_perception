import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_config(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def _create_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    config = load_config(config_path)

    camera = config['static_camera']
    world_marker = config['world_marker']

    world_params = {
        'image_topic': camera['image_topic'],
        'camera_info_topic': camera['camera_info_topic'],
        'table_anchor_frame': world_marker['frame'],
        'marker_dict': world_marker['marker_dict'],
        'marker_size': world_marker['marker_size'],
        'marker_id': world_marker['marker_id'],
        'world_iri': world_marker['iri'],
    }

    return [
        Node(
            package='aruco_perception',
            executable='world_pose_node',
            name='world_pose_node',
            parameters=[world_params],
            output='screen',
        ),
    ]


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('aruco_perception'), 'config', 'table_setup.yml'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to the configuration file'
        ),
        realsense_launch,
        OpaqueFunction(function=_create_nodes),
    ])
