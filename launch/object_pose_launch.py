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

    camera = config['arm_camera']
    world_marker = config['world_marker']
    objects = config['objects']

    objects_params = {
        'image_topic': camera['image_topic'],
        'camera_info_topic': camera['camera_info_topic'],
        'marker_dict': objects['marker_dict'],
        'marker_size': objects['marker_size'],
        'world_marker_id': world_marker['marker_id'],
        'table_anchor_frame': world_marker['marker_frame'],
        'result_frame': world_marker['marker_frame'],
        'scene_file': config['scene']['file_path'],
    }

    kinova_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kinova_vision'),
                'launch',
                'kinova_vision.launch.py',
            )
        ),
        launch_arguments={'device': camera['ip']}.items(),
    )

    return [
        kinova_vision_launch,
        Node(
            package='aruco_perception',
            executable='object_pose_node',
            name='object_pose_node',
            parameters=[objects_params],
            output='screen',
        ),
        Node(
            package='aruco_perception',
            executable='handle_pose_node',
            name='handle_pose_node',
            output='screen',
        )
    ]


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory('aruco_perception'), 'config', 'table_setup.yml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config_path,
            description='Path to the configuration file'
        ),
        OpaqueFunction(function=_create_nodes),
    ])
