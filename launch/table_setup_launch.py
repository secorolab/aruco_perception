import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_config(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def _create_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    config = load_config(config_path)

    camera = config['camera']
    world_marker = config['world_marker']
    objects = config['objects']

    world_params = {
        'image_topic': camera['image_topic'],
        'camera_info_topic': camera['camera_info_topic'],
        'marker_dict': world_marker['marker_dict'],
        'marker_size': world_marker['marker_size'],
        'marker_id': world_marker['marker_id'],
    }

    objects_params = {
        'image_topic': camera['image_topic'],
        'camera_info_topic': camera['camera_info_topic'],
        'marker_dict': objects['marker_dict'],
        'marker_size': objects['marker_size'],
        'world_marker_id': world_marker['marker_id'],
    }

    return [
        Node(
            package='aruco_perception',
            executable='world_pose_node',
            name='world_pose_node',
            parameters=[world_params],
            output='screen',
        ),
        Node(
            package='aruco_perception',
            executable='detect_objects_node',
            name='detect_objects_node',
            parameters=[objects_params],
            output='screen',
        ),
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            description='Path to the configuration file'
        ),
        OpaqueFunction(function=_create_nodes),
    ])
