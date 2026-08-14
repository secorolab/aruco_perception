import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Maps the 'camera' launch argument to the matching section of table_setup.yml.
CAMERA_CONFIG_KEYS = {
    'static': 'static_camera',
    'arm': 'arm_camera',
}


def load_config(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def object_pose_params(config, camera):
    """Parameters for an object_pose_node instance watching `camera`."""
    world_marker = config['world_marker']
    objects = config['objects']
    return {
        'image_topic': camera['image_topic'],
        'camera_info_topic': camera['camera_info_topic'],
        'marker_dict': objects['marker_dict'],
        'marker_size': objects['marker_size'],
        'world_marker_id': world_marker['marker_id'],
        'table_anchor_frame': world_marker['frame'],
        'result_frame': world_marker['frame'],
        'scene_file': config['scene']['file_path'],
    }


def _offset_list(offset):
    return [offset['x'], offset['y'], offset['z'], offset['roll'], offset['pitch'], offset['yaw']]


def handle_pose_params(config):
    world_marker = config['world_marker']
    tray = config['tray']
    return {
        'table_anchor_frame': world_marker['frame'],
        'handle_frame': tray['handle_frame'],
        'scene_file': config['scene']['file_path'],
        'marker_front_id': tray['marker_front']['id'],
        'marker_front_offset': _offset_list(tray['marker_front']['offset']),
        'marker_back_id': tray['marker_back']['id'],
        'marker_back_offset': _offset_list(tray['marker_back']['offset']),
    }


def _create_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    config = load_config(config_path)

    camera_arg = LaunchConfiguration('camera').perform(context)
    camera_key = CAMERA_CONFIG_KEYS.get(camera_arg)
    if camera_key is None:
        raise ValueError(
            f"camera launch argument must be one of {sorted(CAMERA_CONFIG_KEYS)}, "
            f"got '{camera_arg}'"
        )
    camera = config[camera_key]

    return [
        Node(
            package='aruco_perception',
            executable='object_pose_node',
            name='object_pose_node',
            namespace=camera_key,
            parameters=[object_pose_params(config, camera)],
            output='screen',
        ),
        Node(
            package='aruco_perception',
            executable='handle_pose_node',
            name='handle_pose_node',
            parameters=[handle_pose_params(config)],
            output='screen',
        ),
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
        DeclareLaunchArgument(
            'camera',
            default_value='arm',
            description="Which camera to run object detection on: 'static' or 'arm'",
        ),
        OpaqueFunction(function=_create_nodes),
    ])
