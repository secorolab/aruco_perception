from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Time
from scipy.spatial.transform import RigidTransform

from aruco_perception.object_pose_node import transform_stamped

from aruco_perception.utils import (
    detection_msg,
    expand_iri,
    frame_by_name,
    load_scene_graph,
    load_setup,
    transform_between_frames,
)


def test_detection_pose_is_available_to_both_supported_consumers():
    detection = detection_msg(
        'urn:test:object',
        'base_link',
        Time(sec=1),
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
    )

    assert detection.results[0].pose.pose == detection.bbox.center
    assert detection.bbox.center.position.x == 1.0


def test_rdf_frame_pose_converts_to_static_tf():
    matrix = np.eye(4)
    matrix[0:3, 3] = [1.0, 2.0, 3.0]
    msg = transform_stamped(
        'table_anchor', 'robot_table_top', Time(sec=1), RigidTransform.from_matrix(matrix)
    )

    assert msg.header.frame_id == 'table_anchor'
    assert msg.child_frame_id == 'robot_table_top'
    assert msg.transform.translation.z == 3.0
    assert msg.transform.rotation.w == 1.0


def test_collab_model_frames_resolve_in_the_configured_reference_frame():
    config = load_setup(Path(__file__).parents[1] / 'config' / 'table_setup.yml')
    graph = load_scene_graph(config)
    ref = expand_iri(graph, frame_by_name(config, config['ref_frame'])['iri'])

    robot_table = transform_between_frames(
        graph,
        expand_iri(graph, frame_by_name(config, 'robot_table_top')['iri']),
        ref,
    )
    rk_table = transform_between_frames(
        graph,
        expand_iri(graph, frame_by_name(config, 'rk_table_top')['iri']),
        ref,
    )

    assert np.allclose(robot_table.translation, [0.295, 0.725, 0.0])
    assert np.allclose(rk_table.translation, [-0.75, 1.87, 0.05])
    assert {
        str(expand_iri(graph, configured_object['iri']))
        for configured_object in config['objects']
    } == {
        'https://secorolab.github.io/models/demos/collab/scene/drawer',
        'https://secorolab.github.io/models/demos/collab/scene/robot-ws',
        'https://secorolab.github.io/models/demos/collab/scene/wall-ws',
    }
