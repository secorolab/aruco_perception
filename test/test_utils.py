from pathlib import Path
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime
from scipy.spatial.transform import RigidTransform

from aruco_perception.detect_objects_node import (
    observation_has_priority,
    transform_is_fresh,
)
from aruco_perception.object_pose_node import ObjectPoseNode, transform_stamped
from aruco_perception.utils import (
    anchored_sensor_specs,
    detection_msg,
    expand_iri,
    frame_by_name,
    load_scene_graph,
    load_setup,
    transform_between_frames,
)


def test_anchored_and_external_sensor_contract():
    config = load_setup(Path(__file__).parents[1] / "config" / "table_setup.yml")
    anchored = anchored_sensor_specs(config)

    assert set(anchored) == {"robot_table_camera", "rk_table_camera"}
    assert anchored["robot_table_camera"]["anchor_frame"] == "table_anchor"
    assert anchored["rk_table_camera"]["anchor_frame"] == "rk_anchor"
    assert len({sensor["camera_link_frame"] for sensor in anchored.values()}) == 2
    assert "arm_camera" not in anchored


def test_sensor_order_prioritizes_fresh_observations():
    sensors = {
        "static_camera": {"visible": {1}, "seen_at_ns": 1_000_000_000},
        "arm_camera": {"visible": {1}, "seen_at_ns": 1_200_000_000},
    }

    assert observation_has_priority(
        sensors, "static_camera", 1, 1_200_000_000, 500_000_000
    )
    assert not observation_has_priority(
        sensors, "arm_camera", 1, 1_200_000_000, 500_000_000
    )
    assert observation_has_priority(
        sensors, "arm_camera", 1, 1_600_000_000, 500_000_000
    )


def test_anchored_camera_transform_freshness_boundary():
    transform = transform_stamped(
        "table_anchor",
        "camera_link",
        Time(sec=1),
        RigidTransform.from_matrix(np.eye(4)),
    )

    assert transform_is_fresh(transform, RclpyTime(nanoseconds=1_500_000_000), 0.5)
    assert not transform_is_fresh(transform, RclpyTime(nanoseconds=1_500_000_001), 0.5)


def test_detection_pose_is_available_to_both_supported_consumers():
    detection = detection_msg(
        "urn:test:object",
        "base_link",
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
        "table_anchor",
        "robot_table_top",
        Time(sec=1),
        RigidTransform.from_matrix(matrix),
    )

    assert msg.header.frame_id == "table_anchor"
    assert msg.child_frame_id == "robot_table_top"
    assert msg.transform.translation.z == 3.0
    assert msg.transform.rotation.w == 1.0


def test_dynamic_object_is_published_as_detection_and_tf():
    pose = RigidTransform.from_matrix(np.eye(4))
    detections = []
    transforms = []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: RclpyTime(seconds=1.0)),
        table_anchor_frame="table_anchor",
        objects=[
            {
                "name": "drawer_handle_obj",
                "frame": "drawer_handle",
                "iri": "urn:test:drawer",
                "source": None,
                "dynamic": True,
            }
        ],
        _dynamic_pose=lambda source, now: pose,
        objects_pub=SimpleNamespace(publish=detections.append),
        markers_pub=None,
        tf_broadcaster=SimpleNamespace(sendTransform=transforms.append),
    )

    ObjectPoseNode.timer_callback(node)

    assert len(detections[0].detections) == 1
    assert transforms[0].header.frame_id == "table_anchor"
    assert transforms[0].child_frame_id == "drawer_handle"


def test_collab_model_frames_resolve_in_the_configured_reference_frame():
    config = load_setup(Path(__file__).parents[1] / "config" / "table_setup.yml")
    graph = load_scene_graph(config)
    ref = expand_iri(graph, frame_by_name(config, config["ref_frame"])["iri"])

    robot_table = transform_between_frames(
        graph,
        expand_iri(graph, frame_by_name(config, "robot_table_top")["iri"]),
        ref,
    )
    rk_table = transform_between_frames(
        graph,
        expand_iri(graph, frame_by_name(config, "rk_table_top")["iri"]),
        ref,
    )

    assert np.allclose(robot_table.translation, [0.295, 0.725, 0.0])
    assert np.allclose(rk_table.translation, [-0.75, 1.87, 0.05])
    assert {
        str(expand_iri(graph, configured_object["iri"]))
        for configured_object in config["objects"]
    } == {
        "https://secorolab.github.io/models/demos/collab/scene/drawer",
        "https://secorolab.github.io/models/demos/collab/scene/robot-ws",
        "https://secorolab.github.io/models/demos/collab/scene/wall-ws",
    }
