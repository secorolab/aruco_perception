"""Helpers shared by the perception nodes."""

from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rdflib import Namespace
from rdflib.namespace import RDF

from rdf_utils.models.geom_coord import get_transform_between_frames
from rdf_utils.uri import try_expand_curie
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

GEOM = Namespace('https://comp-rob2b.github.io/metamodels/geometry/structural-entities#')


def load_setup(config_path):
    """Load the table setup shared by the launch nodes."""
    with open(config_path) as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"table setup '{config_path}' must contain a mapping")
    return config


def frame_by_name(config, name):
    """Return one named configured frame, rejecting missing or duplicate names."""
    matches = [frame for frame in config.get('frames', []) if frame.get('frame') == name]
    if len(matches) != 1:
        raise ValueError(f"frame '{name}' must be declared exactly once, found {len(matches)}")
    return matches[0]


def load_scene_graph(config):
    """Generate the RDF graph directly from the configured packaged .scenex model."""
    model = config['model']
    if model.get('path_type') != 'ros':
        raise ValueError("model.path_type must be 'ros'")
    path = Path(get_package_share_directory(model['package'])) / model['path']
    return create_scenex_model_graph(scenex_metamodel().model_from_file(str(path)))


def expand_iri(graph, value):
    """Expand one configured CURIE and ensure the scene actually declares it."""
    iri = try_expand_curie(graph.namespace_manager, value)
    if not any(graph.triples((iri, None, None))):
        raise ValueError(f"scene graph has no resource '{value}' ({iri})")
    return iri


def transform_between_frames(graph, frame, ref_frame):
    """Return frame relative to ref_frame, composing through a shared ancestor if needed."""
    direct = get_transform_between_frames(frame, ref_frame, graph)
    if direct is not None:
        return direct

    candidates = [ref_frame, *sorted(graph.subjects(RDF.type, GEOM.Frame), key=str)]
    for ancestor in dict.fromkeys(candidates):
        frame_pose = get_transform_between_frames(frame, ancestor, graph)
        ref_pose = get_transform_between_frames(ref_frame, ancestor, graph)
        if frame_pose is not None and ref_pose is not None:
            return ref_pose.inv() * frame_pose

    raise ValueError(f"no pose path connects '{frame}' to reference frame '{ref_frame}'")


def imgmsg_to_cv2(msg):
    dtype = np.uint16 if '16' in msg.encoding else np.uint8
    channels = 1 if 'mono' in msg.encoding or msg.encoding == '8UC1' else 3
    img = (
        np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        if channels > 1
        else np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    )
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1].copy()
    return img


def detection_msg(iri, frame_id, stamp, position, orientation_xyzw):
    """Fill a Detection3D for both bbox- and hypothesis-pose consumers."""
    detection = Detection3D()
    detection.header.stamp = stamp
    detection.header.frame_id = frame_id
    detection.id = iri

    pose = detection.bbox.center
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(orientation_xyzw[0])
    pose.orientation.y = float(orientation_xyzw[1])
    pose.orientation.z = float(orientation_xyzw[2])
    pose.orientation.w = float(orientation_xyzw[3])

    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.score = 1.0
    hypothesis.pose.pose = pose
    detection.results.append(hypothesis)
    return detection
