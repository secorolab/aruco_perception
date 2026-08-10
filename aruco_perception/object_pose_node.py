"""Continuous object-pose source: aruco markers reported under the scene frame IRI naming them."""

import rclpy
from rclpy.time import Time

from rdflib import Graph, Namespace
from rdflib.namespace import RDF, split_uri

from tf2_ros import TransformException

from vision_msgs.msg import Detection3DArray

from .detect_objects_node import DetectObjectsNode
from .utils import detection_msg

GEOM = Namespace('https://comp-rob2b.github.io/metamodels/geometry/structural-entities#')


def tag_frame_iris(scene_file, marker_prefix):
    """
    Read the scene graph and return {aruco tag: frame IRI} for the frames naming a tag.

    The scene is the only statement of which physical marker an object carries: a frame whose
    local name is '<prefix><tag>' says the object it belongs to wears that marker.
    """
    graph = Graph()
    graph.parse(scene_file, format='json-ld')

    tags = {}
    for frame in sorted(graph.subjects(RDF.type, GEOM.Frame), key=str):
        name = split_uri(frame)[1]
        if not name.startswith(marker_prefix):
            continue

        suffix = name.removeprefix(marker_prefix)
        if not suffix.isdigit():
            raise ValueError(
                f"frame '{frame}' names a marker tag that is not an integer: '{suffix}'"
            )

        tag = int(suffix)
        if tag in tags:
            raise ValueError(f"marker tag {tag} is claimed by both '{tags[tag]}' and '{frame}'")
        tags[tag] = str(frame)

    return tags


class ObjectPoseNode(DetectObjectsNode):
    """Publish the pose of every scene object whose marker is currently visible."""

    def __init__(self):
        super().__init__(node_name='object_pose_node')

        self.declare_parameter('scene_file', '')
        self.declare_parameter('marker_prefix', 'aruco_')
        self.declare_parameter('result_frame', 'base_link')
        self.declare_parameter('objects_topic', '/perception/objects')

        scene_file = self.get_parameter('scene_file').value
        if not scene_file:
            raise ValueError('parameter scene_file is required')

        self.result_frame = self.get_parameter('result_frame').value
        self.tag_iris = tag_frame_iris(scene_file, self.get_parameter('marker_prefix').value)

        self.objects_pub = self.create_publisher(
            Detection3DArray,
            self.get_parameter('objects_topic').value,
            10,
        )

        self._logger.info(
            f'Object pose node reporting {len(self.tag_iris)} objects in {self.result_frame}'
        )

    def markers_detected(self, marker_ids):
        """Report the visible markers the scene knows, in the frame the model asked for."""
        stamp = self.get_clock().now().to_msg()
        detections = Detection3DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.result_frame

        for marker_id in marker_ids:
            iri = self.tag_iris.get(marker_id)
            if iri is None:
                continue

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.result_frame, f'marker_{marker_id}', Time()
                )
            except TransformException:
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            detections.detections.append(
                detection_msg(
                    iri,
                    self.result_frame,
                    stamp,
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )
            )

        self.objects_pub.publish(detections)


def main(args=None):
    """Spin the object pose node."""
    rclpy.init(args=args)
    node = ObjectPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
