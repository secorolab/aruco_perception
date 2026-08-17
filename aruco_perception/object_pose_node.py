"""Publish configured scene-object poses in one reference frame."""

from copy import deepcopy

import numpy as np
import rclpy
from rclpy.time import Time
from scipy.spatial.transform import RigidTransform
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .detect_objects_node import DetectObjectsNode
from .utils import (
    detection_msg,
    expand_iri,
    frame_by_name,
    load_scene_graph,
    transform_between_frames,
)


def offset_transform(offset):
    """Build the configured wrt-frame -> derived-frame transform."""
    matrix = np.eye(4)
    matrix[0:3, 0:3] = R.from_euler(
        'xyz', [offset['roll'], offset['pitch'], offset['yaw']]
    ).as_matrix()
    matrix[0:3, 3] = [offset['x'], offset['y'], offset['z']]
    return RigidTransform.from_matrix(matrix)


def transform_message_value(transform):
    """Convert a geometry Transform into scipy's rigid transform."""
    value = np.eye(4)
    t = transform.translation
    q = transform.rotation
    value[0:3, 0:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    value[0:3, 3] = [t.x, t.y, t.z]
    return RigidTransform.from_matrix(value)


def transform_stamped(parent, child, stamp, pose):
    """Convert a rigid transform into a stamped TF message."""
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (
        pose.translation
    )
    (
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w,
    ) = pose.rotation.as_quat()
    return msg


class ObjectPoseNode(DetectObjectsNode):
    """Publish static model poses and marker-derived poses as one Detection3DArray."""

    def __init__(self):
        super().__init__(node_name='object_pose_node')

        self.declare_parameter('objects_topic', '/recognized_objects')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('publish_markers', False)

        graph = load_scene_graph(self.config)
        ref_spec = frame_by_name(self.config, self.table_anchor_frame)
        ref_iri = expand_iri(graph, ref_spec['iri'])

        static_frames = {
            frame['frame']: transform_between_frames(
                graph, expand_iri(graph, frame['iri']), ref_iri
            )
            for frame in self.config.get('frames', [])
            if frame['frame'] != self.table_anchor_frame
            and 'iri' in frame
            and not frame.get('fixed')
        }
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        self.static_tf_broadcaster.sendTransform(
            [
                transform_stamped(self.table_anchor_frame, name, stamp, pose)
                for name, pose in static_frames.items()
            ]
        )

        marker_ids = {
            marker['frame']: marker_id for marker_id, marker in self.marker_frames.items()
        }
        objects = []
        for configured_object in self.config.get('objects', []):
            frame = frame_by_name(self.config, configured_object['frame'])
            object_iri = str(expand_iri(graph, configured_object['iri']))
            fixed = frame.get('fixed')
            if fixed:
                if 'iri' in frame:
                    expand_iri(graph, frame['iri'])
                alternatives = []
                for relation in fixed:
                    try:
                        marker_id = marker_ids[relation['wrt']]
                    except KeyError as error:
                        raise ValueError(
                            f"fixed frame '{frame['frame']}' refers to non-marker "
                            f"frame '{relation['wrt']}'"
                        ) from error
                    alternatives.append((marker_id, offset_transform(relation)))
                source = alternatives
            else:
                source = static_frames[frame['frame']]

            objects.append(
                {
                    'name': configured_object['name'],
                    'frame': frame['frame'],
                    'iri': object_iri,
                    'source': source,
                    'dynamic': bool(fixed),
                }
            )
        self.objects = objects

        self.objects_pub = self.create_publisher(
            Detection3DArray, self.get_parameter('objects_topic').value, 10
        )
        self.markers_pub = (
            self.create_publisher(MarkerArray, '/recognized_object_markers', 10)
            if self.get_parameter('publish_markers').value
            else None
        )
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self._logger.info(
            f"Reporting {len(self.objects)} configured objects in {self.table_anchor_frame}"
        )

    def _dynamic_pose(self, alternatives, now):
        for marker_id, marker_to_frame in alternatives:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.table_anchor_frame, f'marker_{marker_id}', Time()
                )
            except TransformException:
                continue

            age = (now - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if age <= self.max_tf_age:
                return transform_message_value(transform.transform) * marker_to_frame
        return None

    def timer_callback(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        detections = Detection3DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.table_anchor_frame
        published = []

        for configured_object in self.objects:
            pose = (
                self._dynamic_pose(configured_object['source'], now)
                if configured_object['dynamic']
                else configured_object['source']
            )
            if pose is None:
                continue

            detection = detection_msg(
                configured_object['iri'],
                self.table_anchor_frame,
                stamp,
                pose.translation,
                pose.rotation.as_quat(),
            )
            detections.detections.append(detection)
            published.append((configured_object, detection))
            if configured_object['dynamic']:
                self.tf_broadcaster.sendTransform(
                    transform_stamped(
                        self.table_anchor_frame, configured_object['frame'], stamp, pose
                    )
                )

        self.objects_pub.publish(detections)
        if self.markers_pub is not None:
            self.markers_pub.publish(self._visualization_markers(published, stamp))

    def _visualization_markers(self, published, stamp):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for configured_object, detection in published:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.table_anchor_frame
            marker.ns = configured_object['name']
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = detection.bbox.center
            marker.scale.x = 0.15
            marker.scale.y = 0.025
            marker.scale.z = 0.025
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 0.9
            markers.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = configured_object['name']
            label.id = 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = deepcopy(detection.bbox.center)
            label.pose.position.z += 0.08
            label.scale.z = 0.05
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = configured_object['name']
            markers.markers.append(label)
        return markers


def main(args=None):
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
