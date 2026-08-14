"""Publish the tray handle's pose whenever either tray marker is currently visible."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from vision_msgs.msg import Detection3DArray

from .utils import detection_msg, scene_frame_iris


def offset_matrix(x, y, z, roll, pitch, yaw):
    """Build the rigid marker -> handle transform from a translation + rpy offset."""
    matrix = np.eye(4)
    matrix[0:3, 0:3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    matrix[0:3, 3] = [x, y, z]
    return matrix


class HandlePoseNode(Node):
    """
    Track the tray by whichever of its two markers is visible and publish the handle pose.

    Marker 1 sits on the tray next to the handle; marker 2 sits on the tray's far side. Each
    carries its own fixed offset to the handle (the `tray` section of table_setup.yml), so
    whichever marker a camera currently sees is enough to recover the handle's pose in
    table_anchor -- the offset used must match the marker that was actually seen. Marker 1 is
    preferred when both are visible since it sits closest to the handle.
    """

    def __init__(self):
        super().__init__('handle_pose_node')

        self._logger = self.get_logger()

        self.declare_parameter('table_anchor_frame', '')
        self.declare_parameter('handle_frame', '')
        self.declare_parameter('scene_file', '')
        self.declare_parameter('marker_front_id', -1)
        self.declare_parameter('marker_front_offset', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('marker_back_id', -1)
        self.declare_parameter('marker_back_offset', Parameter.Type.DOUBLE_ARRAY)

        self.declare_parameter('handle_pose_topic', '/recognized_objects')
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('max_tf_age', 0.5)

        self.table_anchor_frame = self._require_str('table_anchor_frame')
        self.handle_frame = self._require_str('handle_frame')
        self.max_tf_age = self.get_parameter('max_tf_age').value

        scene_file = self._require_str('scene_file')
        handle_iris = scene_frame_iris(scene_file).get(self.handle_frame, [])
        if len(handle_iris) == 0:
            raise ValueError(
                f"scene file '{scene_file}' has no frame named '{self.handle_frame}' "
                "(the 'handle_frame' parameter)"
            )
        if len(handle_iris) > 1:
            raise ValueError(
                f"scene file '{scene_file}' has more than one frame named "
                f"'{self.handle_frame}': {handle_iris}"
            )
        self.handle_iri = handle_iris[0]

        # Ordered by priority: the marker closest to the handle wins when both are visible.
        self.tray_markers = [
            (
                self._require_marker_id('marker_front_id'),
                self._require_offset('marker_front_offset'),
            ),
            (
                self._require_marker_id('marker_back_id'),
                self._require_offset('marker_back_offset'),
            ),
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.handle_pose_pub = self.create_publisher(
            Detection3DArray,
            self.get_parameter('handle_pose_topic').value,
            10,
        )

        rate_hz = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

        self._logger.info(
            'Handle pose node tracking markers '
            f'{[marker_id for marker_id, _ in self.tray_markers]} -> '
            f'{self.handle_frame} ({self.handle_iri})'
        )

    def _require_str(self, name):
        value = self.get_parameter(name).value
        if not value:
            raise ValueError(
                f"parameter '{name}' is required (set it from the tray/world_marker section of "
                'table_setup.yml)'
            )
        return value

    def _require_marker_id(self, name):
        value = int(self.get_parameter(name).value)
        if value < 0:
            raise ValueError(
                f"parameter '{name}' is required (set it from the tray section of "
                'table_setup.yml)'
            )
        return value

    def _require_offset(self, name):
        value = self.get_parameter(name).value
        if not value or len(value) != 6:
            raise ValueError(
                f"parameter '{name}' is required and must have 6 values "
                '[x, y, z, roll, pitch, yaw] (set it from the tray section of table_setup.yml)'
            )
        return offset_matrix(*value)

    def timer_callback(self):
        now = self.get_clock().now()

        for marker_id, T_marker_handle in self.tray_markers:
            marker_frame = f'marker_{marker_id}'
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.table_anchor_frame, marker_frame, Time()
                )
            except TransformException:
                continue

            age = (now - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if age > self.max_tf_age:
                # Stale TF: the marker was seen before but is not currently detected.
                continue

            t = transform.transform.translation
            q = transform.transform.rotation
            T_table_marker = np.eye(4)
            T_table_marker[0:3, 0:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T_table_marker[0:3, 3] = [t.x, t.y, t.z]

            T_table_handle = T_table_marker @ T_marker_handle
            position = T_table_handle[0:3, 3]
            orientation = R.from_matrix(T_table_handle[0:3, 0:3]).as_quat()

            stamp = self.get_clock().now().to_msg()

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.table_anchor_frame
            tf_msg.child_frame_id = self.handle_frame
            tf_msg.transform.translation.x = float(position[0])
            tf_msg.transform.translation.y = float(position[1])
            tf_msg.transform.translation.z = float(position[2])
            tf_msg.transform.rotation.x = float(orientation[0])
            tf_msg.transform.rotation.y = float(orientation[1])
            tf_msg.transform.rotation.z = float(orientation[2])
            tf_msg.transform.rotation.w = float(orientation[3])
            self.tf_broadcaster.sendTransform(tf_msg)

            handle_pose = Detection3DArray()
            handle_pose.header.stamp = stamp
            handle_pose.header.frame_id = self.table_anchor_frame
            handle_pose.detections.append(
                detection_msg(
                    self.handle_iri,
                    self.table_anchor_frame,
                    stamp,
                    position,
                    orientation,
                )
            )
            self.handle_pose_pub.publish(handle_pose)
            return


def main(args=None):
    rclpy.init(args=args)
    node = HandlePoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
