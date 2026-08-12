"""Publish the handle frame as a static offset from the marker it is attached to."""

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class HandlePoseNode(Node):
    """Broadcast a static transform marker_<id> -> handle so the handle follows its marker."""

    def __init__(self):
        super().__init__('handle_pose_node')

        self._logger = self.get_logger()

        self.declare_parameter('marker_id', 1)
        self.declare_parameter('handle_frame', 'handle')
        self.declare_parameter('offset_x', -0.055)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.02)
        self.declare_parameter('offset_roll', 0.0)
        self.declare_parameter('offset_pitch', 0.0)
        self.declare_parameter('offset_yaw', 0.0)

        marker_id = int(self.get_parameter('marker_id').value)
        parent_frame = f'marker_{marker_id}'
        child_frame = self.get_parameter('handle_frame').value

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(self.get_parameter('offset_x').value)
        tf_msg.transform.translation.y = float(self.get_parameter('offset_y').value)
        tf_msg.transform.translation.z = float(self.get_parameter('offset_z').value)

        quat = R.from_euler(
            'xyz',
            [
                self.get_parameter('offset_roll').value,
                self.get_parameter('offset_pitch').value,
                self.get_parameter('offset_yaw').value,
            ],
        ).as_quat()
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster.sendTransform(tf_msg)

        self._logger.info(
            f'Published static handle transform {parent_frame} -> {child_frame} at '
            f'({tf_msg.transform.translation.x}, {tf_msg.transform.translation.y}, '
            f'{tf_msg.transform.translation.z})'
        )


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
