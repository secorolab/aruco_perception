"""Mock object-pose source streaming a static YAML pose table where there is no camera."""

import rclpy
import yaml
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from .utils import detection_msg


class MockObjectPosePublisher(Node):
    """Publish a file-backed pose table on a timer, as the camera node would."""

    def __init__(self):
        super().__init__("mock_object_pose_publisher")

        self._logger = self.get_logger()

        self.declare_parameter("poses_file", "")
        self.declare_parameter("objects_topic", "/recognized_objects")
        self.declare_parameter("rate_hz", 10.0)

        poses_file = self.get_parameter("poses_file").value
        if not poses_file:
            raise ValueError("parameter poses_file is required")

        with open(poses_file) as f:
            table = yaml.safe_load(f)

        self.frame_id = table["frame_id"]
        self.objects = table["objects"]

        rate_hz = self.get_parameter("rate_hz").value
        if rate_hz <= 0.0:
            raise ValueError(f"parameter rate_hz must be positive, got {rate_hz}")

        self.objects_pub = self.create_publisher(
            Detection3DArray,
            self.get_parameter("objects_topic").value,
            10,
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_poses)

        self._logger.info(
            f"Mock object pose publisher started with {len(self.objects)} objects "
            f"in {self.frame_id} at {rate_hz} Hz"
        )

    def publish_poses(self):
        """Publish the whole table as one detection array."""
        stamp = self.get_clock().now().to_msg()
        detections = Detection3DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.frame_id

        for iri, entry in self.objects.items():
            detections.detections.append(
                detection_msg(
                    iri,
                    self.frame_id,
                    stamp,
                    entry["position"],
                    entry["orientation_xyzw"],
                )
            )

        self.objects_pub.publish(detections)


def main(args=None):
    """Spin the mock object pose publisher."""
    rclpy.init(args=args)
    node = MockObjectPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
