"""Mock LocateObjects action server answering from a static YAML pose table."""

import time

from aruco_perception.action import LocateObjects

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray

import yaml

from .utils import detection_msg


class MockLocateServer(Node):
    """Serve LocateObjects goals from a file-backed pose table."""

    def __init__(self):
        super().__init__('mock_locate_server')

        self._logger = self.get_logger()

        self.declare_parameter('poses_file', '')
        self.declare_parameter('response_delay_s', 0.2)

        poses_file = self.get_parameter('poses_file').value
        if not poses_file:
            raise ValueError('parameter poses_file is required')

        with open(poses_file) as f:
            table = yaml.safe_load(f)

        self.frame_id = table['frame_id']
        self.objects = table['objects']
        self.response_delay_s = self.get_parameter('response_delay_s').value

        self._action_server = ActionServer(
            self,
            LocateObjects,
            '/perception/locate',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
        )

        self._logger.info(f'Mock locate server started with {len(self.objects)} objects')

    def goal_callback(self, goal_request):
        """Reject malformed goals, accept everything else."""
        if not goal_request.target_iris:
            self._logger.warning('Rejecting goal with empty target_iris')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Succeed iff every requested IRI is in the table, abort otherwise."""
        time.sleep(self.response_delay_s)

        result = LocateObjects.Result()
        stamp = self.get_clock().now().to_msg()
        detections = Detection3DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.frame_id

        for iri in goal_handle.request.target_iris:
            entry = self.objects.get(iri)
            if entry is None:
                self._logger.warning(f'Unknown target IRI: {iri}')
                goal_handle.abort()
                return result
            detections.detections.append(
                detection_msg(
                    iri,
                    self.frame_id,
                    stamp,
                    entry['position'],
                    entry['orientation_xyzw'],
                )
            )

        result.detections = detections
        goal_handle.succeed()
        return result


def main(args=None):
    """Spin the mock locate server."""
    rclpy.init(args=args)
    node = MockLocateServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
