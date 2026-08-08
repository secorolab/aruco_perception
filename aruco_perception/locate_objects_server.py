"""LocateObjects action server backed by the live aruco detection pipeline."""

import time

from aruco_perception.action import LocateObjects

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from tf2_ros import TransformException

from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

import yaml

from .detect_objects_node import DetectObjectsNode


class LocateObjectsServer(DetectObjectsNode):
    """Answer LocateObjects goals from the marker TFs the detection node broadcasts."""

    def __init__(self):
        super().__init__(node_name='locate_objects_server')

        self.declare_parameter('marker_iri_map_file', '')
        self.declare_parameter('result_frame', 'world')
        self.declare_parameter('detect_window_s', 1.0)

        map_file = self.get_parameter('marker_iri_map_file').value
        if not map_file:
            raise ValueError('parameter marker_iri_map_file is required')

        with open(map_file) as f:
            marker_map = yaml.safe_load(f)['markers']

        self.iri_markers = {iri: marker_id for marker_id, iri in marker_map.items()}
        self.result_frame = self.get_parameter('result_frame').value
        self.detect_window_s = self.get_parameter('detect_window_s').value

        self._action_server = ActionServer(
            self,
            LocateObjects,
            '/perception/locate',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._logger.info(
            f'Locate objects server started with {len(self.iri_markers)} markers'
        )

    def goal_callback(self, goal_request):
        """Reject malformed goals, accept everything else."""
        if not goal_request.target_iris:
            self._logger.warning('Rejecting goal with empty target_iris')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Succeed iff every requested IRI was seen within the detection window."""
        result = LocateObjects.Result()

        pending = {}
        for iri in goal_handle.request.target_iris:
            if iri not in self.iri_markers:
                self._logger.warning(f'Unknown target IRI: {iri}')
                goal_handle.abort()
                return result
            pending[iri] = self.iri_markers[iri]

        start = self.get_clock().now()
        deadline = time.monotonic() + self.detect_window_s
        found = {}

        while pending and time.monotonic() < deadline:
            for iri, marker_id in list(pending.items()):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.result_frame, f'marker_{marker_id}', Time()
                    )
                except TransformException:
                    continue
                # only accept a marker seen after the goal arrived
                if Time.from_msg(transform.header.stamp) < start:
                    continue
                found[iri] = transform
                del pending[iri]
            time.sleep(0.05)

        if pending:
            self._logger.warning(f'Targets not found: {sorted(pending)}')
            goal_handle.abort()
            return result

        stamp = self.get_clock().now().to_msg()
        detections = Detection3DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self.result_frame
        for iri in goal_handle.request.target_iris:
            detections.detections.append(self._detection(iri, found[iri], stamp))

        result.detections = detections
        goal_handle.succeed()
        return result

    def _detection(self, iri, transform, stamp):
        detection = Detection3D()
        detection.header.stamp = stamp
        detection.header.frame_id = self.result_frame
        detection.id = iri

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.score = 1.0
        hypothesis.pose.pose.position.x = transform.transform.translation.x
        hypothesis.pose.pose.position.y = transform.transform.translation.y
        hypothesis.pose.pose.position.z = transform.transform.translation.z
        hypothesis.pose.pose.orientation = transform.transform.rotation
        detection.results.append(hypothesis)

        return detection


def main(args=None):
    """Spin the locate objects server on a multi-threaded executor."""
    rclpy.init(args=args)
    node = LocateObjectsServer()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
