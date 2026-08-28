from functools import partial

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .utils import anchored_sensor_specs, imgmsg_to_cv2, load_setup


def observation_has_priority(sensors, sensor_name, marker_id, now_ns, max_age_ns):
    """Return whether this sensor is the first fresh source for a marker."""
    for name, state in sensors.items():
        if name == sensor_name:
            return True
        if marker_id in state["visible"] and now_ns - state["seen_at_ns"] <= max_age_ns:
            return False
    raise ValueError(f"unknown sensor '{sensor_name}'")


def transform_is_fresh(transform, now, max_age):
    """Return whether a dynamic transform is recent enough to consume."""
    stamp = transform.header.stamp
    age_ns = now.nanoseconds - (stamp.sec * 1_000_000_000 + stamp.nanosec)
    return age_ns <= int(max_age * 1e9)


class DetectObjectsNode(Node):
    def __init__(self, node_name="detect_objects_node"):
        super().__init__(node_name)
        self._logger = self.get_logger()

        self.declare_parameter("config_path", "")
        self.declare_parameter("max_tf_age", 0.5)
        config_path = self.get_parameter("config_path").value
        if not config_path:
            raise ValueError("parameter config_path is required")

        self.config = load_setup(config_path)
        self.max_tf_age = float(self.get_parameter("max_tf_age").value)
        configured_sensors = self.config.get("sensors", {})
        anchored_sensors = anchored_sensor_specs(self.config)
        if not configured_sensors:
            raise ValueError("table setup must declare at least one sensor")
        self.sensors = {
            name: {
                "camera_matrix": None,
                "dist_coeffs": None,
                "visible": set(),
                "seen_at_ns": 0,
                "anchored": name in anchored_sensors,
            }
            for name in configured_sensors
        }

        self.table_anchor_frame = self.config["ref_frame"]
        anchor_marker_ids = {
            sensor["marker_id"] for sensor in anchored_sensors.values()
        }

        marker_frames = {}
        for frame in self.config.get("frames", []):
            marker = frame.get("marker")
            if marker is None:
                continue
            marker_id = int(marker["marker_id"])
            if marker_id in marker_frames:
                raise ValueError(f"marker id {marker_id} is declared more than once")
            marker_frames[marker_id] = {
                "frame": frame["frame"],
                "size": float(marker["marker_size"]),
            }
        self.marker_frames = marker_frames
        self.marker_sizes = {
            marker_id: marker["size"]
            for marker_id, marker in marker_frames.items()
            if marker_id not in anchor_marker_ids
        }

        marker_dict_name = self.config.get("marker_dict", "DICT_4X4_50")
        marker_dict = getattr(cv2.aruco, marker_dict_name, None)
        if marker_dict is None:
            raise ValueError(f"Invalid marker dictionary: {marker_dict_name}")
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(marker_dict),
            cv2.aruco.DetectorParameters(),
        )

        self.image_subs = []
        self.camera_info_subs = []
        for sensor_name, camera in configured_sensors.items():
            self.image_subs.append(
                self.create_subscription(
                    Image,
                    camera["image_topic"],
                    partial(self.image_callback, sensor_name=sensor_name),
                    10,
                )
            )
            self.camera_info_subs.append(
                self.create_subscription(
                    CameraInfo,
                    camera["camera_info_topic"],
                    partial(self.camera_info_callback, sensor_name=sensor_name),
                    10,
                )
            )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.debug_pub = self.create_publisher(Image, "debug_image_objects", 10)

        self._logger.info(
            f"Detect Objects node started for sensors: {', '.join(self.sensors)}"
        )

    def camera_info_callback(self, msg, sensor_name):
        sensor = self.sensors[sensor_name]
        sensor["camera_matrix"] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        sensor["dist_coeffs"] = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image, sensor_name):
        sensor = self.sensors[sensor_name]
        camera_matrix = sensor["camera_matrix"]
        if camera_matrix is None:
            self._logger.warning(
                f"Camera info for '{sensor_name}' not received yet, skipping"
            )
            return

        lookup_time = Time() if sensor["anchored"] else Time.from_msg(msg.header.stamp)
        try:
            tf_world_cam = self.tf_buffer.lookup_transform(
                self.table_anchor_frame,
                msg.header.frame_id,
                lookup_time,
            )
        except TransformException:
            self._logger.warning(
                f"{self.table_anchor_frame} -> {msg.header.frame_id} tf not available yet, "
                f"skipping '{sensor_name}' frame"
            )
            return
        if sensor["anchored"] and not transform_is_fresh(
            tf_world_cam, self.get_clock().now(), self.max_tf_age
        ):
            self._logger.warning(f"Camera pose for '{sensor_name}' is stale, skipping")
            return

        t = tf_world_cam.transform.translation
        q = tf_world_cam.transform.rotation
        T_world_cam = np.eye(4)
        T_world_cam[0:3, 0:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_world_cam[0:3, 3] = [t.x, t.y, t.z]

        frame = imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        candidates = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, detected_marker_id in enumerate(ids.flatten()):
                marker_id = int(detected_marker_id)
                marker_size = self.marker_sizes.get(marker_id)
                if marker_size is None:
                    continue

                half = marker_size / 2.0
                obj_points = np.array(
                    [
                        [-half, half, 0],
                        [half, half, 0],
                        [half, -half, 0],
                        [-half, -half, 0],
                    ],
                    dtype=np.float64,
                )
                ok, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corners[i][0].astype(np.float64),
                    camera_matrix,
                    sensor["dist_coeffs"],
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue

                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    sensor["dist_coeffs"],
                    rvec,
                    tvec,
                    marker_size * 0.5,
                )
                R_cam_marker, _ = cv2.Rodrigues(rvec)
                T_cam_marker = np.eye(4)
                T_cam_marker[0:3, 0:3] = R_cam_marker
                T_cam_marker[0:3, 3] = tvec.reshape(3)
                T_world_marker = T_world_cam @ T_cam_marker
                candidates.append(
                    (
                        marker_id,
                        T_world_marker[0:3, 3],
                        R.from_matrix(T_world_marker[0:3, 0:3]).as_quat(),
                    )
                )

        now = self.get_clock().now()
        sensor["visible"] = {marker_id for marker_id, _, _ in candidates}
        sensor["seen_at_ns"] = now.nanoseconds
        detected = []
        max_age_ns = int(self.max_tf_age * 1e9)

        for marker_id, translation, rotation in candidates:
            if not observation_has_priority(
                self.sensors, sensor_name, marker_id, now.nanoseconds, max_age_ns
            ):
                continue

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.table_anchor_frame
            tf_msg.child_frame_id = f"marker_{marker_id}"
            tf_msg.transform.translation.x = float(translation[0])
            tf_msg.transform.translation.y = float(translation[1])
            tf_msg.transform.translation.z = float(translation[2])
            tf_msg.transform.rotation.x = float(rotation[0])
            tf_msg.transform.rotation.y = float(rotation[1])
            tf_msg.transform.rotation.z = float(rotation[2])
            tf_msg.transform.rotation.w = float(rotation[3])
            self.tf_broadcaster.sendTransform(tf_msg)
            detected.append(marker_id)

        self.markers_detected(detected)

        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = frame.shape[0]
        debug_msg.width = frame.shape[1]
        debug_msg.encoding = "bgr8"
        debug_msg.data = frame.tobytes()
        self.debug_pub.publish(debug_msg)

    def markers_detected(self, marker_ids):
        """Hook for subclasses that consume the marker transforms."""


def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
