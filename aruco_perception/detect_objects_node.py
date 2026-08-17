import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .utils import frame_by_name, imgmsg_to_cv2, load_setup


class DetectObjectsNode(Node):
    def __init__(self, node_name='detect_objects_node'):
        super().__init__(node_name)
        self._logger = self.get_logger()

        self.declare_parameter('config_path', '')
        self.declare_parameter('sensor', 'arm_camera')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise ValueError('parameter config_path is required')

        self.config = load_setup(config_path)
        sensor_name = self.get_parameter('sensor').value
        try:
            camera = self.config['sensors'][sensor_name]
        except KeyError as error:
            raise ValueError(f"table setup has no sensor '{sensor_name}'") from error

        self.table_anchor_frame = self.config['ref_frame']
        ref_frame = frame_by_name(self.config, self.table_anchor_frame)
        try:
            self.world_marker_id = int(ref_frame['marker']['marker_id'])
        except KeyError as error:
            raise ValueError(
                f"reference frame '{self.table_anchor_frame}' must declare a marker"
            ) from error

        marker_frames = {}
        for frame in self.config.get('frames', []):
            marker = frame.get('marker')
            if marker is None:
                continue
            marker_id = int(marker['marker_id'])
            if marker_id in marker_frames:
                raise ValueError(f'marker id {marker_id} is declared more than once')
            marker_frames[marker_id] = {
                'frame': frame['frame'],
                'size': float(marker['marker_size']),
            }
        self.marker_frames = marker_frames
        self.marker_sizes = {
            marker_id: marker['size']
            for marker_id, marker in marker_frames.items()
            if marker_id != self.world_marker_id
        }

        marker_dict_name = self.config.get('marker_dict', 'DICT_4X4_50')
        marker_dict = getattr(cv2.aruco, marker_dict_name, None)
        if marker_dict is None:
            raise ValueError(f'Invalid marker dictionary: {marker_dict_name}')
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(marker_dict),
            cv2.aruco.DetectorParameters(),
        )

        self.camera_matrix = None
        self.dist_coeffs = None

        self.image_sub = self.create_subscription(
            Image, camera['image_topic'], self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera['camera_info_topic'], self.camera_info_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.debug_pub = self.create_publisher(Image, 'debug_image_objects', 10)

        self._logger.info(f"Detect Objects node started for sensor '{sensor_name}'")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self._logger.warning('Camera info not received yet, skipping')
            return

        try:
            tf_world_cam = self.tf_buffer.lookup_transform(
                self.table_anchor_frame, msg.header.frame_id, Time()
            )
        except TransformException:
            self._logger.warning(
                f'{self.table_anchor_frame} -> camera tf not available yet, skipping frame'
            )
            return

        t = tf_world_cam.transform.translation
        q = tf_world_cam.transform.rotation
        T_world_cam = np.eye(4)
        T_world_cam[0:3, 0:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_world_cam[0:3, 3] = [t.x, t.y, t.z]

        frame = imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        detected = []

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
                img_points = corners[i][0].astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    marker_size * 0.5,
                )
                R_cam_marker, _ = cv2.Rodrigues(rvec)
                T_cam_marker = np.eye(4)
                T_cam_marker[0:3, 0:3] = R_cam_marker
                T_cam_marker[0:3, 3] = tvec.reshape(3)
                T_world_marker = T_world_cam @ T_cam_marker
                t_world = T_world_marker[0:3, 3]
                quat_world = R.from_matrix(T_world_marker[0:3, 0:3]).as_quat()

                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = self.table_anchor_frame
                tf_msg.child_frame_id = f'marker_{marker_id}'
                tf_msg.transform.translation.x = float(t_world[0])
                tf_msg.transform.translation.y = float(t_world[1])
                tf_msg.transform.translation.z = float(t_world[2])
                tf_msg.transform.rotation.x = float(quat_world[0])
                tf_msg.transform.rotation.y = float(quat_world[1])
                tf_msg.transform.rotation.z = float(quat_world[2])
                tf_msg.transform.rotation.w = float(quat_world[3])
                self.tf_broadcaster.sendTransform(tf_msg)
                detected.append(marker_id)

        self.markers_detected(detected)

        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = frame.shape[0]
        debug_msg.width = frame.shape[1]
        debug_msg.encoding = 'bgr8'
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
