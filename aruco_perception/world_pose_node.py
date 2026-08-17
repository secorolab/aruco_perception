import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener

from .utils import frame_by_name, imgmsg_to_cv2, load_setup


class WorldPoseNode(Node):
    """Use the reference marker once to anchor the static camera in TF."""

    def __init__(self):
        super().__init__('world_pose_node')
        self._logger = self.get_logger()

        self.declare_parameter('config_path', '')
        self.declare_parameter('camera_link_frame', 'camera_link')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            raise ValueError('parameter config_path is required')

        config = load_setup(config_path)
        camera = config['sensors']['static_camera']
        self.table_anchor_frame = config['ref_frame']
        ref_frame = frame_by_name(config, self.table_anchor_frame)
        try:
            marker = ref_frame['marker']
            self.marker_size = float(marker['marker_size'])
            self.marker_id = int(marker['marker_id'])
        except KeyError as error:
            raise ValueError(
                f"reference frame '{self.table_anchor_frame}' must declare a marker"
            ) from error

        marker_dict_name = config.get('marker_dict', 'DICT_4X4_50')
        marker_dict = getattr(cv2.aruco, marker_dict_name, None)
        if marker_dict is None:
            raise ValueError(f'Invalid marker dictionary: {marker_dict_name}')
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(marker_dict),
            cv2.aruco.DetectorParameters(),
        )

        self.camera_matrix = None
        self.dist_coeffs = None
        self.world_tf_published = False
        self.camera_link_frame = self.get_parameter('camera_link_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_sub = self.create_subscription(
            Image, camera['image_topic'], self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera['camera_info_topic'], self.camera_info_callback, 10
        )
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.debug_pub = self.create_publisher(Image, 'debug_image', 10)
        self._logger.info(f'World pose node started, cv2 version: {cv2.__version__}')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self._logger.warning('Camera info not received yet, skipping')
            return

        frame = imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            half = self.marker_size / 2.0
            obj_points = np.array(
                [
                    [-half, half, 0],
                    [half, half, 0],
                    [half, -half, 0],
                    [-half, -half, 0],
                ],
                dtype=np.float64,
            )

            for i, detected_marker_id in enumerate(ids.flatten()):
                if detected_marker_id != self.marker_id:
                    continue

                ok, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corners[i][0].astype(np.float64),
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
                    self.marker_size * 0.5,
                )
                R_cam_marker, _ = cv2.Rodrigues(rvec)
                T_cam_marker = np.eye(4)
                T_cam_marker[0:3, 0:3] = R_cam_marker
                T_cam_marker[0:3, 3] = tvec.reshape(3)
                T_world_optical = np.linalg.inv(T_cam_marker)

                optical_frame = msg.header.frame_id
                try:
                    tf_optical_link = self.tf_buffer.lookup_transform(
                        optical_frame, self.camera_link_frame, Time()
                    )
                except TransformException:
                    self._logger.warning(
                        f'{optical_frame} -> {self.camera_link_frame} tf not available yet, '
                        'skipping world publish'
                    )
                    continue

                T_optical_link = np.eye(4)
                rotation = tf_optical_link.transform.rotation
                translation = tf_optical_link.transform.translation
                T_optical_link[0:3, 0:3] = R.from_quat(
                    [rotation.x, rotation.y, rotation.z, rotation.w]
                ).as_matrix()
                T_optical_link[0:3, 3] = [
                    translation.x,
                    translation.y,
                    translation.z,
                ]
                T_world_link = T_world_optical @ T_optical_link
                t_world = T_world_link[0:3, 3]
                quat_world = R.from_matrix(T_world_link[0:3, 0:3]).as_quat()

                if not self.world_tf_published:
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = self.get_clock().now().to_msg()
                    tf_msg.header.frame_id = self.table_anchor_frame
                    tf_msg.child_frame_id = self.camera_link_frame
                    tf_msg.transform.translation.x = float(t_world[0])
                    tf_msg.transform.translation.y = float(t_world[1])
                    tf_msg.transform.translation.z = float(t_world[2])
                    tf_msg.transform.rotation.x = float(quat_world[0])
                    tf_msg.transform.rotation.y = float(quat_world[1])
                    tf_msg.transform.rotation.z = float(quat_world[2])
                    tf_msg.transform.rotation.w = float(quat_world[3])
                    self.tf_static_broadcaster.sendTransform(tf_msg)
                    self.world_tf_published = True
                    self._logger.info('Published world pose')

        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = frame.shape[0]
        debug_msg.width = frame.shape[1]
        debug_msg.encoding = 'bgr8'
        debug_msg.data = frame.tobytes()
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
