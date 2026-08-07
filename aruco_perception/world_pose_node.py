import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .utils import imgmsg_to_cv2

class WorldPoseNode(Node):
    def __init__(self):
        super().__init__('world_pose_node')

        self._logger = self.get_logger()

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        self.declare_parameter('marker_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.1)
        self.declare_parameter('marker_id', 0)

        marker_dict = getattr(cv2.aruco, self.get_parameter('marker_dict').value, None)
        if marker_dict is None:
            raise ValueError(f"Invalid marker dictionary: {self.get_parameter('marker_dict').value}")

        aruco_dict = cv2.aruco.getPredefinedDictionary(marker_dict)
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.world_tf_published = False
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value

        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            10
        )

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.debug_pub = self.create_publisher(
            Image,
            'debug_image',
            10
        )

        self._logger.info(f"Tf publisher node started, cv2 version: {cv2.__version__}")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            self._logger.warning("Camera info not received yet, skipping")
            return

        frame = imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        self._logger.info(f"Detected {len(corners)} markers: {ids.flatten() if ids is not None else 'None'}")

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            half = self.marker_size / 2.0
            obj_points = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0]
            ], dtype=np.float64)

            for i, detected_marker_id in enumerate(ids.flatten()):
                if detected_marker_id == self.marker_id and not self.world_tf_published:
                    img_points = corners[i][0].astype(np.float64)
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points,
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if not ok:
                        continue

                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.5)

                    R_cam_marker, _ = cv2.Rodrigues(rvec)
                    T_cam_marker = np.eye(4)
                    T_cam_marker[0:3, 0:3] = R_cam_marker
                    T_cam_marker[0:3, 3] = tvec.reshape(3)
                    
                    T_world_cam = np.linalg.inv(T_cam_marker)
                    t_inv = T_world_cam[0:3, 3]
                    quat_inv = R.from_matrix(T_world_cam[0:3, 0:3]).as_quat()

                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = self.get_clock().now().to_msg()
                    tf_msg.header.frame_id = "world"
                    tf_msg.child_frame_id = msg.header.frame_id
                    tf_msg.transform.translation.x = float(t_inv[0])
                    tf_msg.transform.translation.y = float(t_inv[1])
                    tf_msg.transform.translation.z = float(t_inv[2])
                    tf_msg.transform.rotation.x = float(quat_inv[0])
                    tf_msg.transform.rotation.y = float(quat_inv[1])
                    tf_msg.transform.rotation.z = float(quat_inv[2])
                    tf_msg.transform.rotation.w = float(quat_inv[3])

                    self.tf_static_broadcaster.sendTransform(tf_msg)
                    self.world_tf_published = True

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
