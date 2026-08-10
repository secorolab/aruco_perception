"""Helpers shared by the perception nodes."""

import numpy as np

from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose


def imgmsg_to_cv2(msg):
    dtype = np.uint16 if '16' in msg.encoding else np.uint8
    channels = 1 if 'mono' in msg.encoding or msg.encoding == '8UC1' else 3
    img = (
        np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        if channels > 1
        else np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    )
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1].copy()
    return img


def detection_msg(iri, frame_id, stamp, position, orientation_xyzw):
    """
    Fill one Detection3D: the scene IRI it is about, the frame it is stated in, its pose.

    Every source of object poses fills this the same way. A subscriber matches a detection on
    (id, frame_id) and reads the pose out of results[0], so any divergence here is silent --
    the detection is simply dropped and nothing moves.
    """
    detection = Detection3D()
    detection.header.stamp = stamp
    detection.header.frame_id = frame_id
    detection.id = iri

    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.score = 1.0
    hypothesis.pose.pose.position.x = float(position[0])
    hypothesis.pose.pose.position.y = float(position[1])
    hypothesis.pose.pose.position.z = float(position[2])
    hypothesis.pose.pose.orientation.x = float(orientation_xyzw[0])
    hypothesis.pose.pose.orientation.y = float(orientation_xyzw[1])
    hypothesis.pose.pose.orientation.z = float(orientation_xyzw[2])
    hypothesis.pose.pose.orientation.w = float(orientation_xyzw[3])
    detection.results.append(hypothesis)

    return detection
