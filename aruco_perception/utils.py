import numpy as np


def imgmsg_to_cv2(msg):
    dtype = np.uint16 if '16' in msg.encoding else np.uint8
    channels = 1 if 'mono' in msg.encoding or msg.encoding == '8UC1' else 3
    img = np.frombuffer(msg.data, dtype=dtype).reshape(
        msg.height, msg.width, channels) if channels > 1 else \
        np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1].copy()
    return img
