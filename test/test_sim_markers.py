import cv2
import numpy as np
from aruco_perception.sim_markers import (
    marker_image,
    marker_mjcf,
    module_count,
    setup_config,
)


def test_module_count_adds_the_printed_border():
    assert module_count('DICT_4X4_50') == 6
    assert module_count('DICT_5X5_100') == 7


def test_marker_image_is_padded_with_a_white_quiet_zone():
    image = marker_image('DICT_4X4_50', 0, 100, 1)

    assert image.shape == (800, 800)
    assert image[0, 0] == 255
    assert image[99, 99] == 255


def test_marker_geom_grows_by_the_quiet_zone():
    assert 'size="0.0666667 0.0666667 0.0005"' in marker_mjcf(0, 0.1, 1, 'DICT_4X4_50')


def test_generated_marker_decodes():
    image = marker_image('DICT_4X4_50', 1, 100, 1)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters(),
    )

    _, ids, _ = detector.detectMarkers(image)

    assert np.array(ids).flatten().tolist() == [1]


def test_setup_config_matches_the_reader_schema():
    config = setup_config(
        {'dictionary': 'DICT_4X4_50', 'markers': [{'frame': 'marker_1', 'id': 1, 'size': 0.1}]},
        {'static_camera': {'image_topic': '/perception/color'}},
        {'path_type': 'ros'},
        'table_anchor',
        [{'frame': 'robot_table_top', 'iri': 'collab-scn:x'}],
        [{'name': 'probe_obj', 'frame': 'marker_1', 'iri': 'collab-scn:y'}],
    )

    assert config['marker_dict'] == 'DICT_4X4_50'
    assert config['frames'][0] == {
        'frame': 'marker_1',
        'marker': {'marker_size': 0.1, 'marker_id': 1},
    }
    assert config['frames'][1]['frame'] == 'robot_table_top'
