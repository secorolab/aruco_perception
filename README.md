# ArUco Perception

Detects configured ArUco markers and publishes scene-object poses as ROS 2 detections, TF frames,
and optional RViz markers.

## Setup

Create a workspace virtual environment with ROS packages visible, then install the Python
dependencies:

```bash
cd ~/ws
uv venv --system-site-packages
uv pip install -r src/aruco_perception/requirements.txt
```

## Table demo

The YAML launch file is [`launch/table_setup.launch.yaml`](launch/table_setup.launch.yaml):

```bash
ros2 launch aruco_perception table_setup.launch.yaml \
  publish_markers:=true
```

It starts the static RealSense driver, anchors `table_anchor` from marker 0, and subscribes
object detection to every sensor in the configuration. Drivers for additional cameras must
publish their configured image, camera-info, and TF data separately.

Launch arguments:

- `config_path`: setup YAML; defaults to the installed
  [`config/table_setup.yml`](config/table_setup.yml).
- `max_tf_age`: seconds a higher-priority sensor remains preferred after observing a marker,
  and the maximum age of a marker pose used for object detection; defaults to `0.5`.
- `publish_markers`: publish labeled pose arrows on `/recognized_object_markers`; defaults
  to `false`.

## Configuration

[`config/table_setup.yml`](config/table_setup.yml) contains:

- `sensors`: image and camera-info topics for each camera, in priority order. For each marker,
  the first sensor with a fresh observation is used; later sensors are automatic fallbacks.
- `model`: the SceneX model location. With `path_type: ros`, `package` and `path` are
  resolved through the ROS package index and converted to an RDF graph at startup.
- `ref_frame`: the common output frame, currently `table_anchor`.
- `frames`: frame IRIs, marker IDs and sizes, or fixed offsets from marker frames. Frames
  with an IRI and no `fixed` entry are inferred from the RDF model relative to `ref_frame`
  and published as static TFs. A `fixed` list defines ordered marker alternatives for a
  detected movable frame. Translation offsets are metres; `roll`, `pitch`, and `yaw` are radians.
- `objects`: maps configured frames to names and IRIs published in
  `vision_msgs/Detection3DArray`.

## Outputs

- `/recognized_objects` (`vision_msgs/Detection3DArray`): available object poses in
  `table_anchor`.
- `/tf_static`: `table_anchor -> camera_link` after marker 0 is observed, plus RDF-derived
  fixed frames such as `robot_table_top` and `rk_table_top`.
- `/tf`: detected `table_anchor -> marker_<id>` transforms and the derived `table_anchor -> drawer_handle` transform while a drawer marker is fresh.
- `/recognized_object_markers` (`visualization_msgs/MarkerArray`): optional green pose
  arrows with white object-name labels.
- `/world_pose_node/debug_image`: static-camera image with detected marker axes.

The drawer detection and TF are published only while marker 1 or marker 2 is visible and fresh.
