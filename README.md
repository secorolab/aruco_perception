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

It starts the robot-table and RK-table RealSense cameras, continuously updates each pose while its
configured anchor is visible, and subscribes object detection to every sensor. Cameras without an `anchor_frame`, such
as an end-effector camera, must provide their image, camera-info, and pose through an external TF
chain.

Launch arguments:

- `config_path`: setup YAML; defaults to the installed
  [`config/table_setup.yml`](config/table_setup.yml).
- `max_tf_age`: seconds a higher-priority sensor remains preferred after observing a marker,
  and the maximum age of camera and marker poses used for detection; defaults to `0.5`.
- `publish_markers`: publish labeled pose arrows on `/recognized_object_markers`; defaults
  to `false`.
- `robot_table_camera_serial`: physical camera assigned to the stable
  `/cameras/robot_table_camera` role; defaults to `_336222301119`.
- `rk_table_camera_serial`: physical camera assigned to the stable
  `/cameras/rk_table_camera` role; defaults to `_230322273236`.

## Configuration

[`config/table_setup.yml`](config/table_setup.yml) contains:

- `sensors`: image and camera-info topics for each camera, in priority order. An anchored camera
  also declares `anchor_frame` and `camera_link_frame`; its transform is refreshed on `/tf`
  whenever the anchor is visible. A camera without `anchor_frame` uses an existing TF chain. For
  each object marker, the first sensor with a fresh observation is used; later sensors are fallbacks.
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
- `/tf_static`: RDF-derived fixed frames such as `robot_table_top` and `rk_table_top`.
- `/tf`: continuously refreshed `table_anchor -> robot_table_camera_link` and
  `rk_anchor -> rk_table_camera_link` transforms while their anchors are visible, plus detected
  `table_anchor -> marker_<id>` and derived `table_anchor -> drawer_handle` transforms while
  their observations are fresh.
- `/recognized_object_markers` (`visualization_msgs/MarkerArray`): optional green pose
  arrows with white object-name labels.
- `/world_pose_node/debug_image`: the latest calibration image with detected marker axes.

The drawer detection and TF are published only while marker 1 or marker 2 is visible and fresh.
Because TF lookups are bidirectional and composed across the tree, no duplicate
`camera -> object` transform is published. Query an object relative to any connected camera:

```bash
ros2 run tf2_ros tf2_echo \
  robot_table_camera_color_optical_frame drawer_handle
```
