# ArUco Perception Package

Package for working with ArUco markers.

## Setup

Create a virtual environment in your workspace with system site packages:

```bash
cd ~/ws
uv venv --system-site-packages
```

Since OpenCV >= 5.0.0 is required, install the dependencies from `requirements.txt`:

```bash
uv pip install -r src/aruco_perception/requirements.txt
```

## Running Table Setup

The setup uses two cameras: a static camera that anchors the world frame, and a camera mounted
on the Kinova arm. Launch everything -- the static camera driver + world pose node, and the arm
camera's object pose node + the tray handle pose node -- with:

```bash
ros2 launch aruco_perception table_setup_launch.py config_path:=<path_to_config_file>
```

This includes `world_pose_launch.py` and `object_pose_launch.py` (with its `camera` argument
left at its default, `arm`), forwarding `config_path` to both. Run the two halves separately if
the two cameras are brought up on different machines, or to also track the tray markers from the
static camera:

```bash
ros2 launch aruco_perception world_pose_launch.py config_path:=<path_to_config_file>
ros2 launch aruco_perception object_pose_launch.py config_path:=<path_to_config_file> camera:=arm
ros2 launch aruco_perception object_pose_launch.py config_path:=<path_to_config_file> camera:=static
```

### World Pose Launch / World Pose Localization Node

`world_pose_launch.py` starts the static camera's `realsense2_camera` driver and
`world_pose_node`, with the node's parameters translated from `config/table_setup.yml`'s
`static_camera` and `world_marker` sections -- it takes no `camera` argument, it is always the
static camera.

The node itself broadcasts the world marker's pose w.r.t. the camera as a static transform
`table_anchor -> camera_link`, and additionally publishes it on every frame as a
`vision_msgs/Detection3DArray` on `world_pose_topic` (default `/recognized_objects`), where
`detection.id` is the IRI given by the required `world_iri` parameter (`world_marker.iri` in
`table_setup.yml`). Run it directly with:

```bash
ros2 run aruco_perception world_pose_node --ros-args \
    -p world_iri:=<the world frame's IRI>
```

### Object Pose Launch

`object_pose_launch.py` takes `config_path` and a `camera` argument (`static` or `arm`, default
`arm`) that selects the matching `static_camera` / `arm_camera` section of `table_setup.yml`.
It always starts two nodes together:

- `object_pose_node`, namespaced `static_camera` or `arm_camera` to match, watching that
  camera's image topic.
- `handle_pose_node`, fed the `tray` section of the same config file. It isn't camera-specific,
  so running `object_pose_launch.py` for both cameras at once starts two of these and they will
  collide on the node name -- launch it from only one camera's invocation in that case.

```bash
ros2 launch aruco_perception object_pose_launch.py camera:=static
ros2 launch aruco_perception object_pose_launch.py camera:=arm
```

### Detect Objects Node

The base ArUco detector `object_pose_node` builds on: detects markers in the camera image and
broadcasts their poses as dynamic transforms `table_anchor -> marker_<id>` (skipping the world
marker). Not wired into any launch file; run it standalone with:

```bash
ros2 run aruco_perception detect_objects_node
```

### Object Pose Node

Continuously reports the scene objects it can see, for a motion-spec model that subscribes to
them. It broadcasts the same marker TFs as the detect node, and additionally publishes one
`vision_msgs/Detection3DArray` per image where `detection.id` is the object's frame IRI.

Which objects it looks for comes from the scene itself: it loads the generated
`<model>.scenex.ld.json` (`scene_file`) and takes every frame whose local name is
`aruco_<tag>`, so a body declared as

```
body cube {
    frame aruco_12 { ... }
}
```

is reported under its own IRI whenever marker 12 is seen.

```bash
ros2 run aruco_perception object_pose_node --ros-args \
    -p scene_file:=<generation>/generated/model/<model>.scenex.ld.json \
    -p result_frame:=base_link
```

`result_frame` must equal the `wrt:` frame of the world pose in the model. A detection that
arrives in any other frame is not the quantity the model declared, and the subscriber drops it
silently — check this first if perception is running and the robot does nothing.

### Handle Pose Node

The tray the robot handles carries two markers: marker 1 (`tray.marker_front`) next to the
handle, and marker 2 (`tray.marker_back`) on the tray's far side. Each has its own fixed offset
to the handle. This node watches the TF of both marker frames (published dynamically by
whichever object pose node currently sees them) and, whenever one is fresh, applies the matching
offset to get the handle's pose in `table_anchor_frame`, broadcasts that as a dynamic transform
`table_anchor -> handle_frame`, and publishes it as a single-detection
`vision_msgs/Detection3DArray` on `handle_pose_topic` (default `/recognized_objects`).
`detection.id` is the handle's IRI, resolved from `scene_file` by looking up the frame named
`handle_frame` -- same mechanism as the object pose node's `aruco_<tag>` lookup. If both markers
are visible, marker 1 is preferred since it sits closer to the handle.

`table_anchor_frame`, `handle_frame`, `scene_file`, `marker_front_id`, `marker_front_offset`,
`marker_back_id` and `marker_back_offset` describe the physical tray, so they have no defaults
-- the node raises an error naming the missing parameter instead of silently mismeasuring the
handle. `object_pose_launch.py` fills them in from `table_setup.yml`; running the node directly
requires passing them explicitly:

```bash
ros2 run aruco_perception handle_pose_node --ros-args \
    -p table_anchor_frame:=table_anchor \
    -p handle_frame:=handle \
    -p scene_file:=<generation>/generated/model/<model>.scenex.ld.json \
    -p marker_front_id:=1 -p marker_front_offset:="[-0.065, 0.0, 0.02, 0.0, 0.0, 0.0]" \
    -p marker_back_id:=2 -p marker_back_offset:="[0.0, 0.0, -0.22, 0.0, 0.0, 0.0]"
```

### Mock Object Pose Publisher

The same message, streamed from a static YAML table instead of a camera, for running a model in
simulation where there is no image pipeline.

```bash
ros2 run aruco_perception mock_object_pose_publisher --ros-args \
    -p poses_file:=<path>/mock_poses.yml -p rate_hz:=10.0
```
