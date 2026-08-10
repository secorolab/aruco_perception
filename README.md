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

Launch both the world pose localization node and the detect objects node with:

```bash
ros2 launch aruco_perception table_setup_launch.py config_path:=<path_to_config_file>
```

Or run separately.

### World Pose Localization Node

This node will broadcast the ArUco marker's pose w.r.t. the camera as a static transform. Run it with:

```bash
ros2 run aruco_perception world_pose_node
```

### Detect Objects Node

This node will detect ArUco markers in the camera image and publish their poses as transforms. Run it with:

```bash
ros2 run aruco_perception detect_objects_node
```

### Object Pose Node

Continuously reports the scene objects it can see, for a motion-spec model that subscribes to
them. It broadcasts the same marker TFs as the detect node, and additionally publishes one
`vision_msgs/Detection3DArray` per image where `detection.id` is the object's frame IRI.

Which objects it looks for comes from the scene itself: it loads the generated
`<model>.scenex.ld.json` and takes every frame whose local name is `aruco_<tag>`, so a body
declared as

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

### Mock Object Pose Publisher

The same message, streamed from a static YAML table instead of a camera, for running a model in
simulation where there is no image pipeline.

```bash
ros2 run aruco_perception mock_object_pose_publisher --ros-args \
    -p poses_file:=<path>/mock_poses.yml -p rate_hz:=10.0
```
