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
