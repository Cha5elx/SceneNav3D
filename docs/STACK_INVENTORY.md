# Stack Inventory

## 1. Conclusion

The project is feasible in the current WSL2 environment.

The smallest reliable path is:

1. Reuse the existing FishBot Gazebo and Nav2 stack from `~/dev_ws/CHAPT8/chapt8_ws/src`.
2. Add deterministic semantic navigation first: text command -> YAML target -> safe approach pose -> Nav2 `NavigateToPose`.
3. Use the simulation RGB-D camera for early perception tests, while keeping the new perception interface compatible with the physical robot's expected monocular RGB camera.
4. Integrate Chat-Scene only after its source tree, checkpoint, and Python environment are provided.

The current repository now contains the copied simulation baseline under `ros2_ws/src`.

## 2. Current WSL2 Environment

| Item | Observed value |
|---|---|
| OS | WSL2, Ubuntu 22.04.5 LTS |
| Kernel | `6.6.87.2-microsoft-standard-WSL2` |
| CPU | 16 logical CPUs |
| Memory | 15 GiB total, about 3.3 GiB available during inspection |
| Disk | about 894 GiB available |
| ROS2 | Humble |
| Gazebo | Gazebo Classic 11.10.2 |
| GUI | WSLg available through `DISPLAY=:0` and `WAYLAND_DISPLAY=wayland-0` |
| GPU | NVIDIA GeForce RTX 5070 family, 12227 MiB VRAM |
| GPU state during inspection | about 11454 MiB VRAM already in use |
| CUDA driver capability | reported as CUDA 13.2 by `nvidia-smi` |
| Python | 3.10.12 |

Installed ROS packages include Nav2, `slam_toolbox`, `robot_localization`, `gazebo_ros`, `gazebo_ros2_control`, `ros_gz`, `cv_bridge`, `image_transport`, `tf2_ros`, and `rosbag2`.

The system Python environment has OpenCV, NumPy, PyYAML, and SciPy. It does not currently have PyTorch, torchvision, transformers, ultralytics, Detectron2, or Open3D.

## 3. Reusable ROS2 Simulation Stack

The most complete reusable baseline is:

```text
~/dev_ws/CHAPT8/chapt8_ws/src/
├── fishbot_description/
├── fishbot_navigation2/
├── fishbot_application/
├── autopatrol_interfaces/
├── autopatrol_robot/
├── nav2_custom_controller/
└── nav2_custom_planner/
```

Copied baseline:

| Package | Recommendation | Reason |
|---|---|---|
| `fishbot_description` | Copied | Contains Gazebo world, URDF/Xacro, sensors, controllers, and simulation launch |
| `fishbot_navigation2` | Copied | Contains map, Nav2 params, and Nav2 bringup launch |
| `fishbot_application` | Copied as a reference | Contains a working Python `BasicNavigator.goToPose()` example |
| `nav2_custom_controller` | Copied | Current Nav2 params select this controller |
| `nav2_custom_planner` | Do not copy initially | Present in the workspace but the active Nav2 params use `nav2_navfn_planner/NavfnPlanner` |
| `autopatrol_*` | Do not copy initially | Patrol and speech demo code are not required for the semantic navigation MVP |

The copied packages build successfully with `colcon build --symlink-install`. The copied URDF parses successfully with `xacro` and `check_urdf`.

## 4. Observed Simulation Interfaces

The existing FishBot Gazebo launch starts successfully.

### Frames

The static robot TF structure includes:

```text
base_footprint
└── base_link
    ├── camera_link
    │   └── camera_optical_link
    ├── imu_link
    ├── laser_cylinder_link
    │   └── laser_link
    ├── left_wheel_link
    └── right_wheel_link
```

Observed camera transform:

```text
base_footprint -> camera_optical_link
translation: [0.100, 0.000, 0.166]
RPY: [-1.571, 0.000, -1.571]
```

The intended navigation TF chain is:

```text
map -> odom -> base_footprint -> base_link
```

`odom -> base_footprint` is published by the diff-drive controller. `map -> odom` is expected from AMCL after an initial pose is provided.

### Topics

Observed topics include:

| Topic | Type | Notes |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D lidar, about 10 Hz |
| `/imu` | `sensor_msgs/msg/Imu` | about 100 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | diff-drive odometry |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | controller input; semantic nodes must not publish this |
| `/camera_sensor/image_raw` | `sensor_msgs/msg/Image` | RGB image |
| `/camera_sensor/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB intrinsics |
| `/camera_sensor/depth/image_raw` | `sensor_msgs/msg/Image` | simulated depth |
| `/camera_sensor/points` | `sensor_msgs/msg/PointCloud2` | simulated point cloud |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | transforms |

The simulation camera plugin is RGB-D, not monocular RGB. This is useful for simulation tests but must not become an assumption in the physical-robot interface.

### Nav2

The existing Nav2 launch loads the map and exposes:

```text
/navigate_to_pose [nav2_msgs/action/NavigateToPose]
```

The map is `380 x 231` cells at `0.05 m/cell`.

During inspection, Nav2 waited for `map -> odom` because AMCL had not received an initial pose. A bringup flow for this project must explicitly set or request the initial pose before navigation tests.

The copied Gazebo launch can start from this repository and loads its controller YAML from this repository's `ros2_ws/install`. Under WSL2 load, the original launch occasionally times out while loading `fishbot_joint_state_broadcaster`; the diff-drive controller still becomes active.

The copied Nav2 launch now explicitly disables composition. The original composed bringup intermittently timed out while loading components under WSL2 load. With independent Nav2 processes, the complete stack starts successfully. The existing `init_robot_pose` and `nav_to_pose` examples were verified end to end: the robot initialized at `(0, 0)`, navigated to `(1.0, 1.0)`, and reported success.

## 5. Missing Inputs

### Required before Chat-Scene integration

- Chat-Scene source directory or repository URL and exact commit.
- Chat-Scene checkpoint file path and checkpoint name.
- Its owned environment file, such as `requirements.txt`, `environment.yml`, or setup instructions.
- Expected inference input format and any existing preprocessing output.
- A small sample scene or fixture query for validating one object-ID result.

### Required before physical-robot deployment

- Physical robot source packages or repository path.
- Actual camera model, resolution, frame rate, calibration YAML, topic names, and camera-to-base extrinsic.
- Actual lidar model, whether it is 2D or 3D, topic, and frame.
- Actual IMU and wheel odometry topics and frames.
- Physical robot TF tree and Nav2 params.
- The robot's ROS domain ID, DDS implementation, and Wi-Fi network arrangement.
- Emergency-stop procedure and conservative speed limits.

### Required for automatic monocular object localization

- Time-synchronized RGB frames, `CameraInfo`, robot poses, and TF.
- A short rosbag with camera motion around static objects.
- A decision on the first supported object classes.
- Object-position uncertainty thresholds that block unsafe navigation.

### Environment work still needed

- Create a dedicated Python environment for Chat-Scene.
- Install a Chat-Scene-compatible PyTorch build after checking the provided upstream environment.
- Free enough GPU memory for model loading and measure actual latency and VRAM use.
- Rebuild copied ROS2 packages in this repository instead of relying on old `~/dev_ws` build artifacts.

## 6. Document Corrections

`BUILD_PROMPTS.md` is a sensible staged implementation plan. It correctly keeps Chat-Scene away from `cmd_vel`, delays ML integration until Nav2 works, and treats monocular localization conservatively.

`PROJECT_OVERVIEW_CN.md` section 12 is stale for this environment: this Codex session is already running inside WSL2 and can read `~/dev_ws` directly. It should be updated after the ROS2 baseline packages are copied into this repository.

The documents should also explicitly distinguish:

- the reusable simulation RGB-D camera;
- the expected physical monocular RGB camera;
- the need to provide an AMCL initial pose before Nav2 navigation tests.

## 7. Minimal Next Implementation Step

Add a single bringup command that starts the copied Gazebo and Nav2 stack with a robust controller-loading sequence and the documented initial-pose step.

After that baseline is reproducible, implement the deterministic semantic navigation slice from prompts 01 through 05 before adding perception or Chat-Scene.
