# LeTMC Person Zone for ROS 2

Camera-driver-independent ROS 2 packages for a three-way `LEFT / CENTER / RIGHT`
person safety test. The reference hardware is the LeTMC-520 (Astra Pro), but the
pipeline accepts standard ROS image topics and can be integrated with another
RGB-D camera without changing the classifier.

[한국어 안내](README.ko.md) · [Integration guide](docs/INTEGRATION.md) ·
[Operations](docs/RUNBOOK.md)

> This is a perception aid, not a certified safety controller. Never use a
> camera or neural-network result as the only emergency-stop mechanism.

## What is portable

- The C++17 `person_zone_core` library has no ROS dependency.
- The ROS adapter uses stable package-format 3 and standard ROS 2 messages for
  camera input and safety output.
- Topic names, model path, inference device, thresholds, and CSV output are
  parameters or launch arguments; no user home path is embedded.
- Scripts discover the sourced/installed ROS distribution instead of assuming
  `/opt/ros/humble`.
- CI builds on Humble, Jazzy, Kilted, Lyrical, and Rolling.
- YOLO device `auto` selects CUDA only when available. A CUDA inference failure
  reloads the model on CPU and continues at the CPU rate.

## Data flow

```text
RGB Image ──> YOLO Pose ──> PersonKeypoints ──> ROS-free zone core
Depth Image + CameraInfo ──────────────────────> XYZ / exit tracking
                                                     │
                 LEFT · CENTER · RIGHT · warning · diagnostics · debug image
```

The camera driver is deliberately not bundled. The repository that originally
contained this experiment included vendor code without a redistributable
license declaration, so the public project only consumes its standard topics.

## Packages

| Package | Purpose |
|---|---|
| `person_pose_msgs` | Small pose-keypoint interface |
| `person_pose` | One YOLO pose inference, GPU/CPU failover |
| `person_zone_cpp` | ROS-free core, ROS adapter, XYZ and warning logic |
| `person_zone_debug` | Optional three-zone overlay and status monitor |

## Compatibility

The maintained target is ROS 2 Humble or newer on Ubuntu. CI covers multiple
distributions, while camera-driver compatibility is verified separately by the
integrator because LeTMC/Astra drivers and SDK binaries vary by platform.

Requirements:

- a working ROS 2 installation, `colcon`, and `rosdep`
- Python 3 with `venv`
- an RGB topic (`sensor_msgs/msg/Image`)
- optional aligned depth image and camera info for XYZ
- a LeTMC-520/Astra driver only when using that camera

## Quick start

```bash
git clone https://github.com/OIH62/letmc-person-zone-ros2.git
cd letmc-person-zone-ros2

# Needed only when several ROS distros are installed.
export ROS_DISTRO=jazzy
./setup.sh
```

Start your camera driver, then run the portable pipeline:

```bash
./run_letmc.sh
```

Topic names can be supplied without editing a YAML file:

```bash
./run_letmc.sh \
  color_topic:=/rgb/image_raw \
  depth_topic:=/depth/image_raw \
  camera_info_topic:=/depth/camera_info
```

If an `astra_camera` package providing `letmc520.launch.xml` is already built in
the same environment, one command can start both camera and pipeline:

```bash
source install/setup.bash
ros2 launch person_zone_cpp letmc520.launch.py
```

The first run may download `yolov8n-pose.pt`. For offline use, place an
appropriately licensed model outside the repository and pass
`model_path:=/absolute/path/model.pt`.

## GPU and CPU behavior

Default configuration:

```yaml
device: auto
allow_cpu_fallback: true
process_fps: 0.0
gpu_process_fps: 15.0
cpu_process_fps: 3.0
```

- `auto`: use `cuda:0` only when PyTorch reports usable CUDA; otherwise CPU.
- CUDA runtime failure: recreate the model on CPU and retry the same frame.
- `cpu`: force CPU.
- `cuda:1`: request a specific device; falls back unless disabled.
- `process_fps > 0`: override both automatic rates.

Inspect the active device and recovery state:

```bash
ros2 topic echo /person_pose/device
ros2 topic echo /person_pose/diagnostics
```

## Integration contract

Inputs and outputs remain stable across ROS distributions:

| Interface | Type | Direction |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | input |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` (`16UC1`) | optional input |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | optional input |
| `/person_zone_state` | `std_msgs/msg/String` | output |
| `/person_position` | `geometry_msgs/msg/PointStamped` | output |
| `/person_emergency` | `std_msgs/msg/Bool` | output |
| `/person_zone/diagnostics` | `std_msgs/msg/String` | output |
| `/person_zone/debug_image` | `sensor_msgs/msg/Image` | optional output |

See [docs/INTEGRATION.md](docs/INTEGRATION.md) for package embedding, remapping,
QoS, CPU-only deployment, and the ROS-free C++ API.

## Test

```bash
source scripts/ros_env.sh && source_ros2
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/person_pose \
  python3 -m pytest -q src/person_pose/test
```

## Licensing

Project-authored source is Apache-2.0; see [LICENSE](LICENSE). Ultralytics,
PyTorch, model weights, and camera drivers are separate dependencies with their
own terms. Review [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) before
redistributing a combined product.
