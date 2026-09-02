# Integration guide

## 1. Choose the launch level

`pipeline.launch.py` is the portable integration point. It starts pose, zone,
and optional debug nodes but never assumes a camera package.

```bash
ros2 launch person_zone_cpp pipeline.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/aligned_depth_to_color/camera_info \
  device:=auto \
  start_debug:=false
```

`letmc520.launch.py` is only a convenience wrapper. It expects an externally
installed `astra_camera` package with `launch/letmc520.launch.xml`.

## 2. Embed in another launch file

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

person_safety = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("person_zone_cpp"),
            "launch",
            "pipeline.launch.py",
        ])
    ),
    launch_arguments={
        "color_topic": "/front/rgb/image_raw",
        "depth_topic": "/front/depth/image_raw",
        "camera_info_topic": "/front/depth/camera_info",
        "device": "auto",
        "start_debug": "false",
    }.items(),
)
```

No node name, namespace, or topic is required to remain at its default; normal
ROS remapping and namespaces are supported.

## 3. Camera contract

- RGB: `sensor_msgs/msg/Image`, convertible by `cv_bridge` to `bgr8`.
- Depth: `sensor_msgs/msg/Image`, `16UC1` or `mono16`, values in millimetres.
- Camera info: `sensor_msgs/msg/CameraInfo` matching the depth image.
- RGB and depth should be registered/aligned. If they are not, the configured
  `depth_offset_x/y` is only a small calibration correction, not registration.
- Camera inputs use `rclcpp::SensorDataQoS()` / `qos_profile_sensor_data`
  (best effort, keep-last), which works with common camera drivers.

Zone output does not require depth. Missing depth affects XYZ and produces a
`DEPTH_STALE` diagnostic; it does not stop LEFT/CENTER/RIGHT classification.

## 4. Stable output contract

| Name | Type | Values |
|---|---|---|
| `/person_zone_state` | `std_msgs/String` | `LEFT`, `CENTER`, `RIGHT`, `NONE` |
| `/person_position` | `geometry_msgs/PointStamped` | metres in depth optical frame |
| `/person_emergency` | `std_msgs/Bool` | latched in node logic until clear/ACK |
| `/person_exit_direction` | `std_msgs/String` | `LEFT/RIGHT/TOP/BOTTOM/NONE` |
| `/person_edge_warning` | `std_msgs/String` | active edge or `NONE` |
| `/person_exit_event` | `std_msgs/String` | one JSON object per confirmed exit |
| `/person_zone/diagnostics` | `std_msgs/String` | `OK` or stale flags |
| `/person_pose/device` | `std_msgs/String` | `cpu`, `cuda:0`, ... |

The emergency publisher is a software state output, not a hardware-rated
emergency stop. A downstream controller should define its own timeout and
fail-safe behavior if messages stop.

## 5. CPU/GPU deployment

`device:=auto` is the recommended default. Selection and recovery are separate:

1. At startup, CUDA must report both availability and at least one device.
2. If GPU inference still raises (driver mismatch, OOM, unsupported operation),
   a fresh model is constructed and the same frame is retried on CPU.
3. Automatic FPS changes from `gpu_process_fps` to `cpu_process_fps`.
4. `/person_pose/device` and `/person_pose/diagnostics` expose the result.

Force deterministic CPU operation in a container or CI system with:

```bash
ros2 launch person_zone_cpp pipeline.launch.py device:=cpu
```

To treat a GPU failure as fatal, set `allow_cpu_fallback: false` in
`src/person_pose/config/pose.yaml` or a deployment-specific parameter file.

## 6. ROS-free C++ use

The installed header and library can be used without rclcpp:

```cpp
#include <person_zone_cpp/zone_core.hpp>

person_zone_cpp::ZoneTracker tracker;
std::vector<person_zone_cpp::Keypoint> keypoints = /* detector output */;
auto observation = tracker.observe(keypoints, image_width, image_height);
auto state = tracker.update(observation.ratios);
```

Link against `person_zone_core`. This is the preferred boundary for a custom
message adapter, a non-ROS process, or a different middleware.

## 7. Distribution upgrades

Build the repository in the target distribution rather than copying `build/`
or `install/` from another system. Interface packages generate distribution-
specific type support at build time. The project avoids distribution-specific
rclcpp APIs, but the camera driver and binary SDK must still match the target
OS and ROS distribution.
