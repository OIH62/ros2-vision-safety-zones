# Contributing

Keep the ROS-free classifier independent of ROS types. New camera support should
use topic/config adapters rather than adding a vendor SDK to this repository.

Before opening a pull request:

```bash
source scripts/ros_env.sh && source_ros2
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/person_pose \
  python3 -m pytest -q src/person_pose/test
```

Do not commit model weights, camera SDK binaries, recordings containing people,
credentials, absolute home paths, or generated `build/install/log` directories.
