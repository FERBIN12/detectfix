---
inclusion: fileMatch
fileMatchPattern: "sim/**/*.py"
---
# ROS 2 conventions (Jazzy, rclpy inside Isaac's Python 3.12)

- Frames: map -> odom -> base_link -> {os0_lidar, gimbal_cam, thermal_cam}; the ROV uses the prefix `rov/` and topics under `/rov`.
- Sensor topics are BEST_EFFORT/VOLATILE; maps, missions and defects are RELIABLE/TRANSIENT_LOCAL so late RViz subscribers get them.
- Marker colours and scales must be Python floats (Jazzy skips type checks; ints crash the serializer).
- Tests: `source /opt/ros/jazzy/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ~/IsaacLab/env_isaaclab/bin/python -m pytest tests -q`.
