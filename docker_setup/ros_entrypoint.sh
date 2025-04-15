#!/bin/bash
set -e

# Source ROS 2 Humble environment
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  echo "Sourced ROS 2 Humble: /opt/ros/humble/setup.bash"
else
  echo "Warning: ROS 2 Humble setup file not found."
fi

# Source ORB-SLAM3 workspace
if [ -f /root/ros2_test/install/setup.bash ]; then
  source /root/ros2_test/install/setup.bash
  echo "Sourced workspace: /root/ros2_test/install/setup.bash"
fi

# Source PX4 ROS Com workspace
if [ -f /root/horus/ws_px4/install/setup.bash ]; then
  source /root/horus/ws_px4/install/setup.bash
  echo "Sourced workspace: /root/horus/ws_px4/install/setup.bash"
fi

# Execute the command passed into the container
exec "$@"
