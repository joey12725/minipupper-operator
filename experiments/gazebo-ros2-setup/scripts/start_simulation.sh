#!/usr/bin/env bash
# Start the containers and launch the Mini Pupper Gazebo world.
# Run from experiments/gazebo-ros2-setup/ after scripts/setup.sh and a colcon build.
set -euo pipefail

docker compose up -d

echo "noVNC desktop: http://localhost:6080/vnc.html"
echo "Raw VNC:       vnc://localhost:5901"

sleep 10

docker compose exec ros2-gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 launch mini_pupper_simulation gazebo.launch.py
"
