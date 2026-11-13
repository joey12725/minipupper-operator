#!/usr/bin/env bash
# Fetch the Mini Pupper ROS 2 sources into ./dev_ws/src, which docker-compose.yml
# mounts into both containers. Run from experiments/gazebo-ros2-setup/.
set -euo pipefail

mkdir -p dev_ws/src
cd dev_ws/src

git clone --recursive -b ros2-dev \
  https://github.com/mangdangroboticsclub/mini_pupper_ros.git

cd mini_pupper_ros
export PATH="$HOME/.local/bin:$PATH"
vcs import < .minipupper.repos --recursive

echo "Source tree ready. Start the stack with: ./scripts/start_simulation.sh"
