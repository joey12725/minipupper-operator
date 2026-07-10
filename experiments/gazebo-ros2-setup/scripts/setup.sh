#!/usr/bin/env bash
set -e

# create the catkin-style workspace skeleton
mkdir -p workspace/src
cd workspace/src

# 1. main meta-repo
git clone --recursive -b ros2-dev \
      https://github.com/mangdangroboticsclub/mini_pupper_ros.git

# 2. pull all sub-repos declared in .minipupper.repos
cd mini_pupper_ros
export PATH="$HOME/.local/bin:$PATH"
vcs import < .minipupper.repos --recursive

echo "✔  Source tree ready.  Start the stack with:  docker-compose up"
