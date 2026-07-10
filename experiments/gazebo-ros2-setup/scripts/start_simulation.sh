#!/bin/bash

# Build and start all services
docker-compose up --build -d

echo "Services starting..."
echo "Gazebo Web UI will be available at: http://localhost:8081"
echo "noVNC Web Interface available at: http://localhost:6080"
echo "VNC direct connection: localhost:5900"

# Wait for services to be ready
sleep 10

# Launch Mini Pupper simulation
docker-compose exec ros2-gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/install/setup.bash &&
  ros2 launch mini_pupper_gazebo mini_pupper_world.launch.py
"