# Experiments

Side quests from getting a ROS 2 + Gazebo toolchain running next to the real
robot. None of these are needed for the operator tools.

| Folder | What it is |
| --- | --- |
| `gazebo-ros2-setup/` | Two-container Compose stack for Apple Silicon: a `humble-desktop-full` shell for building, and a noVNC desktop (browser at `http://localhost:6080/vnc.html`) for Gazebo and RViz. `scripts/setup.sh` pulls `mini_pupper_ros` and its `.repos` into `dev_ws/src`; `scripts/start_simulation.sh` brings the stack up and launches `mini_pupper_simulation gazebo.launch.py`. |
| `mini_pupper_sim/` | The single-container version: just the noVNC ROS 2 desktop with `./dev_ws` mounted. |
| `ros2-humble-devcontainer/` | Minimal `ros:humble-ros-base` image with colcon and `ROS_DOMAIN_ID=42`, for building `ros2_ws/` or talking to the robot over DDS from a container. |

```bash
cd experiments/gazebo-ros2-setup
./scripts/setup.sh
docker compose run --rm ros2-gazebo bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
./scripts/start_simulation.sh
```
