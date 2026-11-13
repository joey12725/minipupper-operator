# Third-party notices

The code in this repository is original work by Joey Hersh and is released
under the [MIT License](LICENSE), except as listed below.

## Mini Pupper platform (MangDang)

This project is an operator layer for the [Mini Pupper](https://github.com/mangdangroboticsclub/mini_pupper)
robot by MangDang Technology Co., Limited, which is itself inspired by the
[Stanford Pupper](https://github.com/stanfordroboticsclub/StanfordQuadruped/).

- `sources/mini_pupper_ros` is a git submodule pointing at
  [mangdangroboticsclub/mini_pupper_ros](https://github.com/mangdangroboticsclub/mini_pupper_ros)
  (Apache License 2.0). No MangDang source is copied into this repository;
  the submodule pins the upstream revision the tools were developed against.
- The tools talk to that stack at runtime (package names, launch files,
  topic names, joint names, the Cartographer `slam.lua` config shipped in
  `mini_pupper_slam`). Those remain MangDang's work under Apache-2.0.
- `experiments/gazebo-ros2-setup/scripts/setup.sh` clones the upstream
  repository at setup time; it does not vendor it.

"Mini Pupper" and "MangDang" are names of their respective owners. This
project is not affiliated with or endorsed by MangDang.

## ROS 2 package boilerplate

`ros2_ws/src/pose_toggler/test/test_copyright.py`, `test_flake8.py` and
`test_pep257.py` are the standard `ros2 pkg create` lint tests, Copyright
Open Source Robotics Foundation, Inc., Apache License 2.0. Their headers are
kept intact.

## Container images

The files under `experiments/` reference public images (`ros:humble-ros-base`,
`osrf/ros:humble-desktop-full`, `tiryoh/ros2-desktop-vnc`) that are pulled at
run time under their own licenses.
