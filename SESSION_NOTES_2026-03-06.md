# Mini Pupper Session Notes

Date: 2026-03-06

## Goal

Get the Mini Pupper working from this desktop for:

- reading sensors/state over ROS 2
- moving the robot from this machine
- understanding the robot's current software and hardware state

## Environment

- Desktop: Windows, PowerShell, WSL available but no ROS 2 installed in WSL
- Robot IP used in session: `<robot-ip>`
- Robot login: `ubuntu` / stock image default password
- Robot OS: Ubuntu 22.04
- Robot ROS workspace: `~/ros2_ws`
- Robot model used: `mini_pupper`
- ROS domain used: `42`

## What Was Confirmed

### 1. Desktop-to-robot connectivity works

- SSH from the desktop to the robot works.
- The robot is reachable over Wi-Fi from this machine.

### 2. ROS 2 packages are installed on the robot

The robot has:

- `mini_pupper_bringup`
- `mini_pupper_driver`
- `mini_pupper_dance`
- `mini_pupper_slam`
- CHAMP packages

The ROS 2 workspace exists and can be sourced from:

- `/opt/ros/humble/setup.bash`
- `~/ros2_ws/install/setup.bash`

### 3. A working ROS graph was achieved from the desktop

A desktop-managed worker process was implemented in:

- [minipupper_test_cli.py](minipupper_test_cli.py)

That worker keeps two SSH sessions open to the robot:

- `mini_pupper_bringup bringup.launch.py hardware_connected:=False`
- `mini_pupper_driver servo_interface`

With that worker running, the following ROS topics were successfully observed:

- `/joint_states`
- `/joint_group_effort_controller/joint_trajectory`
- `/cmd_vel`
- `/odom`
- `/imu/data` in some runs

### 4. Desktop ROS-state reads work

The CLI successfully read live robot state from the desktop, especially:

- `/joint_states`
- `/joint_group_effort_controller/joint_trajectory`

### 5. ROS command flow works

The desktop successfully published:

- `/cmd_vel`
- direct ROS `JointTrajectory` messages to `/joint_group_effort_controller/joint_trajectory`

This means the desktop-to-ROS command path is functional.

## Major Findings

### 1. Built-in robot startup is incomplete

The installed built-in service on the robot is:

- `/etc/systemd/system/robot.service`

It runs:

- `/var/lib/minipupper/run.sh`

But `run.sh` only shows the IP address and contains:

- `#TODO: Add ros2 launch`

So the stock image is not actually set up to keep the ROS control stack running by itself.

### 2. `mini_pupper_bringup` hardware path was not reliable as-is

Using:

- `ros2 launch mini_pupper_bringup bringup.launch.py`

with full hardware enabled did not produce a stable usable graph.

The path that did work was:

- `ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=False`

plus:

- `ros2 run mini_pupper_driver servo_interface`

### 3. The dance package is broken on the robot

The robot has `mini_pupper_dance` installed, but it is not runnable in its current state.

Failures found:

- installed `dance.launch.py` is invalid
- `service` entry point has no `main`
- `client` entry point has no `main`
- `pose_controller` entry point has no `main`

So the built-in dance script/package currently needs source fixes and rebuild before it can work.

### 4. Calibration storage was mismatched

This was one of the biggest issues found.

The robot's calibration EEPROM/NVRAM contained real calibration values, but the installed Python code expected a pickle object while the stored data was plain text.

Observed raw calibration values:

```text
[[  9,  13,  -2,  -2],
 [ 39,  38,  36,  47],
 [-52, -59, -61, -47]]
```

This mismatch caused the active Mini Pupper Python stack to fall back to zero neutral angles, which made the robot behave incorrectly.

The EEPROM data was converted into the format expected by the installed `MangDang.mini_pupper` Python package.

After that, the installed `ServoCalibration.py` correctly reported:

```text
[[  9.  13.  -2.  -2.]
 [ 39.  38.  36.  47.]
 [-52. -59. -61. -47.]]
```

### 5. CHAMP default posture assumptions did not match the robot well

Even after ROS control was working, default CHAMP posture/body pose behavior did not map cleanly to the real robot.

Symptoms:

- robot stayed overly tucked
- `/body_pose` did not produce useful posture corrections in practice
- direct `JointTrajectory` control was more predictable than `/body_pose`

### 6. Direct hardware control worked better for posture recovery

Using the installed Mini Pupper hardware API directly via:

- `MangDang.mini_pupper.HardwareInterface`

was the most reliable path for:

- neutral pose
- slow stand
- sit
- lay

This was used to recover useful poses while ROS posture control was still being tuned.

## Mechanical / Hardware Findings

### 1. Front-left leg issue

During diagnosis, the front-left leg showed mechanical problems.

Observed behavior:

- the lower segment moved correctly
- the upper segment did not move as expected
- resistance felt abnormal compared to the other legs

Conclusion:

- likely mechanical or servo issue, not just software

The user repaired that leg during the session.

### 2. Power supply issue

The robot repeatedly shut down or rebooted when run from an external bench supply setup.

Important detail from the user:

- OEM battery works
- external variable supply with alligator clips causes shutdowns under motion

Most likely causes discussed:

- current limiting
- voltage sag under servo load
- poor transient response
- poor connector quality from alligator clips

This remains an important real-world blocker for aggressive motion.

### 3. Battery monitor errors were seen in logs

Journal logs repeatedly showed:

- `max1720x_battery` Remote I/O errors
- battery monitor script errors

These were visible around boot and external-power use, though they were not the only cause of the robot behavior.

## Files Created / Updated Locally

### Added

- [minipupper_test_cli.py](minipupper_test_cli.py)
- [robot_test_config.json](robot_test_config.json)
- [README-minipupper-test-control.md](README-minipupper-test-control.md)

### Purpose

`minipupper_test_cli.py` currently supports:

- `bringup_robot`
- `read_state`
- `move_robot`
- `set_posture`
- `set_joint_pose`
- `stop_robot`
- `show_config`

The most important practical command path is:

1. `bringup_robot`
2. `read_state`
3. `set_joint_pose --preset ...`
4. `move_robot`

## What Worked Reliably By End Of Session

### From the desktop

- SSH to robot
- Start a ROS worker that keeps the robot graph alive
- Read ROS state topics
- Publish ROS `cmd_vel`
- Publish ROS `JointTrajectory` posture presets
- Send direct hardware poses over SSH when needed

## Additional Demo Bringup Findings

### 1. Camera demo is present but broken on the current robot image

The robot includes:

- `mini_pupper_driver/launch/camera.launch.py`
- ROS topic names such as `/image_raw` and `/image_raw/compressed`

But the node does not actually start streaming frames.

Observed failures:

- `v4l2_camera`: `Failed stream start: Invalid argument (22)`
- `v4l2-ctl`: `VIDIOC_STREAMON returned -1 (Invalid argument)`
- `fswebcam`: falls back from `mmap` to `read`, then captures no frames
- Ubuntu `libcamera-tools` `cam --list`: segmentation fault

Conclusion:

- camera support is currently blocked by the robot image / camera stack, not by missing desktop control code

### 2. SLAM package is installed but LiDAR is not operational

The robot includes:

- `mini_pupper_slam`
- `ldlidar_stl_ros2`

Important detail:

- `mini_pupper_slam` expects `/scan`

However `/scan` is not available with the working bringup path.

The installed LiDAR launch defaults to:

- `/dev/ttyUSB0`

But on this robot the visible serial candidates are:

- `/dev/ttyS0` via `/dev/serial0`
- `/dev/ttyAMA0` via `/dev/serial1`

Tests performed:

- `ldlidar_stl_ros2` on `/dev/ttyUSB0`: device missing
- `ldlidar_stl_ros2` on `/dev/serial0`: opens under `sudo`, then `ldlidar communication is abnormal`
- `ldlidar_stl_ros2` on `/dev/ttyAMA0` after stopping Bluetooth: same abnormal communication
- all supported profiles tested (`LD06`, `LD19`, `STL27L`): same result

Conclusion:

- the LiDAR serial configuration or underlying image setup is wrong, so SLAM cannot run end-to-end yet

### 2a. LiDAR was later fixed by replacing the USB cable

On March 7, 2026, the LiDAR USB cable was replaced.

After that change:

- `lsusb` showed `10c4:ea60 Silicon Labs CP210x UART Bridge`
- `/dev/ttyUSB0` appeared
- `ldlidar_stl_ros2` launched successfully on the stock `ld06.launch.py`
- logs showed `ldlidar communication is normal`
- `/scan` became available in ROS 2

### 2b. Stock SLAM backend works headless without RViz

The installed `mini_pupper_slam` package still fails if launched exactly as shipped on this robot image because:

- `rviz2` is not installed on the robot

However, the stock Cartographer configuration works when launched headless:

- `cartographer_node` with `mini_pupper_slam/config/slam.lua`
- `cartographer_occupancy_grid_node`

Verified topics:

- `/scan`
- `/submap_list`
- `/map`

This path was wrapped into the desktop helper as:

- `python .\minipupper_test_cli.py start_slam`
- `python .\minipupper_test_cli.py slam_status`
- `python .\minipupper_test_cli.py stop_slam`

### 3. ROS apt repository on the robot is stale

While trying to install alternate camera packages, the robot reported:

- expired ROS apt key (`EXPKEYSIG F42ED6FBAB17C654`)
- cached package URLs returning `404`

This did not block the already-installed ROS stack, but it does block clean installation of newer ROS camera packages until the ROS apt repo config is repaired.

### Robot poses that were meaningfully tested

Using direct hardware path:

- neutral pose
- modest stand
- sit
- sit with more crouched rear legs
- lay

Using ROS path:

- `JointTrajectory` presets can be published
- `stand` preset was sent successfully over ROS

## What Is Still Not Fully Solved

### 1. ROS-native locomotion tuning

ROS transport works, but the locomotion stack is not yet tuned enough to trust normal `cmd_vel` walking behavior.

Reasons:

- CHAMP defaults do not match this robot well
- posture defaults were too tucked
- robot hardware/calibration was only partially recovered during the session

### 2. ROS-native posture layer

`/body_pose` is subscribed by the controller, but it did not produce practical posture changes that matched the physical robot during this session.

Direct ROS `JointTrajectory` presets were more reliable.

### 3. Built-in dance package

Broken and needs repair/rebuild.

### 4. Full interactive calibration

Not completed. The built-in calibration flow is interactive and requires a person at the robot to verify each joint.

## Recommended Next Steps

### Highest priority

1. Use OEM battery or a much better external power connection before testing heavier motion.
2. Keep using the desktop-managed ROS worker, not the robot's unfinished built-in startup path.
3. Treat direct ROS `JointTrajectory` presets as the current posture interface.

### Then

4. Add named ROS posture presets as the main control interface:
   - neutral
   - stand
   - sit
   - lay
5. Tune those presets based on the repaired hardware.
6. Only after posture is solid, resume `cmd_vel` gait tuning.

### Separate track

7. Repair the `mini_pupper_dance` package in source and rebuild it.
8. Perform a full manual servo calibration if posture symmetry remains off.

## Short Takeaway

The desktop-to-robot ROS pipeline now works.

The biggest real issues discovered were:

- incomplete built-in robot startup
- broken dance package
- calibration format mismatch in EEPROM/NVRAM handling
- CHAMP posture assumptions not matching the physical robot
- power instability under external supply
- at least one leg having had a mechanical problem

The project is now past "can't connect / can't control anything" and into "control exists, but posture/gait/power need tuning."
