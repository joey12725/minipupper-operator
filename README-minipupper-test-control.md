# Mini Pupper Test Control

This is a disposable operator setup for bringing up a Mini Pupper 1, reading ROS2 state, and sending short motion tests from this Windows machine.

## Runtime config

All commands resolve the robot address in this order:

1. `--robot-ip`
2. `ROBOT_IP`
3. `robot_test_config.json`

Update the robot IP in exactly one of those places when it changes after reboot.

Default SSH user is `ubuntu`. Password can come from `ROBOT_PASSWORD`, from the config file, or from an interactive prompt.
The helper defaults `ROBOT_MODEL` to `mini_pupper` for Mini Pupper 1.
The checked-in config now targets the last verified robot IP `<robot-ip>` and defaults to `ssh` mode because ROS 2 is not installed in local WSL.

## Quick start

Set the robot IP for the current PowerShell session:

```powershell
$env:ROBOT_IP="<robot-ip>"
```

Show the resolved config:

```powershell
python .\minipupper_test_cli.py show_config
```

Start bringup on the robot:

```powershell
python .\minipupper_test_cli.py bringup_robot
```

What `bringup_robot` actually does now:

- starts a detached desktop-side worker process on this Windows machine
- keeps a live SSH session open to the robot for `mini_pupper_bringup`
- keeps a second live SSH session open for `servo_interface`
- uses `hardware_connected:=False` because that path is the one that was verified to produce a working ROS2 graph on this robot
- verifies `/cmd_vel`, `/joint_states`, `/joint_group_effort_controller/joint_trajectory`, and `/odom`

Show worker status:

```powershell
python .\minipupper_test_cli.py show_config
```

Read one round of discovered state topics:

```powershell
python .\minipupper_test_cli.py read_state --once
```

Move forward briefly:

```powershell
python .\minipupper_test_cli.py move_robot --linear-x 0.15 --angular-z 0.0 --duration-sec 1.5
```

Rotate briefly:

```powershell
python .\minipupper_test_cli.py move_robot --linear-x 0.0 --angular-z 0.6 --duration-sec 1.0
```

Start headless SLAM after `bringup_robot` is already running:

```powershell
python .\minipupper_test_cli.py start_slam
```

Show recorded SLAM PIDs/log paths:

```powershell
python .\minipupper_test_cli.py slam_status
```

Stop the robot-side LiDAR + Cartographer SLAM processes:

```powershell
python .\minipupper_test_cli.py stop_slam
```

Launch the local keyboard teleop + SLAM map viewer:

```powershell
$env:ROBOT_PASSWORD='<robot-password>'
python .\minipupper_slam_operator.py
```

Emergency stop:

```powershell
python .\minipupper_test_cli.py stop_robot
```

Force SSH fallback:

```powershell
python .\minipupper_test_cli.py --mode ssh read_state --once
python .\minipupper_test_cli.py --mode ssh move_robot --linear-x 0.10 --angular-z 0.0 --duration-sec 1.0
```

Force DDS mode after ROS2 Humble is installed in WSL:

```powershell
python .\minipupper_test_cli.py --mode dds read_state --once
```

## Current sensor status

As of March 6, 2026 on robot `<robot-ip>`:

- Desktop-to-robot control works through `bringup_robot`, `move_robot`, `set_joint_pose`, and `stop_robot`.
- The LiDAR driver package `ldlidar_stl_ros2` is now confirmed working on `/dev/ttyUSB0` after replacing the LiDAR USB cable. Logs show `ldlidar communication is normal` and `/scan` appears in the ROS graph.
- Headless Cartographer SLAM is now confirmed working from this desktop helper through `start_slam`. Verified topics include `/scan`, `/submap_list`, and `/map`.
- A local Windows operator app now exists at `minipupper_slam_operator.py` for W/A/S/D teleop while polling and displaying the live `/map` output.
- The stock ROS camera launch exists at `mini_pupper_driver camera.launch.py`, but the current robot image still fails camera stream startup with `Failed stream start: Invalid argument (22)`.
- Direct camera capture was also tested with `v4l2-ctl`, `fswebcam`, and Ubuntu `libcamera-tools`; all still fail on this image, so browser/video viewing remains blocked by the robot image's camera stack rather than this desktop helper.

That means this workspace can now reliably control the robot and run LiDAR-backed headless SLAM from the desktop, but camera viewing still requires robot-side image fixes before an existing demo can run end-to-end.

## Modes

- `auto`: tries WSL DDS first, then falls back to SSH if `ros2` is missing or topic discovery fails
- `dds`: requires ROS2 Humble in WSL `Ubuntu`
- `ssh`: uses Paramiko to run the ROS2 commands on the robot directly

## Notes

- The script prints the live topic list before sampling state so it can adapt to whichever sensors are actually present.
- State sampling prefers `/joint_states`, IMU, odometry, and `/scan`.
- Motion always sends an explicit zero `cmd_vel` after the requested duration.
- Desktop session-worker metadata is written to `.minipupper_session_meta.json`.
- Desktop session-worker output is written to `minipupper_session.log`.
