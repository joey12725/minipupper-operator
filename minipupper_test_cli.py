#!/usr/bin/env python3
from __future__ import annotations

import argparse
import getpass
import json
import os
import signal
import shlex
import subprocess
import sys
import textwrap
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import paramiko


WORKSPACE_DIR = Path(__file__).resolve().parent
CONFIG_PATH = WORKSPACE_DIR / "robot_test_config.json"
DEFAULT_DOMAIN_ID = "42"
PLACEHOLDER_IP = "<robot-ip>"
DEFAULT_SSH_USER = "ubuntu"
DEFAULT_ROBOT_MODEL = "mini_pupper"
WORKER_PID_PATH = WORKSPACE_DIR / ".minipupper_session_pid"
WORKER_META_PATH = WORKSPACE_DIR / ".minipupper_session_meta.json"
WORKER_LOG_PATH = WORKSPACE_DIR / "minipupper_session.log"
SLAM_META_PATH = WORKSPACE_DIR / ".minipupper_slam_meta.json"

STATE_TOPIC_PRIORITIES = {
    "joint_states": ["/joint_states"],
    "imu": ["/imu/data", "/imu", "/imu/data_raw"],
    "odom": ["/odom", "/odometry/filtered"],
    "scan": ["/scan"],
}

JOINT_NAMES = [
    "base_lf1",
    "lf1_lf2",
    "lf2_lf3",
    "base_rf1",
    "rf1_rf2",
    "rf2_rf3",
    "base_lb1",
    "lb1_lb2",
    "lb2_lb3",
    "base_rb1",
    "rb1_rb2",
    "rb2_rb3",
]

JOINT_POSE_PRESETS_DEG = {
    "neutral": [0.0, 15.0, -20.0, 0.0, 15.0, -20.0, 0.0, 15.0, -20.0, 0.0, 15.0, -20.0],
    "stand": [0.0, 8.0, -12.0, 0.0, 8.0, -12.0, 0.0, 8.0, -12.0, 0.0, 8.0, -12.0],
    "sit": [0.0, 25.0, -35.0, 0.0, 25.0, -35.0, 0.0, 40.0, -55.0, 0.0, 40.0, -55.0],
    "lay": [0.0, 40.0, -55.0, 0.0, 40.0, -55.0, 0.0, 40.0, -55.0, 0.0, 40.0, -55.0],
}


class CliError(RuntimeError):
    pass


@dataclass
class RuntimeConfig:
    robot_ip: str
    ssh_user: str
    ssh_password: str | None
    ros_domain_id: str
    preferred_mode: str
    robot_model: str


def load_config_file() -> dict:
    if not CONFIG_PATH.exists():
        return {}
    try:
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise CliError(f"Invalid JSON in {CONFIG_PATH}: {exc}") from exc


def resolve_value(cli_value: str | None, env_name: str, config: dict, config_key: str, default: str | None = None) -> str | None:
    if cli_value:
        return cli_value
    env_value = os.environ.get(env_name)
    if env_value:
        return env_value
    file_value = config.get(config_key)
    if file_value:
        return str(file_value)
    return default


def resolve_runtime_config(args: argparse.Namespace, allow_placeholder_ip: bool = False) -> RuntimeConfig:
    config = load_config_file()
    robot_ip = resolve_value(args.robot_ip, "ROBOT_IP", config, "robot_ip", PLACEHOLDER_IP)
    if not allow_placeholder_ip and (not robot_ip or robot_ip == PLACEHOLDER_IP):
        raise CliError(
            "Robot IP is not configured. Set --robot-ip, ROBOT_IP, or update robot_test_config.json."
        )

    ssh_user = resolve_value(args.ssh_user, "ROBOT_SSH_USER", config, "ssh_user", DEFAULT_SSH_USER) or DEFAULT_SSH_USER
    ssh_password = resolve_value(None, "ROBOT_PASSWORD", config, "ssh_password", None)
    ros_domain_id = resolve_value(args.ros_domain_id, "ROS_DOMAIN_ID", config, "ros_domain_id", DEFAULT_DOMAIN_ID) or DEFAULT_DOMAIN_ID
    preferred_mode = resolve_value(args.mode, "MINIPUPPER_MODE", config, "preferred_mode", "auto") or "auto"
    robot_model = resolve_value(None, "ROBOT_MODEL", config, "robot_model", DEFAULT_ROBOT_MODEL) or DEFAULT_ROBOT_MODEL

    return RuntimeConfig(
        robot_ip=robot_ip,
        ssh_user=ssh_user,
        ssh_password=ssh_password,
        ros_domain_id=ros_domain_id,
        preferred_mode=preferred_mode,
        robot_model=robot_model,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Disposable Mini Pupper 1 operator CLI for bringup, state reads, and motion tests."
    )
    parser.add_argument("--robot-ip", help="Robot IP address.")
    parser.add_argument("--ssh-user", help="SSH username. Defaults to ubuntu.")
    parser.add_argument("--ros-domain-id", help="ROS_DOMAIN_ID for both robot and WSL DDS path.")
    parser.add_argument(
        "--mode",
        choices=["auto", "dds", "ssh"],
        help="Control mode. auto tries WSL DDS first and falls back to SSH.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("bringup_robot", help="Start Mini Pupper bringup on the robot over SSH.")

    read_state = subparsers.add_parser("read_state", help="List topics and sample the main state topics.")
    read_state.add_argument("--topic", default="auto", help="Specific topic to sample or auto.")
    read_state.add_argument("--once", action="store_true", help="Included for compatibility; reads one sample per topic.")

    move_robot = subparsers.add_parser("move_robot", help="Publish a bounded cmd_vel motion test.")
    move_robot.add_argument("--linear-x", type=float, default=0.0, required=False)
    move_robot.add_argument("--linear-y", type=float, default=0.0, required=False)
    move_robot.add_argument("--angular-z", type=float, default=0.0, required=False)
    move_robot.add_argument("--duration-sec", type=float, required=True)

    posture = subparsers.add_parser("set_posture", help="Set body posture through /body_pose.")
    posture.add_argument("--z-offset", type=float, required=True, help="Body height offset added to nominal height.")
    posture.add_argument("--x", type=float, default=0.0)
    posture.add_argument("--y", type=float, default=0.0)
    posture.add_argument("--roll", type=float, default=0.0, help="Roll in radians.")
    posture.add_argument("--pitch", type=float, default=0.0, help="Pitch in radians.")
    posture.add_argument("--yaw", type=float, default=0.0, help="Yaw in radians.")
    posture.add_argument("--hold-sec", type=float, default=2.0, help="How long to publish the pose command.")

    joint_pose = subparsers.add_parser("set_joint_pose", help="Publish a direct ROS JointTrajectory pose.")
    joint_pose.add_argument("--preset", choices=sorted(JOINT_POSE_PRESETS_DEG.keys()), required=True)
    joint_pose.add_argument("--hold-sec", type=float, default=1.5)

    subparsers.add_parser("start_slam", help="Start LiDAR and headless Cartographer SLAM on the robot.")
    subparsers.add_parser("stop_slam", help="Stop the robot-side LiDAR and Cartographer SLAM processes.")
    subparsers.add_parser("slam_status", help="Show the current recorded robot-side SLAM process state.")

    subparsers.add_parser("stop_robot", help="Publish an immediate zero-velocity stop.")
    subparsers.add_parser("show_config", help="Print the resolved runtime configuration.")
    subparsers.add_parser("session_worker", help=argparse.SUPPRESS)

    return parser


def maybe_prompt_for_password(runtime: RuntimeConfig) -> RuntimeConfig:
    if runtime.ssh_password:
        return runtime
    password = getpass.getpass(f"SSH password for {runtime.ssh_user}@{runtime.robot_ip}: ")
    return RuntimeConfig(
        robot_ip=runtime.robot_ip,
        ssh_user=runtime.ssh_user,
        ssh_password=password,
        ros_domain_id=runtime.ros_domain_id,
        preferred_mode=runtime.preferred_mode,
        robot_model=runtime.robot_model,
    )


def create_ssh_client(runtime: RuntimeConfig) -> paramiko.SSHClient:
    if not runtime.ssh_password:
        raise CliError("SSH password is not set for this operation.")
    last_error: Exception | None = None
    for attempt in range(1, 4):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                runtime.robot_ip,
                username=runtime.ssh_user,
                password=runtime.ssh_password,
                timeout=15,
                look_for_keys=False,
                allow_agent=False,
            )
            return client
        except Exception as exc:
            last_error = exc
            client.close()
            if attempt < 3:
                time.sleep(2)
    raise CliError(f"SSH connection to {runtime.ssh_user}@{runtime.robot_ip} failed after retries: {last_error}")


def run_remote_command(runtime: RuntimeConfig, command: str, timeout: int = 60) -> tuple[int, str, str]:
    client = create_ssh_client(runtime)
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        exit_code = stdout.channel.recv_exit_status()
        return exit_code, stdout.read().decode("utf-8", errors="replace"), stderr.read().decode("utf-8", errors="replace")
    finally:
        client.close()


def run_remote_command_checked(runtime: RuntimeConfig, command: str, timeout: int = 60, context: str = "Remote command") -> str:
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=timeout)
    ensure_success(exit_code, stdout, stderr, context)
    return stdout


def shell_join(lines: Iterable[str]) -> str:
    return " && ".join(lines)


def ros_env(runtime: RuntimeConfig) -> str:
    return textwrap.dedent(
        f"""
        export ROS_DOMAIN_ID={shlex.quote(runtime.ros_domain_id)}
        export ROBOT_MODEL={shlex.quote(runtime.robot_model)}
        if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
        if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi
        """
    ).strip()


def bashlc(script: str) -> str:
    return f"bash -lc {shlex.quote(script)}"


def ensure_success(exit_code: int, stdout: str, stderr: str, context: str) -> None:
    if exit_code != 0:
        raise CliError(f"{context} failed with exit code {exit_code}\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}")


def run_wsl_bash(script: str, timeout: int = 60) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["wsl.exe", "-d", "Ubuntu", "bash", "-lc", script],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def check_dds_available(runtime: RuntimeConfig) -> tuple[bool, str]:
    script = shell_join(
        [
            ros_env(runtime),
            "command -v ros2 >/dev/null",
            "ros2 topic list >/tmp/minipupper_topics.txt 2>/tmp/minipupper_topics.err",
            "cat /tmp/minipupper_topics.txt",
        ]
    )
    result = run_wsl_bash(script, timeout=25)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        return False, detail or "ros2 CLI not available in WSL"
    return True, result.stdout.strip()


def select_mode(runtime: RuntimeConfig) -> tuple[str, str | None]:
    if runtime.preferred_mode == "ssh":
        return "ssh", None
    if runtime.preferred_mode == "dds":
        ok, detail = check_dds_available(runtime)
        if not ok:
            raise CliError(f"DDS mode requested but unavailable: {detail}")
        return "dds", None

    ok, detail = check_dds_available(runtime)
    if ok:
        return "dds", None
    return "ssh", detail


def get_topic_list_ssh(runtime: RuntimeConfig) -> List[str]:
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            ros2 daemon stop >/dev/null 2>&1 || true
            for _ in 1 2 3 4 5 6 7 8 9 10; do
              topics="$(ros2 topic list 2>/dev/null || true)"
              if printf '%s\n' "$topics" | grep -q '^/cmd_vel$'; then
                printf '%s\n' "$topics"
                exit 0
              fi
              sleep 1
            done
            ros2 topic list
            """
        ).strip()
    )
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=45)
    ensure_success(exit_code, stdout, stderr, "Remote ros2 topic list")
    return [line.strip() for line in stdout.splitlines() if line.strip()]


def get_topic_list_dds(runtime: RuntimeConfig) -> List[str]:
    script = shell_join([ros_env(runtime), "ros2 topic list"])
    result = run_wsl_bash(script)
    if result.returncode != 0:
        raise CliError(f"WSL ros2 topic list failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def print_topics(topics: list[str]) -> None:
    print("Discovered topics:")
    for topic in topics:
        print(f"  {topic}")


def choose_topics(topics: list[str], explicit_topic: str) -> list[str]:
    if explicit_topic != "auto":
        return [explicit_topic]

    selected: list[str] = []
    for candidates in STATE_TOPIC_PRIORITIES.values():
        for candidate in candidates:
            if candidate in topics:
                selected.append(candidate)
                break
    return selected


def echo_topic_ssh(runtime: RuntimeConfig, topic: str) -> str:
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            ros2 daemon stop >/dev/null 2>&1 || true
            sleep 2
            ros2 topic echo {shlex.quote(topic)} --once
            """
        ).strip()
    )
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=40)
    ensure_success(exit_code, stdout, stderr, f"Remote ros2 topic echo {topic}")
    return stdout.strip()


def echo_topic_dds(runtime: RuntimeConfig, topic: str) -> str:
    script = shell_join([ros_env(runtime), f"ros2 topic echo {shlex.quote(topic)} --once"])
    result = run_wsl_bash(script, timeout=30)
    if result.returncode != 0:
        raise CliError(f"WSL ros2 topic echo {topic} failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")
    return result.stdout.strip()


def build_twist_message(linear_x: float, linear_y: float, angular_z: float) -> str:
    return (
        "{"
        f"linear: {{x: {linear_x:.4f}, y: {linear_y:.4f}, z: 0.0}}, "
        f"angular: {{x: 0.0, y: 0.0, z: {angular_z:.4f}}}"
        "}"
    )


def publish_twist_ssh(runtime: RuntimeConfig, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    twist = build_twist_message(linear_x, linear_y, angular_z)
    duration = max(duration_sec, 0.1)
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            ros2 daemon stop >/dev/null 2>&1 || true
            sleep 1
            timeout {duration:.2f}s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist {shlex.quote(twist)} >/dev/null 2>&1 || [ $? -eq 124 ]
            ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {shlex.quote(build_twist_message(0.0, 0.0, 0.0))}
            """
        ).strip()
    )
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=max(30, int(duration_sec) + 15))
    ensure_success(exit_code, stdout, stderr, "Remote cmd_vel publish")


def publish_twist_dds(runtime: RuntimeConfig, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    twist = build_twist_message(linear_x, linear_y, angular_z)
    duration = max(duration_sec, 0.1)
    script = shell_join(
        [
            ros_env(runtime),
            f"timeout {duration:.2f}s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist {shlex.quote(twist)}",
            f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {shlex.quote(build_twist_message(0.0, 0.0, 0.0))}",
        ]
    )
    result = run_wsl_bash(script, timeout=max(30, int(duration_sec) + 15))
    if result.returncode != 0:
        raise CliError(f"WSL cmd_vel publish failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = __import__("math").cos(yaw * 0.5)
    sy = __import__("math").sin(yaw * 0.5)
    cp = __import__("math").cos(pitch * 0.5)
    sp = __import__("math").sin(pitch * 0.5)
    cr = __import__("math").cos(roll * 0.5)
    sr = __import__("math").sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def build_pose_message(x: float, y: float, z_offset: float, roll: float, pitch: float, yaw: float) -> str:
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    return (
        "{"
        f"position: {{x: {x:.4f}, y: {y:.4f}, z: {z_offset:.4f}}}, "
        f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}"
        "}"
    )


def build_joint_trajectory_message(positions_deg: list[float]) -> str:
    positions_rad = [__import__("math").radians(value) for value in positions_deg]
    positions_text = ", ".join(f"{value:.6f}" for value in positions_rad)
    names_text = ", ".join(JOINT_NAMES)
    return (
        "{"
        "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, "
        f"joint_names: [{names_text}], "
        f"points: [{{positions: [{positions_text}], time_from_start: {{sec: 0, nanosec: 200000000}}}}]"
        "}"
    )


def publish_pose_ssh(runtime: RuntimeConfig, x: float, y: float, z_offset: float, roll: float, pitch: float, yaw: float, hold_sec: float) -> None:
    pose = build_pose_message(x, y, z_offset, roll, pitch, yaw)
    duration = max(hold_sec, 0.2)
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            ros2 daemon stop >/dev/null 2>&1 || true
            sleep 1
            timeout {duration:.2f}s ros2 topic pub -r 10 /body_pose geometry_msgs/msg/Pose {shlex.quote(pose)} >/dev/null 2>&1 || [ $? -eq 124 ]
            """
        ).strip()
    )
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=max(30, int(hold_sec) + 15))
    ensure_success(exit_code, stdout, stderr, "Remote body_pose publish")


def publish_pose_dds(runtime: RuntimeConfig, x: float, y: float, z_offset: float, roll: float, pitch: float, yaw: float, hold_sec: float) -> None:
    pose = build_pose_message(x, y, z_offset, roll, pitch, yaw)
    duration = max(hold_sec, 0.2)
    script = shell_join(
        [
            ros_env(runtime),
            f"timeout {duration:.2f}s ros2 topic pub -r 10 /body_pose geometry_msgs/msg/Pose {shlex.quote(pose)}",
        ]
    )
    result = run_wsl_bash(script, timeout=max(30, int(hold_sec) + 15))
    if result.returncode not in (0, 124):
        raise CliError(f"WSL body_pose publish failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")


def publish_joint_pose_ssh(runtime: RuntimeConfig, preset: str, hold_sec: float) -> None:
    msg = build_joint_trajectory_message(JOINT_POSE_PRESETS_DEG[preset])
    duration = max(hold_sec, 0.2)
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            ros2 daemon stop >/dev/null 2>&1 || true
            sleep 1
            timeout {duration:.2f}s ros2 topic pub -r 10 /joint_group_effort_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory {shlex.quote(msg)} >/dev/null 2>&1 || [ $? -eq 124 ]
            """
        ).strip()
    )
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=max(30, int(hold_sec) + 15))
    ensure_success(exit_code, stdout, stderr, "Remote joint trajectory publish")


def publish_joint_pose_dds(runtime: RuntimeConfig, preset: str, hold_sec: float) -> None:
    msg = build_joint_trajectory_message(JOINT_POSE_PRESETS_DEG[preset])
    duration = max(hold_sec, 0.2)
    script = shell_join(
        [
            ros_env(runtime),
            f"timeout {duration:.2f}s ros2 topic pub -r 10 /joint_group_effort_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory {shlex.quote(msg)}",
        ]
    )
    result = run_wsl_bash(script, timeout=max(30, int(hold_sec) + 15))
    if result.returncode not in (0, 124):
        raise CliError(f"WSL joint trajectory publish failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")


def process_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def load_worker_meta() -> dict:
    if not WORKER_META_PATH.exists():
        return {}
    try:
        return json.loads(WORKER_META_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}


def load_slam_meta() -> dict:
    if not SLAM_META_PATH.exists():
        return {}
    try:
        return json.loads(SLAM_META_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}


def write_slam_meta(meta: dict) -> None:
    SLAM_META_PATH.write_text(json.dumps(meta, indent=2), encoding="utf-8")


def clear_slam_meta() -> None:
    if SLAM_META_PATH.exists():
        SLAM_META_PATH.unlink()


def worker_running() -> bool:
    meta = load_worker_meta()
    pid = meta.get("pid")
    return isinstance(pid, int) and process_is_alive(pid)


def terminate_worker() -> None:
    meta = load_worker_meta()
    pid = meta.get("pid")
    if not isinstance(pid, int):
        return
    if not process_is_alive(pid):
        return
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True, text=True, check=False)
        else:
            os.kill(pid, signal.SIGTERM)
    except OSError:
        pass
    time.sleep(1)


def start_local_worker(runtime: RuntimeConfig) -> int:
    terminate_worker()
    env = os.environ.copy()
    env["ROBOT_IP"] = runtime.robot_ip
    env["ROBOT_SSH_USER"] = runtime.ssh_user
    env["ROBOT_PASSWORD"] = runtime.ssh_password or ""
    env["ROS_DOMAIN_ID"] = runtime.ros_domain_id
    env["ROBOT_MODEL"] = runtime.robot_model
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--robot-ip",
        runtime.robot_ip,
        "--ssh-user",
        runtime.ssh_user,
        "--ros-domain-id",
        runtime.ros_domain_id,
        "--mode",
        "ssh",
        "session_worker",
    ]
    log_handle = open(WORKER_LOG_PATH, "a", encoding="utf-8")
    kwargs: dict = {
        "args": cmd,
        "env": env,
        "stdin": subprocess.DEVNULL,
        "stdout": log_handle,
        "stderr": subprocess.STDOUT,
        "cwd": str(WORKSPACE_DIR),
        "close_fds": True,
    }
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS
    else:
        kwargs["start_new_session"] = True
    proc = subprocess.Popen(**kwargs)
    log_handle.close()
    return proc.pid


def write_worker_meta(runtime: RuntimeConfig, pid: int, status: str) -> None:
    WORKER_META_PATH.write_text(
        json.dumps(
            {
                "pid": pid,
                "status": status,
                "robot_ip": runtime.robot_ip,
                "ssh_user": runtime.ssh_user,
                "ros_domain_id": runtime.ros_domain_id,
                "robot_model": runtime.robot_model,
                "log_path": str(WORKER_LOG_PATH),
                "updated_at": time.time(),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    WORKER_PID_PATH.write_text(str(pid), encoding="utf-8")


def update_worker_status(status: str) -> None:
    meta = load_worker_meta()
    if not meta:
        return
    meta["status"] = status
    meta["updated_at"] = time.time()
    WORKER_META_PATH.write_text(json.dumps(meta, indent=2), encoding="utf-8")


def remote_topic_list(runtime: RuntimeConfig) -> list[str]:
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            ros2 daemon stop >/dev/null 2>&1 || true
            sleep 1
            ros2 topic list
            """
        ).strip()
    )
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=45)
    ensure_success(exit_code, stdout, stderr, "Remote ros2 topic list")
    return [line.strip() for line in stdout.splitlines() if line.strip()]


def remote_background_pid(runtime: RuntimeConfig, script: str, context: str) -> int:
    command = bashlc(
        textwrap.dedent(
            f"""
            {ros_env(runtime)}
            {script}
            """
        ).strip()
    )
    stdout = run_remote_command_checked(runtime, command, timeout=60, context=context).strip()
    try:
        return int(stdout.splitlines()[-1].strip())
    except (IndexError, ValueError) as exc:
        raise CliError(f"{context} did not return a valid PID. Output was: {stdout}") from exc


def stop_slam_processes(runtime: RuntimeConfig) -> None:
    meta = load_slam_meta()
    pids = [meta.get("occupancy_pid"), meta.get("cartographer_pid"), meta.get("lidar_pid")]
    live_pids = [str(pid) for pid in pids if isinstance(pid, int)]
    if live_pids:
        run_remote_command(
            runtime,
            bashlc(f"kill {' '.join(live_pids)} 2>/dev/null || true"),
            timeout=20,
        )
    clear_slam_meta()


def wait_for_remote_topics(runtime: RuntimeConfig, required_topics: list[str], timeout_sec: int) -> list[str]:
    deadline = time.time() + timeout_sec
    last_topics: list[str] = []
    while time.time() < deadline:
        try:
            last_topics = remote_topic_list(runtime)
            if all(topic in last_topics for topic in required_topics):
                return last_topics
        except CliError:
            pass
        time.sleep(1)
    return last_topics


def wait_for_topics(runtime: RuntimeConfig, timeout_sec: int = 20) -> list[str]:
    deadline = time.time() + timeout_sec
    last_topics: list[str] = []
    while time.time() < deadline:
        try:
            topics = get_topic_list_ssh(runtime)
            last_topics = topics
            if "/cmd_vel" in topics and "/joint_states" in topics:
                return topics
        except CliError:
            pass
        time.sleep(1)
    return last_topics


def drain_channel(channel: paramiko.ChannelFile, log_path: Path, prefix: str) -> None:
    with open(log_path, "a", encoding="utf-8") as handle:
        for line in iter(channel.readline, ""):
            handle.write(f"[{prefix}] {line}")
            handle.flush()


def run_session_worker(runtime: RuntimeConfig) -> int:
    runtime = maybe_prompt_for_password(runtime)
    pid = os.getpid()
    write_worker_meta(runtime, pid, "starting")
    bringup_client = create_ssh_client(runtime)
    servo_client = create_ssh_client(runtime)
    try:
        bringup_cmd = bashlc(
            shell_join(
                [
                    "source /opt/ros/humble/setup.bash",
                    "source ~/ros2_ws/install/setup.bash",
                    f"export ROS_DOMAIN_ID={shlex.quote(runtime.ros_domain_id)}",
                    f"export ROBOT_MODEL={shlex.quote(runtime.robot_model)}",
                    "exec ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=False",
                ]
            )
        )
        bringup_stdin, bringup_stdout, bringup_stderr = bringup_client.exec_command(bringup_cmd)
        threading.Thread(target=drain_channel, args=(bringup_stdout, WORKER_LOG_PATH, "bringup"), daemon=True).start()
        threading.Thread(target=drain_channel, args=(bringup_stderr, WORKER_LOG_PATH, "bringup"), daemon=True).start()

        topics = wait_for_topics(runtime, timeout_sec=20)
        if "/cmd_vel" not in topics:
            update_worker_status("failed")
            raise CliError("Remote bringup did not expose /cmd_vel within 20s. Check minipupper_session.log.")

        servo_cmd = bashlc(
            shell_join(
                [
                    "source /opt/ros/humble/setup.bash",
                    "source ~/ros2_ws/install/setup.bash",
                    f"export ROS_DOMAIN_ID={shlex.quote(runtime.ros_domain_id)}",
                    "exec ros2 run mini_pupper_driver servo_interface",
                ]
            )
        )
        servo_stdin, servo_stdout, servo_stderr = servo_client.exec_command(servo_cmd)
        threading.Thread(target=drain_channel, args=(servo_stdout, WORKER_LOG_PATH, "servo"), daemon=True).start()
        threading.Thread(target=drain_channel, args=(servo_stderr, WORKER_LOG_PATH, "servo"), daemon=True).start()
        update_worker_status("running")

        while True:
            if bringup_stdout.channel.exit_status_ready():
                update_worker_status("bringup_exited")
                return 1
            if servo_stdout.channel.exit_status_ready():
                update_worker_status("servo_exited")
                return 1
            time.sleep(1)
    finally:
        bringup_client.close()
        servo_client.close()


def handle_bringup(runtime: RuntimeConfig) -> None:
    runtime = maybe_prompt_for_password(runtime)
    inspect_command = bashlc(
        "printf 'HOSTNAME='; hostname; "
        "printf 'OS='; . /etc/os-release && printf '%s %s\\n' \"$NAME\" \"$VERSION_ID\"; "
        "command -v ros2 >/dev/null && echo ROS2=present || echo ROS2=missing; "
        "test -d ~/ros2_ws && echo ROS2_WS=present || echo ROS2_WS=missing; "
        "bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; source ~/ros2_ws/install/setup.bash 2>/dev/null; ros2 pkg list 2>/dev/null | grep -E \"^mini_pupper_bringup$\" >/dev/null && echo BRINGUP_PKG=present || echo BRINGUP_PKG=missing'"
    )
    exit_code, stdout, stderr = run_remote_command(runtime, inspect_command, timeout=30)
    ensure_success(exit_code, stdout, stderr, "Robot inspection")
    print(stdout.strip())
    pid = start_local_worker(runtime)
    write_worker_meta(runtime, pid, "starting")
    print(f"Started local desktop session worker PID {pid}.")
    print(f"Worker log: {WORKER_LOG_PATH}")
    print("The worker keeps remote bringup and servo_interface attached over SSH from this desktop.")

    deadline = time.time() + 30
    topics: list[str] = []
    while time.time() < deadline:
        if not process_is_alive(pid):
            raise CliError(f"Session worker exited early. Check {WORKER_LOG_PATH}.")
        try:
            topics = get_topic_list_ssh(runtime)
            if "/cmd_vel" in topics and "/joint_states" in topics and "/joint_group_effort_controller/joint_trajectory" in topics:
                break
        except CliError:
            pass
        time.sleep(1)
    required_topics = ["/cmd_vel", "/joint_states", "/joint_group_effort_controller/joint_trajectory", "/odom"]
    missing = [topic for topic in required_topics if topic not in topics]
    if missing:
        raise CliError(f"Bringup started but required topics are missing: {', '.join(missing)}")

    print("\nVerified topics:")
    for topic in required_topics:
        print(f"  {topic}")


def handle_read_state(runtime: RuntimeConfig, topic: str) -> None:
    mode, fallback_reason = select_mode(runtime)
    if fallback_reason:
        print(f"DDS unavailable, falling back to SSH: {fallback_reason}")
    if mode == "ssh":
        runtime = maybe_prompt_for_password(runtime)
        if not worker_running():
            raise CliError(f"No active desktop session worker. Run `bringup_robot` first. Log: {WORKER_LOG_PATH}")

    topics = get_topic_list_dds(runtime) if mode == "dds" else get_topic_list_ssh(runtime)
    print_topics(topics)

    selected_topics = choose_topics(topics, topic)
    if not selected_topics:
        print("\nNo priority state topics were found. Use --topic <name> to sample a specific topic.")
        return

    for chosen_topic in selected_topics:
        print(f"\n=== {chosen_topic} ===")
        payload = echo_topic_dds(runtime, chosen_topic) if mode == "dds" else echo_topic_ssh(runtime, chosen_topic)
        print(payload or "<no output>")


def handle_move(runtime: RuntimeConfig, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    if duration_sec <= 0:
        raise CliError("--duration-sec must be greater than 0")
    mode, fallback_reason = select_mode(runtime)
    if fallback_reason:
        print(f"DDS unavailable, falling back to SSH: {fallback_reason}")
    if mode == "ssh":
        runtime = maybe_prompt_for_password(runtime)
        if not worker_running():
            raise CliError(f"No active desktop session worker. Run `bringup_robot` first. Log: {WORKER_LOG_PATH}")

    if mode == "dds":
        publish_twist_dds(runtime, linear_x, linear_y, angular_z, duration_sec)
    else:
        publish_twist_ssh(runtime, linear_x, linear_y, angular_z, duration_sec)
    print(f"Published cmd_vel for {duration_sec:.2f}s and sent stop.")


def handle_posture(
    runtime: RuntimeConfig,
    x: float,
    y: float,
    z_offset: float,
    roll: float,
    pitch: float,
    yaw: float,
    hold_sec: float,
) -> None:
    mode, fallback_reason = select_mode(runtime)
    if fallback_reason:
        print(f"DDS unavailable, falling back to SSH: {fallback_reason}")
    if mode == "ssh":
        runtime = maybe_prompt_for_password(runtime)
        if not worker_running():
            raise CliError(f"No active desktop session worker. Run `bringup_robot` first. Log: {WORKER_LOG_PATH}")
        publish_pose_ssh(runtime, x, y, z_offset, roll, pitch, yaw, hold_sec)
    else:
        publish_pose_dds(runtime, x, y, z_offset, roll, pitch, yaw, hold_sec)
    print(f"Published body_pose z_offset={z_offset:.3f} for {hold_sec:.2f}s.")


def handle_joint_pose(runtime: RuntimeConfig, preset: str, hold_sec: float) -> None:
    mode, fallback_reason = select_mode(runtime)
    if fallback_reason:
        print(f"DDS unavailable, falling back to SSH: {fallback_reason}")
    if mode == "ssh":
        runtime = maybe_prompt_for_password(runtime)
        if not worker_running():
            raise CliError(f"No active desktop session worker. Run `bringup_robot` first. Log: {WORKER_LOG_PATH}")
        publish_joint_pose_ssh(runtime, preset, hold_sec)
    else:
        publish_joint_pose_dds(runtime, preset, hold_sec)
    print(f"Published ROS joint pose preset `{preset}` for {hold_sec:.2f}s.")


def handle_stop(runtime: RuntimeConfig) -> None:
    handle_move(runtime, 0.0, 0.0, 0.0, 0.2)


def handle_start_slam(runtime: RuntimeConfig) -> None:
    runtime = maybe_prompt_for_password(runtime)
    if not worker_running():
        raise CliError(f"No active desktop session worker. Run `bringup_robot` first. Log: {WORKER_LOG_PATH}")

    stop_slam_processes(runtime)

    lidar_pid = remote_background_pid(
        runtime,
        "nohup ros2 launch ldlidar_stl_ros2 ld06.launch.py >/tmp/minipupper_lidar.log 2>&1 < /dev/null & echo $!",
        "Start LiDAR",
    )
    time.sleep(3)

    cartographer_pid = remote_background_pid(
        runtime,
        textwrap.dedent(
            """
            config_dir="$(ros2 pkg prefix mini_pupper_slam)/share/mini_pupper_slam/config"
            nohup ros2 run cartographer_ros cartographer_node \
              -configuration_directory "$config_dir" \
              -configuration_basename slam.lua \
              --ros-args -r /imu/data:=/imu \
              >/tmp/minipupper_slam_cartographer.log 2>&1 < /dev/null & echo $!
            """
        ).strip(),
        "Start Cartographer",
    )

    occupancy_pid = remote_background_pid(
        runtime,
        "nohup ros2 run cartographer_ros cartographer_occupancy_grid_node >/tmp/minipupper_slam_occupancy.log 2>&1 < /dev/null & echo $!",
        "Start occupancy grid node",
    )

    topics = wait_for_remote_topics(runtime, ["/scan", "/submap_list", "/map"], timeout_sec=20)
    missing = [topic for topic in ["/scan", "/submap_list", "/map"] if topic not in topics]
    if missing:
        stop_slam_processes(runtime)
        raise CliError(
            "SLAM start did not expose required topics: "
            + ", ".join(missing)
            + ". Check /tmp/minipupper_lidar.log, /tmp/minipupper_slam_cartographer.log, and /tmp/minipupper_slam_occupancy.log on the robot."
        )

    write_slam_meta(
        {
            "robot_ip": runtime.robot_ip,
            "lidar_pid": lidar_pid,
            "cartographer_pid": cartographer_pid,
            "occupancy_pid": occupancy_pid,
            "updated_at": time.time(),
            "logs": {
                "lidar": "/tmp/minipupper_lidar.log",
                "cartographer": "/tmp/minipupper_slam_cartographer.log",
                "occupancy": "/tmp/minipupper_slam_occupancy.log",
            },
        }
    )

    print("Started headless SLAM on the robot.")
    print("Verified topics:")
    for topic in ["/scan", "/submap_list", "/map"]:
        print(f"  {topic}")


def handle_stop_slam(runtime: RuntimeConfig) -> None:
    runtime = maybe_prompt_for_password(runtime)
    stop_slam_processes(runtime)
    print("Stopped recorded robot-side SLAM processes.")


def handle_slam_status(runtime: RuntimeConfig) -> None:
    meta = load_slam_meta()
    print(json.dumps(meta or {"status": "not_running"}, indent=2))


def handle_show_config(runtime: RuntimeConfig) -> None:
    visible_password = "<set>" if runtime.ssh_password else "<prompt>"
    print(
        json.dumps(
            {
                "robot_ip": runtime.robot_ip,
                "ssh_user": runtime.ssh_user,
                "ssh_password": visible_password,
                "ros_domain_id": runtime.ros_domain_id,
                "preferred_mode": runtime.preferred_mode,
                "robot_model": runtime.robot_model,
                "config_path": str(CONFIG_PATH),
                "worker_meta_path": str(WORKER_META_PATH),
                "worker_log_path": str(WORKER_LOG_PATH),
                "worker_running": worker_running(),
            },
            indent=2,
        )
    )


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        runtime = resolve_runtime_config(args, allow_placeholder_ip=(args.command == "show_config"))
        if args.command == "bringup_robot":
            handle_bringup(runtime)
        elif args.command == "read_state":
            handle_read_state(runtime, args.topic)
        elif args.command == "move_robot":
            handle_move(runtime, args.linear_x, args.linear_y, args.angular_z, args.duration_sec)
        elif args.command == "set_posture":
            handle_posture(runtime, args.x, args.y, args.z_offset, args.roll, args.pitch, args.yaw, args.hold_sec)
        elif args.command == "set_joint_pose":
            handle_joint_pose(runtime, args.preset, args.hold_sec)
        elif args.command == "stop_robot":
            handle_stop(runtime)
        elif args.command == "start_slam":
            handle_start_slam(runtime)
        elif args.command == "stop_slam":
            handle_stop_slam(runtime)
        elif args.command == "slam_status":
            handle_slam_status(runtime)
        elif args.command == "show_config":
            handle_show_config(runtime)
        elif args.command == "session_worker":
            return run_session_worker(runtime)
        else:
            raise CliError(f"Unknown command: {args.command}")
        return 0
    except CliError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
