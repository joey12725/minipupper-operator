"""Desktop operator CLI for a Mini Pupper running the stock ROS 2 Humble image.

The robot's stock image does not keep its ROS graph alive on its own, so this
tool does it from the desktop: a detached worker holds two SSH sessions open
(bringup + servo interface), and every other command talks to that graph over
SSH or, when ROS 2 is installed locally, directly over DDS.
"""

from __future__ import annotations

import argparse
import dataclasses
import getpass
import json
import os
import shlex
import signal
import subprocess
import sys
import textwrap
import threading
import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from . import messages as msg

if TYPE_CHECKING:  # pragma: no cover
    import paramiko

PLACEHOLDER_IP = "<robot-ip>"
DEFAULT_DOMAIN_ID = "42"
DEFAULT_SSH_USER = "ubuntu"
DEFAULT_ROBOT_MODEL = "mini_pupper"
DEFAULT_CONFIG_NAME = "robot_config.json"

STATE_DIR = Path(os.environ.get("MINIPUPPER_STATE_DIR", ".minipupper")).resolve()
WORKER_PID_PATH = STATE_DIR / "session.pid"
WORKER_META_PATH = STATE_DIR / "session.json"
WORKER_LOG_PATH = STATE_DIR / "session.log"
SLAM_META_PATH = STATE_DIR / "slam.json"

BRINGUP_TOPICS = ["/cmd_vel", "/joint_states", "/joint_group_effort_controller/joint_trajectory", "/odom"]
SLAM_TOPICS = ["/scan", "/submap_list", "/map"]
JOINT_TRAJECTORY_TOPIC = "/joint_group_effort_controller/joint_trajectory"

ROBOT_LOGS = {
    "lidar": "/tmp/minipupper_lidar.log",
    "cartographer": "/tmp/minipupper_slam_cartographer.log",
    "occupancy": "/tmp/minipupper_slam_occupancy.log",
}

MOTION_COMMANDS = {"move_robot", "set_posture", "set_joint_pose", "stop_robot", "presets", "show_config"}

# Set by --dry-run. Motion commands print the exact shell they would run
# instead of connecting to the robot.
DRY_RUN = False


class CliError(RuntimeError):
    pass


@dataclass(frozen=True)
class RuntimeConfig:
    robot_ip: str
    ssh_user: str
    ssh_password: str | None
    ros_domain_id: str
    preferred_mode: str
    robot_model: str


# --------------------------------------------------------------------------- config


def config_path(cli_value: str | None = None) -> Path:
    return Path(cli_value or os.environ.get("MINIPUPPER_CONFIG") or DEFAULT_CONFIG_NAME)


def load_config_file(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise CliError(f"Invalid JSON in {path}: {exc}") from exc


def resolve_value(
    cli_value: str | None,
    env_name: str,
    config: dict,
    config_key: str,
    default: str | None = None,
) -> str | None:
    """CLI flag beats environment variable beats config file beats default."""
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
    config = load_config_file(config_path(getattr(args, "config", None)))
    robot_ip = resolve_value(args.robot_ip, "ROBOT_IP", config, "robot_ip", PLACEHOLDER_IP) or PLACEHOLDER_IP
    if not allow_placeholder_ip and robot_ip == PLACEHOLDER_IP:
        raise CliError(f"Robot IP is not configured. Set --robot-ip, ROBOT_IP, or robot_ip in {DEFAULT_CONFIG_NAME}.")

    return RuntimeConfig(
        robot_ip=robot_ip,
        ssh_user=resolve_value(args.ssh_user, "ROBOT_SSH_USER", config, "ssh_user") or DEFAULT_SSH_USER,
        ssh_password=resolve_value(None, "ROBOT_PASSWORD", config, "ssh_password"),
        ros_domain_id=resolve_value(args.ros_domain_id, "ROS_DOMAIN_ID", config, "ros_domain_id") or DEFAULT_DOMAIN_ID,
        preferred_mode=resolve_value(args.mode, "MINIPUPPER_MODE", config, "preferred_mode") or "auto",
        robot_model=resolve_value(None, "ROBOT_MODEL", config, "robot_model") or DEFAULT_ROBOT_MODEL,
    )


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", help=f"Path to the JSON config file (default: ./{DEFAULT_CONFIG_NAME}).")
    parser.add_argument("--robot-ip", help="Robot IP address or hostname.")
    parser.add_argument("--ssh-user", help="SSH username. Defaults to ubuntu.")
    parser.add_argument("--ros-domain-id", help="ROS_DOMAIN_ID shared by robot and desktop.")
    parser.add_argument(
        "--mode",
        choices=["auto", "dds", "ssh"],
        help="Control path. auto tries local ROS 2 (DDS) first and falls back to SSH.",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="minipupper-op",
        description="Bring up, drive, and map with a Mini Pupper from your desktop.",
    )
    add_connection_args(parser)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the commands a motion command would run on the robot instead of running them.",
    )

    sub = parser.add_subparsers(dest="command", required=True, metavar="command")

    sub.add_parser("bringup_robot", help="Start the ROS graph on the robot and keep it alive from this desktop.")

    read_state = sub.add_parser("read_state", help="List topics and sample joint state, IMU, odometry, and LiDAR.")
    read_state.add_argument("--topic", default="auto", help="Specific topic to sample, or auto.")
    read_state.add_argument("--once", action="store_true", help="Accepted for compatibility; always one sample.")

    move = sub.add_parser("move_robot", help="Publish /cmd_vel for a bounded time, then an explicit stop.")
    move.add_argument("--linear-x", type=float, default=0.0)
    move.add_argument("--linear-y", type=float, default=0.0)
    move.add_argument("--angular-z", type=float, default=0.0)
    move.add_argument("--duration-sec", type=float, required=True)

    posture = sub.add_parser("set_posture", help="Publish a body pose on /body_pose.")
    posture.add_argument("--z-offset", type=float, required=True, help="Body height offset from nominal, metres.")
    posture.add_argument("--x", type=float, default=0.0)
    posture.add_argument("--y", type=float, default=0.0)
    posture.add_argument("--roll", type=float, default=0.0, help="Radians.")
    posture.add_argument("--pitch", type=float, default=0.0, help="Radians.")
    posture.add_argument("--yaw", type=float, default=0.0, help="Radians.")
    posture.add_argument("--hold-sec", type=float, default=2.0)

    joint_pose = sub.add_parser("set_joint_pose", help="Send a named JointTrajectory preset (neutral, stand, sit, lay).")
    joint_pose.add_argument("--preset", choices=sorted(msg.JOINT_POSE_PRESETS_DEG), required=True)
    joint_pose.add_argument("--hold-sec", type=float, default=1.5)

    sub.add_parser("presets", help="List the joint pose presets.")
    sub.add_parser("start_slam", help="Start LiDAR + headless Cartographer SLAM on the robot.")
    sub.add_parser("stop_slam", help="Stop the robot-side SLAM processes started by start_slam.")
    sub.add_parser("slam_status", help="Show recorded SLAM process IDs and robot-side log paths.")
    sub.add_parser("stop_robot", help="Emergency stop: publish zero velocity immediately.")
    sub.add_parser("shutdown", help="Stop SLAM, send a zero-velocity stop, and end the session worker.")
    sub.add_parser("show_config", help="Print the resolved configuration.")
    sub.add_parser("session_worker")  # internal: spawned by bringup_robot
    return parser


def maybe_prompt_for_password(runtime: RuntimeConfig) -> RuntimeConfig:
    if runtime.ssh_password or DRY_RUN:
        return runtime
    password = getpass.getpass(f"SSH password for {runtime.ssh_user}@{runtime.robot_ip}: ")
    return dataclasses.replace(runtime, ssh_password=password)


# --------------------------------------------------------------------------- shell plumbing


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


def ros_script(runtime: RuntimeConfig, body: str) -> str:
    """Wrap a script in the ROS environment.

    `ros2 daemon stop` first: a stale daemon on the robot kept serving an old
    topic list after bringup restarts, which made discovery look broken.
    """
    return "\n".join([ros_env(runtime), "ros2 daemon stop >/dev/null 2>&1 || true", textwrap.dedent(body).strip()])


def ensure_success(exit_code: int, stdout: str, stderr: str, context: str) -> None:
    if exit_code != 0:
        raise CliError(f"{context} failed with exit code {exit_code}\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}")


def create_ssh_client(runtime: RuntimeConfig) -> paramiko.SSHClient:
    import paramiko

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
        except Exception as exc:  # noqa: BLE001 - paramiko raises a zoo of types
            last_error = exc
            client.close()
            if attempt < 3:
                time.sleep(2)
    raise CliError(f"SSH connection to {runtime.ssh_user}@{runtime.robot_ip} failed after retries: {last_error}")


def run_remote_command(runtime: RuntimeConfig, command: str, timeout: int = 60) -> tuple[int, str, str]:
    if DRY_RUN:
        print(f"[dry-run] ssh {runtime.ssh_user}@{runtime.robot_ip} {command}")
        return 0, "", ""
    client = create_ssh_client(runtime)
    try:
        _stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        exit_code = stdout.channel.recv_exit_status()
        return (
            exit_code,
            stdout.read().decode("utf-8", errors="replace"),
            stderr.read().decode("utf-8", errors="replace"),
        )
    finally:
        client.close()


def run_remote_command_checked(runtime: RuntimeConfig, command: str, timeout: int = 60, context: str = "Remote command") -> str:
    exit_code, stdout, stderr = run_remote_command(runtime, command, timeout=timeout)
    ensure_success(exit_code, stdout, stderr, context)
    return stdout


def run_remote_ros(runtime: RuntimeConfig, body: str, context: str, timeout: int = 60) -> str:
    return run_remote_command_checked(runtime, bashlc(ros_script(runtime, body)), timeout=timeout, context=context)


def local_bash_argv(script: str) -> list[str]:
    """Local ROS 2 lives in WSL on Windows and in the host shell everywhere else."""
    if os.name == "nt":
        distro = os.environ.get("MINIPUPPER_WSL_DISTRO", "Ubuntu")
        return ["wsl.exe", "-d", distro, "bash", "-lc", script]
    return ["bash", "-lc", script]


def run_local_ros(runtime: RuntimeConfig, script: str, timeout: int = 60) -> subprocess.CompletedProcess[str]:
    full = shell_join([ros_env(runtime), script])
    if DRY_RUN:
        print(f"[dry-run] {shlex.join(local_bash_argv(full))}")
        return subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")
    return subprocess.run(local_bash_argv(full), capture_output=True, text=True, timeout=timeout, check=False)


def check_dds_available(runtime: RuntimeConfig) -> tuple[bool, str]:
    try:
        result = run_local_ros(runtime, "command -v ros2 >/dev/null && ros2 topic list", timeout=25)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        return False, detail or "ros2 CLI not available locally"
    return True, result.stdout.strip()


def select_mode(runtime: RuntimeConfig) -> str:
    if runtime.preferred_mode == "ssh":
        return "ssh"
    if DRY_RUN:
        return "ssh" if runtime.preferred_mode == "auto" else runtime.preferred_mode
    ok, detail = check_dds_available(runtime)
    if ok:
        return "dds"
    if runtime.preferred_mode == "dds":
        raise CliError(f"DDS mode requested but unavailable: {detail}")
    print(f"Local ROS 2 unavailable, falling back to SSH: {detail}")
    return "ssh"


# --------------------------------------------------------------------------- topics and publishing


def split_lines(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if line.strip()]


def get_topic_list_ssh(runtime: RuntimeConfig) -> list[str]:
    # Discovery right after bringup is flaky; retry until /cmd_vel shows up.
    body = """
        for _ in 1 2 3 4 5 6 7 8 9 10; do
          topics="$(ros2 topic list 2>/dev/null || true)"
          if printf '%s\\n' "$topics" | grep -q '^/cmd_vel$'; then
            printf '%s\\n' "$topics"
            exit 0
          fi
          sleep 1
        done
        ros2 topic list
    """
    return split_lines(run_remote_ros(runtime, body, "Remote ros2 topic list", timeout=45))


def get_topic_list_dds(runtime: RuntimeConfig) -> list[str]:
    result = run_local_ros(runtime, "ros2 topic list")
    if result.returncode != 0:
        raise CliError(f"Local ros2 topic list failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")
    return split_lines(result.stdout)


def echo_topic(runtime: RuntimeConfig, mode: str, topic: str) -> str:
    if mode == "dds":
        result = run_local_ros(runtime, f"ros2 topic echo {shlex.quote(topic)} --once", timeout=30)
        if result.returncode != 0:
            raise CliError(f"Local ros2 topic echo {topic} failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")
        return result.stdout.strip()
    body = f"sleep 2\nros2 topic echo {shlex.quote(topic)} --once"
    return run_remote_ros(runtime, body, f"Remote ros2 topic echo {topic}", timeout=40).strip()


def timed_publish_command(topic: str, msg_type: str, payload: str, duration_sec: float) -> str:
    """Publish at 10 Hz for `duration_sec`. `timeout` exiting 124 is success."""
    return f"timeout {duration_sec:.2f}s ros2 topic pub -r 10 {topic} {msg_type} {shlex.quote(payload)} >/dev/null 2>&1 || [ $? -eq 124 ]"


def stop_command() -> str:
    return f"ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist {shlex.quote(msg.STOP_TWIST)}"


def publish(runtime: RuntimeConfig, mode: str, lines: Sequence[str], hold_sec: float, context: str) -> None:
    timeout = max(30, int(hold_sec) + 15)
    if mode == "dds":
        result = run_local_ros(runtime, "\n".join(lines), timeout=timeout)
        if result.returncode != 0:
            raise CliError(f"Local {context} failed\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}")
        return
    run_remote_ros(runtime, "sleep 1\n" + "\n".join(lines), f"Remote {context}", timeout=timeout)


def publish_twist(runtime: RuntimeConfig, mode: str, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    """Drive for `duration_sec`, then always follow with an explicit zero twist."""
    twist = msg.build_twist_message(linear_x, linear_y, angular_z)
    duration = max(duration_sec, 0.1)
    lines = [timed_publish_command("/cmd_vel", "geometry_msgs/msg/Twist", twist, duration), stop_command()]
    publish(runtime, mode, lines, duration_sec, "cmd_vel publish")


def publish_twist_ssh(runtime: RuntimeConfig, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    publish_twist(runtime, "ssh", linear_x, linear_y, angular_z, duration_sec)


# --------------------------------------------------------------------------- local state files


def process_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def read_json(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def worker_running() -> bool:
    pid = read_json(WORKER_META_PATH).get("pid")
    return isinstance(pid, int) and process_is_alive(pid)


def require_worker() -> None:
    if DRY_RUN:
        return
    if not worker_running():
        raise CliError(f"No active session worker. Run `bringup_robot` first. Log: {WORKER_LOG_PATH}")


def terminate_worker() -> None:
    pid = read_json(WORKER_META_PATH).get("pid")
    if not isinstance(pid, int) or not process_is_alive(pid):
        return
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True, text=True, check=False)
        else:
            os.kill(pid, signal.SIGTERM)
    except OSError:
        pass
    time.sleep(1)


def write_worker_meta(runtime: RuntimeConfig, pid: int, status: str) -> None:
    write_json(
        WORKER_META_PATH,
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
    )
    WORKER_PID_PATH.write_text(str(pid), encoding="utf-8")


def update_worker_status(status: str) -> None:
    meta = read_json(WORKER_META_PATH)
    if not meta:
        return
    meta["status"] = status
    meta["updated_at"] = time.time()
    write_json(WORKER_META_PATH, meta)


def start_local_worker(runtime: RuntimeConfig) -> int:
    """Spawn a detached copy of this CLI in `session_worker` mode."""
    terminate_worker()
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(
        {
            "ROBOT_IP": runtime.robot_ip,
            "ROBOT_SSH_USER": runtime.ssh_user,
            "ROBOT_PASSWORD": runtime.ssh_password or "",
            "ROS_DOMAIN_ID": runtime.ros_domain_id,
            "ROBOT_MODEL": runtime.robot_model,
            "MINIPUPPER_STATE_DIR": str(STATE_DIR),
        }
    )
    cmd = [sys.executable, "-m", "minipupper_operator.cli", "--mode", "ssh", "session_worker"]
    with open(WORKER_LOG_PATH, "a", encoding="utf-8") as log_handle:
        kwargs: dict = {
            "args": cmd,
            "env": env,
            "stdin": subprocess.DEVNULL,
            "stdout": log_handle,
            "stderr": subprocess.STDOUT,
            "close_fds": True,
        }
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.DETACHED_PROCESS
        else:
            kwargs["start_new_session"] = True
        proc = subprocess.Popen(**kwargs)
    return proc.pid


def drain_channel(channel, log_path: Path, prefix: str) -> None:
    with open(log_path, "a", encoding="utf-8") as handle:
        for line in iter(channel.readline, ""):
            handle.write(f"[{prefix}] {line}")
            handle.flush()


def wait_for_topics(runtime: RuntimeConfig, required: Sequence[str], timeout_sec: int, lister=None) -> list[str]:
    lister = lister or get_topic_list_ssh
    deadline = time.time() + timeout_sec
    last_topics: list[str] = []
    while time.time() < deadline:
        try:
            last_topics = lister(runtime)
            if not msg.missing_topics(last_topics, required):
                return last_topics
        except CliError:
            pass
        time.sleep(1)
    return last_topics


# --------------------------------------------------------------------------- session worker


def run_session_worker(runtime: RuntimeConfig) -> int:
    """Hold bringup and servo_interface open over two SSH sessions until either exits.

    `hardware_connected:=False` plus a separate servo_interface is the
    combination that produced a stable graph on the stock image; the default
    hardware path did not.
    """
    import paramiko  # noqa: F401 - fail fast in the worker if missing

    runtime = maybe_prompt_for_password(runtime)
    write_worker_meta(runtime, os.getpid(), "starting")
    bringup_client = create_ssh_client(runtime)
    servo_client = create_ssh_client(runtime)
    base = [
        "source /opt/ros/humble/setup.bash",
        "source ~/ros2_ws/install/setup.bash",
        f"export ROS_DOMAIN_ID={shlex.quote(runtime.ros_domain_id)}",
        f"export ROBOT_MODEL={shlex.quote(runtime.robot_model)}",
    ]
    try:
        bringup_cmd = bashlc(shell_join([*base, "exec ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=False"]))
        _, bringup_stdout, bringup_stderr = bringup_client.exec_command(bringup_cmd)
        for stream in (bringup_stdout, bringup_stderr):
            threading.Thread(target=drain_channel, args=(stream, WORKER_LOG_PATH, "bringup"), daemon=True).start()

        topics = wait_for_topics(runtime, ["/cmd_vel", "/joint_states"], timeout_sec=20)
        if "/cmd_vel" not in topics:
            update_worker_status("failed")
            raise CliError(f"Remote bringup did not expose /cmd_vel within 20s. Check {WORKER_LOG_PATH}.")

        servo_cmd = bashlc(shell_join([*base, "exec ros2 run mini_pupper_driver servo_interface"]))
        _, servo_stdout, servo_stderr = servo_client.exec_command(servo_cmd)
        for stream in (servo_stdout, servo_stderr):
            threading.Thread(target=drain_channel, args=(stream, WORKER_LOG_PATH, "servo"), daemon=True).start()
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


# --------------------------------------------------------------------------- command handlers


def handle_bringup(runtime: RuntimeConfig) -> None:
    runtime = maybe_prompt_for_password(runtime)
    inspect_command = bashlc(
        "printf 'HOSTNAME='; hostname; "
        "printf 'OS='; . /etc/os-release && printf '%s %s\\n' \"$NAME\" \"$VERSION_ID\"; "
        "command -v ros2 >/dev/null && echo ROS2=present || echo ROS2=missing; "
        "test -d ~/ros2_ws && echo ROS2_WS=present || echo ROS2_WS=missing; "
        "bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; source ~/ros2_ws/install/setup.bash 2>/dev/null; "
        'ros2 pkg list 2>/dev/null | grep -E "^mini_pupper_bringup$" >/dev/null '
        "&& echo BRINGUP_PKG=present || echo BRINGUP_PKG=missing'"
    )
    print(run_remote_command_checked(runtime, inspect_command, timeout=30, context="Robot inspection").strip())

    pid = start_local_worker(runtime)
    write_worker_meta(runtime, pid, "starting")
    print(f"Started session worker PID {pid}. Log: {WORKER_LOG_PATH}")

    deadline = time.time() + 30
    topics: list[str] = []
    while time.time() < deadline:
        if not process_is_alive(pid):
            raise CliError(f"Session worker exited early. Check {WORKER_LOG_PATH}.")
        try:
            topics = get_topic_list_ssh(runtime)
            if not msg.missing_topics(topics, BRINGUP_TOPICS[:3]):
                break
        except CliError:
            pass
        time.sleep(1)
    missing = msg.missing_topics(topics, BRINGUP_TOPICS)
    if missing:
        raise CliError(f"Bringup started but required topics are missing: {', '.join(missing)}")
    print("\nVerified topics:")
    for topic in BRINGUP_TOPICS:
        print(f"  {topic}")


def prepare_motion(runtime: RuntimeConfig) -> tuple[RuntimeConfig, str]:
    mode = select_mode(runtime)
    if mode == "ssh":
        runtime = maybe_prompt_for_password(runtime)
        require_worker()
    return runtime, mode


def handle_read_state(runtime: RuntimeConfig, topic: str) -> None:
    runtime, mode = prepare_motion(runtime)
    topics = get_topic_list_dds(runtime) if mode == "dds" else get_topic_list_ssh(runtime)
    print("Discovered topics:")
    for name in topics:
        print(f"  {name}")

    selected = msg.choose_topics(topics, topic)
    if not selected:
        print("\nNo priority state topics were found. Use --topic <name> to sample a specific topic.")
        return
    for chosen in selected:
        print(f"\n=== {chosen} ===")
        print(echo_topic(runtime, mode, chosen) or "<no output>")


def handle_move(runtime: RuntimeConfig, linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    try:
        msg.validate_motion(linear_x, linear_y, angular_z, duration_sec)
    except msg.MotionLimitError as exc:
        raise CliError(str(exc)) from exc
    runtime, mode = prepare_motion(runtime)
    publish_twist(runtime, mode, linear_x, linear_y, angular_z, duration_sec)
    print(f"Published cmd_vel for {duration_sec:.2f}s and sent stop.")


def handle_posture(
    runtime: RuntimeConfig, x: float, y: float, z_offset: float, roll: float, pitch: float, yaw: float, hold_sec: float
) -> None:
    runtime, mode = prepare_motion(runtime)
    pose = msg.build_pose_message(x, y, z_offset, roll, pitch, yaw)
    line = timed_publish_command("/body_pose", "geometry_msgs/msg/Pose", pose, max(hold_sec, 0.2))
    publish(runtime, mode, [line], hold_sec, "body_pose publish")
    print(f"Published body_pose z_offset={z_offset:.3f} for {hold_sec:.2f}s.")


def handle_joint_pose(runtime: RuntimeConfig, preset: str, hold_sec: float) -> None:
    runtime, mode = prepare_motion(runtime)
    trajectory = msg.build_joint_trajectory_message(msg.JOINT_POSE_PRESETS_DEG[preset])
    line = timed_publish_command(JOINT_TRAJECTORY_TOPIC, "trajectory_msgs/msg/JointTrajectory", trajectory, max(hold_sec, 0.2))
    publish(runtime, mode, [line], hold_sec, "joint trajectory publish")
    print(f"Published joint pose preset `{preset}` for {hold_sec:.2f}s.")


def handle_stop(runtime: RuntimeConfig) -> None:
    runtime, mode = prepare_motion(runtime)
    publish(runtime, mode, [stop_command()], 0, "stop publish")
    print("Sent zero-velocity stop.")


def remote_background_pid(runtime: RuntimeConfig, script: str, context: str) -> int:
    stdout = run_remote_command_checked(runtime, bashlc(f"{ros_env(runtime)}\n{script}"), timeout=60, context=context).strip()
    try:
        return int(stdout.splitlines()[-1].strip())
    except (IndexError, ValueError) as exc:
        raise CliError(f"{context} did not return a valid PID. Output was: {stdout}") from exc


def stop_slam_processes(runtime: RuntimeConfig) -> None:
    meta = read_json(SLAM_META_PATH)
    pids = [str(meta.get(key)) for key in ("occupancy_pid", "cartographer_pid", "lidar_pid") if isinstance(meta.get(key), int)]
    if pids:
        run_remote_command(runtime, bashlc(f"kill {' '.join(pids)} 2>/dev/null || true"), timeout=20)
    if SLAM_META_PATH.exists():
        SLAM_META_PATH.unlink()


def remote_topic_list(runtime: RuntimeConfig) -> list[str]:
    return split_lines(run_remote_ros(runtime, "sleep 1\nros2 topic list", "Remote ros2 topic list", timeout=45))


def handle_start_slam(runtime: RuntimeConfig) -> None:
    """Run Cartographer headless.

    The stock `mini_pupper_slam` launch file also starts RViz, which is not
    installed on the robot image, so the launch fails. Starting the LiDAR
    driver, cartographer_node, and the occupancy grid node by hand avoids it.
    """
    runtime = maybe_prompt_for_password(runtime)
    require_worker()
    stop_slam_processes(runtime)

    lidar_pid = remote_background_pid(
        runtime,
        f"nohup ros2 launch ldlidar_stl_ros2 ld06.launch.py >{ROBOT_LOGS['lidar']} 2>&1 < /dev/null & echo $!",
        "Start LiDAR",
    )
    time.sleep(3)
    cartographer_pid = remote_background_pid(
        runtime,
        textwrap.dedent(
            f"""
            config_dir="$(ros2 pkg prefix mini_pupper_slam)/share/mini_pupper_slam/config"
            nohup ros2 run cartographer_ros cartographer_node \\
              -configuration_directory "$config_dir" \\
              -configuration_basename slam.lua \\
              --ros-args -r /imu/data:=/imu \\
              >{ROBOT_LOGS["cartographer"]} 2>&1 < /dev/null & echo $!
            """
        ).strip(),
        "Start Cartographer",
    )
    occupancy_pid = remote_background_pid(
        runtime,
        f"nohup ros2 run cartographer_ros cartographer_occupancy_grid_node >{ROBOT_LOGS['occupancy']} 2>&1 < /dev/null & echo $!",
        "Start occupancy grid node",
    )
    write_json(
        SLAM_META_PATH,
        {
            "robot_ip": runtime.robot_ip,
            "lidar_pid": lidar_pid,
            "cartographer_pid": cartographer_pid,
            "occupancy_pid": occupancy_pid,
            "updated_at": time.time(),
            "logs": ROBOT_LOGS,
        },
    )

    topics = wait_for_topics(runtime, SLAM_TOPICS, timeout_sec=20, lister=remote_topic_list)
    missing = msg.missing_topics(topics, SLAM_TOPICS)
    if missing:
        stop_slam_processes(runtime)
        raise CliError(f"SLAM did not expose required topics: {', '.join(missing)}. Check {', '.join(ROBOT_LOGS.values())} on the robot.")
    print("Started headless SLAM on the robot.\nVerified topics:")
    for topic in SLAM_TOPICS:
        print(f"  {topic}")


def handle_stop_slam(runtime: RuntimeConfig) -> None:
    stop_slam_processes(maybe_prompt_for_password(runtime))
    print("Stopped recorded robot-side SLAM processes.")


def handle_shutdown(runtime: RuntimeConfig) -> None:
    runtime = maybe_prompt_for_password(runtime)
    stop_slam_processes(runtime)
    if worker_running():
        run_remote_ros(runtime, stop_command(), "Stop publish", timeout=30)
    terminate_worker()
    update_worker_status("stopped")
    print("Robot stopped and session worker ended.")


def handle_show_config(runtime: RuntimeConfig, args: argparse.Namespace) -> None:
    print(
        json.dumps(
            {
                "robot_ip": runtime.robot_ip,
                "ssh_user": runtime.ssh_user,
                "ssh_password": "<set>" if runtime.ssh_password else "<prompt>",
                "ros_domain_id": runtime.ros_domain_id,
                "preferred_mode": runtime.preferred_mode,
                "robot_model": runtime.robot_model,
                "config_path": str(config_path(args.config).resolve()),
                "state_dir": str(STATE_DIR),
                "worker_running": worker_running(),
            },
            indent=2,
        )
    )


def main(argv: Sequence[str] | None = None) -> int:
    global DRY_RUN
    args = build_parser().parse_args(argv)
    DRY_RUN = args.dry_run
    try:
        if DRY_RUN and args.command not in MOTION_COMMANDS:
            raise CliError(f"--dry-run supports {', '.join(sorted(MOTION_COMMANDS))}")
        if args.command == "presets":
            for name in sorted(msg.JOINT_POSE_PRESETS_DEG):
                print(msg.summarize_preset(name))
            return 0
        if args.command == "slam_status":
            print(json.dumps(read_json(SLAM_META_PATH) or {"status": "not_running"}, indent=2))
            return 0

        runtime = resolve_runtime_config(args, allow_placeholder_ip=args.command == "show_config")
        handlers = {
            "bringup_robot": lambda: handle_bringup(runtime),
            "read_state": lambda: handle_read_state(runtime, args.topic),
            "move_robot": lambda: handle_move(runtime, args.linear_x, args.linear_y, args.angular_z, args.duration_sec),
            "set_posture": lambda: handle_posture(runtime, args.x, args.y, args.z_offset, args.roll, args.pitch, args.yaw, args.hold_sec),
            "set_joint_pose": lambda: handle_joint_pose(runtime, args.preset, args.hold_sec),
            "stop_robot": lambda: handle_stop(runtime),
            "start_slam": lambda: handle_start_slam(runtime),
            "stop_slam": lambda: handle_stop_slam(runtime),
            "shutdown": lambda: handle_shutdown(runtime),
            "show_config": lambda: handle_show_config(runtime, args),
        }
        if args.command == "session_worker":
            return run_session_worker(runtime)
        handlers[args.command]()
        return 0
    except CliError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
