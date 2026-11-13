import json

import pytest

from minipupper_operator import cli


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    for name in ("ROBOT_IP", "ROBOT_SSH_USER", "ROBOT_PASSWORD", "ROS_DOMAIN_ID", "MINIPUPPER_MODE", "ROBOT_MODEL", "MINIPUPPER_CONFIG"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(cli, "STATE_DIR", tmp_path / ".minipupper")
    monkeypatch.setattr(cli, "WORKER_META_PATH", tmp_path / ".minipupper" / "session.json")
    monkeypatch.setattr(cli, "SLAM_META_PATH", tmp_path / ".minipupper" / "slam.json")
    monkeypatch.setattr(cli, "DRY_RUN", False)
    return tmp_path


def parse(*argv):
    return cli.build_parser().parse_args(list(argv))


def test_flag_beats_env_beats_file(isolated, monkeypatch):
    (isolated / "robot_config.json").write_text(json.dumps({"robot_ip": "192.0.2.1", "ros_domain_id": "7", "ssh_user": "pi"}))
    monkeypatch.setenv("ROBOT_IP", "192.0.2.2")

    runtime = cli.resolve_runtime_config(parse("show_config"))
    assert runtime.robot_ip == "192.0.2.2"
    assert runtime.ros_domain_id == "7"
    assert runtime.ssh_user == "pi"

    runtime = cli.resolve_runtime_config(parse("--robot-ip", "192.0.2.3", "show_config"))
    assert runtime.robot_ip == "192.0.2.3"


def test_defaults_without_config(isolated):
    runtime = cli.resolve_runtime_config(parse("--robot-ip", "pupper.local", "show_config"))
    assert (runtime.ssh_user, runtime.ros_domain_id, runtime.preferred_mode, runtime.robot_model) == (
        "ubuntu",
        "42",
        "auto",
        "mini_pupper",
    )
    assert runtime.ssh_password is None


def test_missing_ip_is_an_error(isolated):
    with pytest.raises(cli.CliError):
        cli.resolve_runtime_config(parse("stop_robot"))


def test_placeholder_ip_is_an_error(isolated):
    (isolated / "robot_config.json").write_text(json.dumps({"robot_ip": "<robot-ip>"}))
    with pytest.raises(cli.CliError):
        cli.resolve_runtime_config(parse("stop_robot"))


def test_invalid_json_config(isolated):
    (isolated / "robot_config.json").write_text("{nope")
    with pytest.raises(cli.CliError):
        cli.resolve_runtime_config(parse("show_config"))


def test_explicit_config_path(isolated):
    path = isolated / "lab.json"
    path.write_text(json.dumps({"robot_ip": "192.0.2.9"}))
    assert cli.resolve_runtime_config(parse("--config", str(path), "stop_robot")).robot_ip == "192.0.2.9"


def test_timed_publish_tolerates_timeout_exit():
    line = cli.timed_publish_command("/cmd_vel", "geometry_msgs/msg/Twist", "{}", 1.5)
    assert line.startswith("timeout 1.50s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{}'")
    assert line.endswith("|| [ $? -eq 124 ]")


def test_dry_run_move_prints_drive_then_stop(capsys):
    code = cli.main(["--robot-ip", "192.0.2.10", "--dry-run", "move_robot", "--linear-x", "0.15", "--duration-sec", "1.5"])
    out = capsys.readouterr().out
    assert code == 0
    assert out.startswith("[dry-run] ssh ubuntu@192.0.2.10 bash -lc")
    drive = out.index("timeout 1.50s ros2 topic pub -r 10 /cmd_vel")
    stop = out.index("ros2 topic pub --once /cmd_vel")
    assert drive < stop
    assert "ros2 daemon stop" in out


def test_dry_run_joint_pose_targets_effort_controller(capsys):
    code = cli.main(["--robot-ip", "192.0.2.10", "--dry-run", "set_joint_pose", "--preset", "sit"])
    assert code == 0
    assert "/joint_group_effort_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory" in capsys.readouterr().out


def test_dry_run_dds_mode_uses_local_shell(capsys, monkeypatch):
    monkeypatch.setattr(cli.os, "name", "posix")
    code = cli.main(["--robot-ip", "192.0.2.10", "--mode", "dds", "--dry-run", "stop_robot"])
    out = capsys.readouterr().out
    assert code == 0
    assert out.startswith("[dry-run] bash -lc")
    assert "ROS_DOMAIN_ID=42" in out


def test_dry_run_refuses_non_motion_commands(capsys):
    assert cli.main(["--robot-ip", "192.0.2.10", "--dry-run", "start_slam"]) == 2
    assert "--dry-run supports" in capsys.readouterr().err


def test_motion_limits_are_enforced_before_connecting(capsys):
    assert cli.main(["--robot-ip", "192.0.2.10", "--dry-run", "move_robot", "--linear-x", "3", "--duration-sec", "1"]) == 2
    err = capsys.readouterr().err
    assert "linear-x" in err
    assert "[dry-run]" not in err


def test_motion_without_worker_asks_for_bringup(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT_PASSWORD", "x")
    assert cli.main(["--robot-ip", "192.0.2.10", "--mode", "ssh", "stop_robot"]) == 2
    assert "bringup_robot" in capsys.readouterr().err


def test_local_bash_argv_uses_wsl_on_windows(monkeypatch):
    monkeypatch.setattr(cli.os, "name", "nt")
    monkeypatch.setenv("MINIPUPPER_WSL_DISTRO", "Ubuntu-22.04")
    assert cli.local_bash_argv("true") == ["wsl.exe", "-d", "Ubuntu-22.04", "bash", "-lc", "true"]
    monkeypatch.setattr(cli.os, "name", "posix")
    assert cli.local_bash_argv("true") == ["bash", "-lc", "true"]


def test_show_config_hides_password(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT_PASSWORD", "hunter2")
    assert cli.main(["show_config"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["ssh_password"] == "<set>"
    assert out["robot_ip"] == "<robot-ip>"
    assert out["worker_running"] is False


def test_slam_status_without_session(capsys):
    assert cli.main(["slam_status"]) == 0
    assert json.loads(capsys.readouterr().out) == {"status": "not_running"}


def test_presets_command(capsys):
    assert cli.main(["presets"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert [line.split(":")[0] for line in out] == ["lay", "neutral", "sit", "stand"]


def test_worker_running_reads_pid(isolated):
    import os

    assert cli.worker_running() is False
    cli.write_json(cli.WORKER_META_PATH, {"pid": os.getpid()})
    assert cli.worker_running() is True
    cli.write_json(cli.WORKER_META_PATH, {"pid": "nope"})
    assert cli.worker_running() is False
