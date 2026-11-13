"""Tk desktop app: W/A/S/D teleop plus a live view of the robot's SLAM map.

Drive the robot from the keyboard while Cartographer builds the map; the
occupancy grid is pulled from `/map` about once a second and drawn beside the
controls. Requires `bringup_robot` (or the Bringup button) first.
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import tkinter as tk

from PIL import Image, ImageTk

from . import cli as mp
from . import messages as msg
from .map_render import CANVAS_SIZE, MapSnapshot, snapshot_to_image

MAP_POLL_SEC = 1.0
TELEOP_PUBLISH_SEC = 0.18
TELEOP_LOOP_SEC = 0.12
LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.90


def remote_ros_python(runtime: mp.RuntimeConfig, python_source: str, context: str, timeout: int = 30) -> str:
    script = f"{mp.ros_env(runtime)}\npython3 - <<'PY'\n{python_source}\nPY"
    return mp.run_remote_command_checked(runtime, mp.bashlc(script), timeout=timeout, context=context).strip()


def fetch_map_snapshot(runtime: mp.RuntimeConfig) -> MapSnapshot | None:
    source = """
import json
import rclpy
from nav_msgs.msg import OccupancyGrid

payload = {}

def callback(msg):
    payload["width"] = msg.info.width
    payload["height"] = msg.info.height
    payload["resolution"] = msg.info.resolution
    payload["origin_x"] = msg.info.origin.position.x
    payload["origin_y"] = msg.info.origin.position.y
    payload["data"] = list(msg.data)
    raise SystemExit(0)

rclpy.init()
node = rclpy.create_node("minipupper_map_fetch")
node.create_subscription(OccupancyGrid, "/map", callback, 10)
deadline = node.get_clock().now().nanoseconds + 5_000_000_000
try:
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()

if not payload:
    raise SystemExit(2)
print(json.dumps(payload))
"""
    try:
        raw = remote_ros_python(runtime, source, "Fetch /map", timeout=15)
    except Exception:
        return None
    payload = json.loads(raw)
    return MapSnapshot(
        width=payload["width"],
        height=payload["height"],
        resolution=payload["resolution"],
        origin_x=payload["origin_x"],
        origin_y=payload["origin_y"],
        data=payload["data"],
    )


class OperatorApp:
    def __init__(self, runtime: mp.RuntimeConfig) -> None:
        self.runtime = runtime
        self.root = tk.Tk()
        self.root.title("Mini Pupper SLAM Operator")
        self.root.geometry("940x760")

        self.status_var = tk.StringVar(value="Idle")
        self.keys_down: set[str] = set()
        self.stop_event = threading.Event()
        self.current_map_photo: ImageTk.PhotoImage | None = None
        self.last_twist = (None, None)
        self.ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()

        self._build_ui()
        self._bind_keys()

        self.map_thread = threading.Thread(target=self._map_loop, daemon=True)
        self.teleop_thread = threading.Thread(target=self._teleop_loop, daemon=True)
        self.map_thread.start()
        self.teleop_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._drain_ui_queue)

    def _build_ui(self) -> None:
        frame = tk.Frame(self.root, padx=12, pady=12)
        frame.pack(fill=tk.BOTH, expand=True)

        top = tk.Frame(frame)
        top.pack(fill=tk.X)

        for label, callback in [
            ("Bringup", self._bringup),
            ("Start SLAM", self._start_slam),
            ("Stop SLAM", self._stop_slam),
            ("Stop Robot", self._stop_robot),
        ]:
            tk.Button(top, text=label, command=callback, width=12).pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(top, textvariable=self.status_var, anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)

        body = tk.Frame(frame)
        body.pack(fill=tk.BOTH, expand=True, pady=(12, 0))

        self.map_label = tk.Label(body, bd=1, relief=tk.SUNKEN, width=CANVAS_SIZE, height=CANVAS_SIZE)
        self.map_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=False)

        info = tk.Frame(body, padx=16)
        info.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tk.Label(info, text="Keyboard Teleop", font=("TkDefaultFont", 13, "bold")).pack(anchor="w")
        tk.Label(info, text="W/S: forward/back").pack(anchor="w", pady=(12, 0))
        tk.Label(info, text="A/D: rotate left/right").pack(anchor="w")
        tk.Label(info, text="Space: immediate stop").pack(anchor="w")
        tk.Label(info, text="Keep this window focused while driving.").pack(anchor="w", pady=(16, 0))

        self.topic_text = tk.Text(info, height=24, width=34)
        self.topic_text.pack(fill=tk.BOTH, expand=True, pady=(20, 0))
        self.topic_text.insert(
            "1.0",
            "Run order:\n1. Bringup\n2. Start SLAM\n3. Drive with W/A/S/D\n\nThe map view refreshes from /map about once per second.\n",
        )
        self.topic_text.configure(state=tk.DISABLED)

    def _bind_keys(self) -> None:
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)

    def _set_status(self, message: str) -> None:
        self.ui_queue.put(("status", message))

    def _set_map_image(self, image: Image.Image) -> None:
        self.ui_queue.put(("map", image))

    def _drain_ui_queue(self) -> None:
        while True:
            try:
                kind, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "status":
                self.status_var.set(str(payload))
            elif kind == "map":
                photo = ImageTk.PhotoImage(payload)
                self.current_map_photo = photo
                self.map_label.configure(image=photo)
        if not self.stop_event.is_set():
            self.root.after(50, self._drain_ui_queue)

    def _run_action(self, label: str, fn) -> None:
        def runner() -> None:
            self._set_status(f"{label}...")
            try:
                fn()
            except Exception as exc:
                self._set_status(f"{label} failed: {exc}")
            else:
                self._set_status(f"{label} complete")

        threading.Thread(target=runner, daemon=True).start()

    def _bringup(self) -> None:
        self._run_action("Bringup", lambda: mp.handle_bringup(self.runtime))

    def _start_slam(self) -> None:
        self._run_action("Start SLAM", lambda: mp.handle_start_slam(self.runtime))

    def _stop_slam(self) -> None:
        self._run_action("Stop SLAM", lambda: mp.handle_stop_slam(self.runtime))

    def _stop_robot(self) -> None:
        self._run_action("Stop Robot", lambda: mp.handle_stop(self.runtime))

    def _on_key_press(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        if key == "space":
            self.keys_down.clear()
            self._stop_robot()
            return
        if key in {"w", "a", "s", "d"}:
            self.keys_down.add(key)

    def _on_key_release(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        self.keys_down.discard(key)

    def _desired_twist(self) -> tuple[float, float]:
        return msg.teleop_twist(self.keys_down, LINEAR_SPEED, ANGULAR_SPEED)

    def _teleop_loop(self) -> None:
        while not self.stop_event.is_set():
            linear, angular = self._desired_twist()
            twist = (round(linear, 3), round(angular, 3))
            if twist != self.last_twist or twist != (0.0, 0.0):
                try:
                    mp.publish_twist_ssh(self.runtime, linear, 0.0, angular, TELEOP_PUBLISH_SEC)
                except Exception as exc:
                    self._set_status(f"Teleop failed: {exc}")
                    time.sleep(0.6)
                else:
                    if twist != (0.0, 0.0):
                        self._set_status(f"Driving: linear={linear:.2f} angular={angular:.2f}")
                    self.last_twist = twist
            time.sleep(TELEOP_LOOP_SEC)

    def _map_loop(self) -> None:
        while not self.stop_event.is_set():
            snapshot = fetch_map_snapshot(self.runtime)
            if snapshot is not None:
                image = snapshot_to_image(snapshot)
                self._set_map_image(image)
            time.sleep(MAP_POLL_SEC)

    def _on_close(self) -> None:
        self.stop_event.set()
        self.root.after(50, self.root.destroy)

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(prog="minipupper-slam", description="Mini Pupper keyboard teleop + live SLAM map viewer.")
    mp.add_connection_args(parser)
    args = parser.parse_args()

    try:
        runtime = mp.maybe_prompt_for_password(mp.resolve_runtime_config(args))
    except mp.CliError as exc:
        print(f"ERROR: {exc}")
        return 2

    app = OperatorApp(runtime)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
