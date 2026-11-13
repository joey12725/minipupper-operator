"""Render docs/assets/demo-map.png: a synthetic LiDAR map drawn by the SLAM viewer's renderer.

A made-up floor plan is "scanned" by ray-casting a 360-degree LD06-style LiDAR
from poses along a path, producing an OccupancyGrid with free, occupied, and
unknown cells, which then goes through the same snapshot_to_image the viewer
uses. No robot data is involved.

    python scripts/render_demo_map.py
"""

from __future__ import annotations

import math
from pathlib import Path

from minipupper_operator.map_render import MapSnapshot, snapshot_to_image

W, H = 160, 120  # cells, 5 cm each -> 8 m x 6 m
RES = 0.05
MAX_RANGE = 3.0 / RES  # cells; roughly an LD06 indoors


def floor_plan() -> list[list[bool]]:
    wall = [[False] * W for _ in range(H)]

    def box(x0, y0, x1, y1):
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                wall[y][x] = True

    box(5, 5, 154, 6)  # outer walls
    box(5, 113, 154, 114)
    box(5, 5, 6, 114)
    box(153, 5, 154, 114)
    box(90, 5, 91, 50)  # interior wall with doorway
    box(90, 68, 91, 114)
    box(91, 70, 125, 71)  # second room
    box(20, 80, 45, 95)  # couch
    box(60, 20, 72, 32)  # table
    box(125, 25, 140, 40)  # cabinet
    for cx, cy in ((35, 30), (115, 95), (140, 95)):  # chair / plant legs
        box(cx, cy, cx + 2, cy + 2)
    return wall


def scan(wall: list[list[bool]]) -> list[int]:
    grid = [-1] * (W * H)
    path = [(25 + i * 4, 55 + int(12 * math.sin(i / 3))) for i in range(30)] + [(130, 55 + i * 3) for i in range(5)]
    for px, py in path:
        for k in range(360):
            angle = math.radians(k)
            dx, dy = math.cos(angle), math.sin(angle)
            for step in range(int(MAX_RANGE)):
                x, y = int(px + dx * step), int(py + dy * step)
                if not (0 <= x < W and 0 <= y < H):
                    break
                if wall[y][x]:
                    grid[y * W + x] = 100
                    break
                if grid[y * W + x] != 100:
                    grid[y * W + x] = 0
    return grid


def main() -> None:
    snapshot = MapSnapshot(width=W, height=H, resolution=RES, origin_x=0.0, origin_y=0.0, data=scan(floor_plan()))
    out = Path(__file__).resolve().parent.parent / "docs" / "assets" / "demo-map.png"
    snapshot_to_image(snapshot).save(out, optimize=True)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
