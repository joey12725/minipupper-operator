"""Render a ROS OccupancyGrid snapshot to a PIL image (no Tk required)."""

from __future__ import annotations

from dataclasses import dataclass

from PIL import Image, ImageDraw

from . import messages as msg

CANVAS_SIZE = 640


@dataclass
class MapSnapshot:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list[int]


def fit_size(width: int, height: int, canvas: int = CANVAS_SIZE) -> tuple[int, int]:
    """Largest size with the grid's aspect ratio that fits in a canvas x canvas box."""
    scale = canvas / max(width, height)
    return max(1, round(width * scale)), max(1, round(height * scale))


def snapshot_to_image(snapshot: MapSnapshot, canvas: int = CANVAS_SIZE) -> Image.Image:
    grid = Image.new("RGB", (snapshot.width, snapshot.height))
    rows = msg.occupancy_rows(snapshot.data, snapshot.width, snapshot.height)
    grid.putdata([pixel for row in rows for pixel in row])
    grid = grid.resize(fit_size(snapshot.width, snapshot.height, canvas), Image.Resampling.NEAREST)

    image = Image.new("RGB", (canvas, canvas), msg.occupancy_color(-1))
    image.paste(grid, ((canvas - grid.width) // 2, (canvas - grid.height) // 2))
    draw = ImageDraw.Draw(image)
    draw.rectangle((8, 8, 160, 48), fill=(255, 255, 255))
    draw.text((14, 14), f"{snapshot.width}x{snapshot.height} cells", fill=(0, 0, 0))
    draw.text((14, 30), f"{snapshot.resolution:.3f} m/cell", fill=(0, 0, 0))
    return image
