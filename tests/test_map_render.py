import pytest

pytest.importorskip("PIL")

from minipupper_operator.map_render import MapSnapshot, fit_size, snapshot_to_image  # noqa: E402


def test_fit_size_keeps_aspect_ratio():
    assert fit_size(100, 50, 640) == (640, 320)
    assert fit_size(20, 40, 640) == (320, 640)
    assert fit_size(1, 1, 640) == (640, 640)


def test_snapshot_to_image_draws_walls_free_space_and_unknown():
    # 4x2 grid: bottom row free, top row occupied (ROS rows start at the bottom).
    snapshot = MapSnapshot(width=4, height=2, resolution=0.05, origin_x=0, origin_y=0, data=[0] * 4 + [100] * 4)
    image = snapshot_to_image(snapshot, canvas=400)
    assert image.size == (400, 400)
    # Grid is letterboxed to 400x200 and centred vertically.
    assert image.getpixel((200, 120)) == (25, 25, 25)  # top half of grid: wall
    assert image.getpixel((200, 280)) == (245, 245, 245)  # bottom half: free
    assert image.getpixel((200, 395)) == (185, 185, 185)  # letterbox: unknown grey
