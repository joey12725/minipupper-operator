# Copyright (c) 2026 Joey Hersh
# SPDX-License-Identifier: MIT
import pytest
from pose_toggler.presets import JOINT_NAMES, PRESETS_DEG, next_index


def test_every_preset_covers_every_joint():
    for values in PRESETS_DEG.values():
        assert len(values) == len(JOINT_NAMES)


def test_next_index_wraps():
    assert [next_index(i, 2) for i in (-1, 0, 1)] == [0, 1, 0]


def test_next_index_rejects_empty():
    with pytest.raises(ValueError):
        next_index(0, 0)
