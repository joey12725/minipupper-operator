# Copyright (c) 2026 Joey Hersh
# SPDX-License-Identifier: MIT
"""
Joint-space pose presets for a Mini Pupper 1 (degrees, LF/RF/LB/RB legs).

Kept in sync with minipupper_operator.messages.JOINT_POSE_PRESETS_DEG.
"""

JOINT_NAMES = (
    'base_lf1', 'lf1_lf2', 'lf2_lf3',
    'base_rf1', 'rf1_rf2', 'rf2_rf3',
    'base_lb1', 'lb1_lb2', 'lb2_lb3',
    'base_rb1', 'rb1_rb2', 'rb2_rb3',
)

PRESETS_DEG = {
    'neutral': (0.0, 15.0, -20.0) * 4,
    'stand': (0.0, 8.0, -12.0) * 4,
    'sit': (0.0, 25.0, -35.0) * 2 + (0.0, 40.0, -55.0) * 2,
    'lay': (0.0, 40.0, -55.0) * 4,
}


def next_index(index, length):
    """Return the next position in a looping sequence of `length` items."""
    if length <= 0:
        raise ValueError('sequence must not be empty')
    return (index + 1) % length
