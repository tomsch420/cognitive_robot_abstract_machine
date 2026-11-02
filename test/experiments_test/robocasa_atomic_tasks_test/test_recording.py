"""
Turning recorded poses into the positions of the free joints a video is replayed with.
"""

from __future__ import annotations

import numpy as np

from experiments.robocasa_atomic_tasks.recording import RobotViewVideo

# %% free joint positions


def test_a_free_joint_position_is_the_translation_then_the_quaternion_scalar_first():
    quarter_turn_about_z = np.array(
        [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    half_angle = np.pi / 4

    [position] = RobotViewVideo.free_joint_positions(quarter_turn_about_z[None])

    np.testing.assert_allclose(
        position, [1.0, 2.0, 3.0, np.cos(half_angle), 0.0, 0.0, np.sin(half_angle)]
    )


def test_free_joint_positions_keep_the_leading_axes_of_the_poses():
    frames, bodies = 3, 5
    poses = np.broadcast_to(np.eye(4), (frames, bodies, 4, 4))

    positions = RobotViewVideo.free_joint_positions(poses)

    np.testing.assert_allclose(
        positions, np.broadcast_to([0, 0, 0, 1, 0, 0, 0], (frames, bodies, 7))
    )
