"""
The grasps the PR2 tries on the kitchen's parts and objects, coming in along the way it
faces the kitchen.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from experiments.robocasa_atomic_tasks.geometry import ApproachFrame, BodyBounds
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle

# %% the grasps tried


@dataclass
class KitchenGrasps:
    """
    Grasps on the parts and objects of a kitchen the robot faces along
    :attr:`kitchen_direction`.
    """

    bounds: BodyBounds
    """
    The bounding boxes of the world's bodies.
    """

    kitchen_direction: np.ndarray
    """
    The horizontal direction the robot faces the kitchen in.
    """

    def on_part(
        self,
        handle: Handle,
        depth: float = 0.02,
        upward_slope: float = 0.8,
        lower_end_height: float = 0.04,
    ) -> list[GraspCandidate]:
        """
        The grasps a part of a fixture is taken hold of by, such as a handle, a knob, a
        lever or the front edge of a rack.

        The gripper approaches the part horizontally from each of the four sides of the
        kitchen, or from above, and its fingers close across either of the part's two
        extents square to the approach, the thinner first. The grasp lies on the face
        the gripper comes in through, no deeper into the part than ``depth``. The
        gripper also comes in from below each side, rising towards the part's lower end
        with its fingers closing level.

        :param handle: The annotation of the part.
        :param depth: How far behind the face the gripper comes in through the grasp
            lies at most, in metres.
        :param upward_slope: How steeply the gripper rises when coming in from below, as
            the rise per unit of horizontal travel.
        :param lower_end_height: How far above its lowest point a part is taken when
            coming in from below, in metres, at most half its height.
        :return: The grasps, the horizontal ones first, then those from above, then
            those from below.
        """
        box = self.bounds.of_body(handle.root)
        center = (box.lower + box.upper) / 2
        extents = box.upper - box.lower
        horizontal_approaches = [
            np.array([1.0, 0.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, -1.0, 0.0]),
        ]
        grasps = []
        for approach in [*horizontal_approaches, np.array([0.0, 0.0, -1.0])]:
            along = int(np.argmax(np.abs(approach)))
            across = [axis for axis in range(3) if axis != along]
            half_depth = extents[along] / 2
            position = center - approach * (half_depth - min(half_depth, depth))
            for closing_axis in sorted(across, key=lambda axis: extents[axis]):
                closing = np.zeros(3)
                closing[closing_axis] = 1.0
                grasps.append(self.grasp_at(handle, position, approach, closing))
        for horizontal in horizontal_approaches:
            approach = horizontal + np.array([0.0, 0.0, upward_slope])
            position = center.copy()
            position[2] = box.lower[2] + min(extents[2] / 2, lower_end_height)
            closing = np.cross([0.0, 0.0, 1.0], horizontal)
            grasps.append(self.grasp_at(handle, position, approach, closing))
        return grasps

    def from_above(
        self,
        graspable: HasGraspCandidates,
        depth: float = 0.03,
        pitches: tuple[float, ...] = (math.pi / 4, math.pi / 2),
        degenerate_closing: float = 1e-6,
    ) -> list[GraspCandidate]:
        """
        Grasps that take an object from above: coming down onto its top along
        :attr:`kitchen_direction`, pitched down by each of a list of angles, with the
        fingers closing level across the approach, once along each of the object's
        horizontal extents.

        :param graspable: The object.
        :param depth: How far below the top of the object the grasp lies at most, in
            metres.
        :param pitches: How steeply the gripper comes down, in radians below the
            horizontal.
        :param degenerate_closing: How short a closing direction, once made square to
            the approach, counts as none.
        :return: The grasps, the least steep first.
        """
        box = self.bounds.of_body(graspable.root)
        height = box.upper[2] - box.lower[2]
        position = np.append(
            (box.lower[:2] + box.upper[:2]) / 2, box.upper[2] - min(depth, height / 2)
        )
        direction = self.horizontal_kitchen_direction
        grasps = []
        for pitch in pitches:
            approach = math.cos(pitch) * direction + np.array(
                [0.0, 0.0, -math.sin(pitch)]
            )
            for closing in (np.cross([0.0, 0.0, 1.0], direction), direction):
                closing = closing - (closing @ approach) * approach
                if np.linalg.norm(closing) < degenerate_closing:
                    continue
                grasps.append(self.grasp_at(graspable, position, approach, closing))
        return grasps

    def approaching_along_the_kitchen(
        self,
        grasps: list[GraspCandidate],
        minimum_alignment: float = 0.5,
        lowest_rise: float = -0.5,
        highest_rise: float = 0.9,
        level_tolerance: float = 0.3,
        degenerate_horizontal_part: float = 1e-6,
    ) -> list[GraspCandidate]:
        """
        :param grasps: Grasps on a part.
        :param minimum_alignment: How well at least, as the cosine of the angle between
            them, a horizontal grasp's approach has to follow :attr:`kitchen_direction`.
        :param lowest_rise: The vertical component of the approach below which a grasp
            comes in from above.
        :param highest_rise: The vertical component of the approach above which a grasp
            is left out, as it comes in from straight below.
        :param level_tolerance: The vertical component of the approach up to which a
            grasp counts as level.
        :param degenerate_horizontal_part: How short the horizontal part of an
            approach counts as none, aligned with no direction.
        :return: The grasps that come in along :attr:`kitchen_direction`, the level ones
            before the rising ones and each best aligned first, followed by those coming
            in from above.
        """
        direction = self.horizontal_kitchen_direction

        def approach_of(grasp: GraspCandidate) -> np.ndarray:
            return grasp.world_T_grasp.to_np()[:3, 0]

        def alignment_of(grasp: GraspCandidate) -> float:
            approach = approach_of(grasp)
            horizontal_part = np.linalg.norm(approach[:2])
            if horizontal_part < degenerate_horizontal_part:
                return 0.0
            return float(approach[:2] @ direction[:2]) / horizontal_part

        level_or_rising = [
            grasp
            for grasp in grasps
            if lowest_rise < approach_of(grasp)[2] < highest_rise
            and alignment_of(grasp) >= minimum_alignment
        ]
        level_or_rising.sort(
            key=lambda grasp: (
                abs(approach_of(grasp)[2]) > level_tolerance,
                -alignment_of(grasp),
            )
        )
        from_above = [grasp for grasp in grasps if approach_of(grasp)[2] <= lowest_rise]
        return level_or_rising + from_above

    @staticmethod
    def keep_the_grasps_that_fit(
        grasps: list[GraspCandidate], opening: float
    ) -> list[GraspCandidate]:
        """
        :param grasps: Grasps on parts or objects.
        :param opening: How far apart the gripper's fingertips stand when it is open,
            in metres.
        :return: The grasps, in their order, whose object is no wider along the axis the
            fingers close along than ``opening``, or that carries no collision to measure.
        """
        return [
            grasp
            for grasp in grasps
            if not grasp.graspable.root.has_collision()
            or grasp.width_along_closing_axis <= opening
        ]

    @property
    def horizontal_kitchen_direction(self) -> np.ndarray:
        """
        :attr:`kitchen_direction`, levelled and of unit length.
        """
        direction = np.asarray(self.kitchen_direction, dtype=float)
        return np.append(direction[:2] / np.linalg.norm(direction[:2]), 0.0)

    @staticmethod
    def grasp_at(
        graspable: HasGraspCandidates,
        position: np.ndarray,
        approach: np.ndarray,
        closing: np.ndarray,
    ) -> GraspCandidate:
        """
        :param graspable: What is grasped.
        :param position: Where the grasp lies, in the world's root frame.
        :param approach: The direction the gripper comes in along.
        :param closing: The direction the fingers close along.
        :return: The grasp, relative to the grasped body.
        """
        world = graspable.root._world
        world_T_grasp = ApproachFrame(
            position=position, approach=approach, closing=closing
        ).in_reference_frame(world.root)
        return GraspCandidate(
            graspable, world.transform(world_T_grasp.pose, graspable.root)
        )
