"""
This module collects the placements RoboCasa actually resolves, so that the cost of the
orientation coupling can be measured on real regions and objects rather than on invented
ones.

A task states where an object may go only abstractly. The concrete region, the object's
footprint and the yaws it may take are decided when a scene is built, and they differ
across the kitchen layouts a task can be instantiated in. Resetting a task repeatedly
therefore samples the placements a robot really faces.

Running this requires a working simulator. The records it writes are plain data, so
:class:`~experiments.random_events_experiments.orientation_coupling.PlacementSurvey` can
consume them without one.
"""

from __future__ import annotations

import json
import math
import pathlib
from dataclasses import dataclass, field

import numpy as np
import robocasa  # noqa: F401  # registers the kitchen tasks with robosuite
import robosuite
from robosuite.controllers import load_composite_controller_config

ALL_LAYOUTS_AND_STYLES = -3
"""
Layout and style selector that lets a task be instantiated in any kitchen it supports.
"""

UNCONSTRAINED_YAW_RANGE = (-math.pi, math.pi)
"""
Yaws an object may take when a placement leaves its rotation entirely free.
"""

DEFAULT_ROBOT = "PandaOmron"
"""
Robot the tasks are instantiated with, which does not affect where objects are placed
but is required to build a scene.
"""

SAMPLER_SUFFIX = "_Sampler"
"""
Suffix RoboCasa appends to an object's name when it names that object's sampler.
"""


@dataclass
class ResolvedPlacement:
    """
    One placement as a built scene resolved it.

    The fields are the quantities that drive the coupling between where an object may go
    and how it may be turned, kept as plain numbers so they survive a round trip through
    a file.
    """

    task: str
    """
    Task the placement belongs to.
    """

    object_name: str
    """
    Name the scene gave the placed object.
    """

    region_width: float
    """
    Extent of the resolved region along its first axis.
    """

    region_depth: float
    """
    Extent of the resolved region along its second axis.
    """

    footprint_width: float
    """
    Extent of the object's base along its own first axis.
    """

    footprint_depth: float
    """
    Extent of the object's base along its own second axis.
    """

    yaw_lower: float
    """
    Smallest yaw the object may be placed at.
    """

    yaw_upper: float
    """
    Largest yaw the object may be placed at.
    """

    reference_yaw: float
    """
    Yaw of the frame the region is axis aligned in.
    """

    fit_is_enforced: bool
    """
    Whether the scene requires the object's footprint to stay inside the region, which is
    what couples its position to its orientation.
    """


@dataclass
class TaskPlacementCollector:
    """
    Collects the placements a set of RoboCasa tasks resolve across their layouts.
    """

    task_names: list[str]
    """
    Tasks to instantiate.
    """

    resets_per_task: int = 10
    """
    How often each task is rebuilt, each time in a freshly sampled kitchen.
    """

    records: list[ResolvedPlacement] = field(default_factory=list)
    """
    Placements collected so far.
    """

    def collect(self) -> list[ResolvedPlacement]:
        """
        :return: Every placement resolved while rebuilding the tasks.
        """
        controller = load_composite_controller_config(
            controller=None, robot=DEFAULT_ROBOT
        )
        for task_name in self.task_names:
            self.collect_task(task_name, controller)
        return self.records

    def collect_task(self, task_name: str, controller: dict) -> None:
        """
        :param task_name: Task to instantiate.
        :param controller: Controller configuration the scene needs to be built.
        """
        environment = robosuite.make(
            env_name=task_name,
            robots=DEFAULT_ROBOT,
            controller_configs=controller,
            has_renderer=False,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            use_object_obs=True,
            ignore_done=True,
            seed=0,
            layout_ids=ALL_LAYOUTS_AND_STYLES,
            style_ids=ALL_LAYOUTS_AND_STYLES,
        )
        for _ in range(self.resets_per_task):
            environment.reset()
            self.records.extend(self.placements_of(environment, task_name))
        environment.close()

    def placements_of(self, environment, task_name: str) -> list[ResolvedPlacement]:
        """
        :param environment: A freshly built scene.
        :param task_name: Task the scene belongs to.
        :return: The placements the scene resolved for objects with a known footprint.
        """
        placements = []
        for sampler_name, sampler in environment.placement_initializer.samplers.items():
            object_name = sampler_name[: -len(SAMPLER_SUFFIX)]
            placed_object = environment.objects.get(object_name)
            if placed_object is None or not hasattr(placed_object, "get_bbox_points"):
                continue
            placements.append(
                self.describe(sampler, placed_object, object_name, task_name)
            )
        return placements

    def describe(
        self, sampler, placed_object, object_name: str, task_name: str
    ) -> ResolvedPlacement:
        """
        :param sampler: The sampler the scene built for this object.
        :param placed_object: The object being placed.
        :param object_name: Name the scene gave the object.
        :param task_name: Task the scene belongs to.
        :return: The placement reduced to the quantities that drive the coupling.
        """
        corners = placed_object.get_bbox_points()
        yaw_lower, yaw_upper = self.yaw_range_of(sampler)
        return ResolvedPlacement(
            task=task_name,
            object_name=object_name,
            region_width=float(sampler.x_range[1] - sampler.x_range[0]),
            region_depth=float(sampler.y_range[1] - sampler.y_range[0]),
            footprint_width=abs(float(corners[1][0] - corners[0][0])),
            footprint_depth=abs(float(corners[2][1] - corners[0][1])),
            yaw_lower=yaw_lower,
            yaw_upper=yaw_upper,
            reference_yaw=float(sampler.reference_rot),
            fit_is_enforced=bool(sampler.ensure_object_boundary_in_range),
        )

    @staticmethod
    def yaw_range_of(sampler) -> tuple[float, float]:
        """
        A placement may leave the yaw free, pin it to one value, give a range, or offer
        several ranges to choose between.

        :param sampler: The sampler the scene built.
        :return: The smallest and largest yaw the object may take.
        """
        rotation = sampler.rotation
        if rotation is None:
            return UNCONSTRAINED_YAW_RANGE
        if np.ndim(rotation) == 0:
            return float(rotation), float(rotation)
        values = np.array(rotation, dtype=float).ravel()
        return float(values.min()), float(values.max())


SURVEYED_TASKS = [
    "TurnOnBlender",
    "TurnOnToaster",
    "TurnOnMicrowave",
    "AddMarshmallow",
    "RetrieveIceTray",
    "PlaceIceInCup",
]
"""
Tasks the survey covers, spanning appliances, counters and the stools and dining
counters whose frames RoboCasa treats as a special case.
"""


def main():
    collector = TaskPlacementCollector(SURVEYED_TASKS)
    records = collector.collect()
    destination = pathlib.Path("robocasa_placements.json")
    destination.write_text(
        json.dumps([record.__dict__ for record in records], indent=2)
    )
    print(f"wrote {len(records)} placements to {destination.resolve()}")


if __name__ == "__main__":
    main()
