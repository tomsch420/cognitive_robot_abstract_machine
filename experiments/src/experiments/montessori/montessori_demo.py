"""
Build the Montessori shape-sorting world and an ICub3 robot in a semantic digital twin
world, and visualize it live in Rerun.

Run with (the ``experiments`` package must be importable)::

    python -m experiments.montessori.montessori_demo
    python -m experiments.montessori.montessori_demo --save recording.rrd

.. note::
    Requires the ``rerun-sdk`` dependency (declared by ``semantic_digital_twin``); run
    ``uv sync`` once from the repository root if it is not yet installed. The ICub3
    robot additionally requires the ``iai_icub_description`` ROS package to be built
    and sourced; without it, the scene is spawned without the robot.
"""

from __future__ import annotations

import argparse
import logging
import time

from semantic_digital_twin.adapters.rerun import RerunAdapter, RerunMode
from semantic_digital_twin.utils import icub_installed

from experiments.montessori.world import build_montessori_world, spawn_icub3

logger = logging.getLogger(__name__)


def _parse_arguments() -> argparse.Namespace:
    """
    Parse command-line arguments selecting the visualization target.

    :return: The parsed arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--save",
        default=None,
        help="Save the Rerun recording to this .rrd file instead of spawning a live viewer.",
    )
    return parser.parse_args()


def main() -> None:
    """
    Build the Montessori world, visualize it in Rerun, and keep the live viewer open
    until interrupted.
    """
    logging.basicConfig(level=logging.INFO)
    arguments = _parse_arguments()

    world = build_montessori_world()

    if icub_installed():
        spawn_icub3(world)
    else:
        logger.warning(
            "iai_icub_description is not installed; spawning the Montessori scene "
            "without the ICub3 robot."
        )
    logger.info("Built Montessori world with %d bodies.", len(world.bodies))

    adapter = RerunAdapter(
        _world=world,
        application_id="montessori",
        mode=RerunMode.SAVE if arguments.save else RerunMode.SPAWN,
        target=arguments.save,
    )

    if arguments.save:
        adapter.stop()
        return

    logger.info("Visualizing the Montessori world. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop()


if __name__ == "__main__":
    main()
