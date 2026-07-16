"""
Build the Montessori shape-sorting world and an HSRB robot in a semantic digital twin
world, visualize it live in RViz, and have the robot sort every loose shape into its
matching hole.

Run with (the ``experiments`` package must be importable)::

    python -m experiments.montessori.montessori_demo

.. note::
    Requires a sourced ROS 2 workspace with ``rclpy`` installed; add a ``MarkerArray``
    display in RViz2 for the topic printed at startup (with
    ``DurabilityPolicy.TRANSIENT_LOCAL``) to see the scene. The HSRB robot
    additionally requires the ``hsr_description`` ROS package to be built and
    sourced; without it, the scene is spawned without the robot, and no shapes are
    inserted.
"""

from __future__ import annotations

import logging
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import Arms
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import execute_single
from experiments.montessori.insert_shape_action import InsertMontessoriShapeAction
from experiments.montessori.semantics import MontessoriShape, NoMatchingHoleError
from experiments.montessori.world import MontessoriWorld
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.utils import hsrb_installed

logger = logging.getLogger(__name__)


def _insert_all_shapes(montessori: MontessoriWorld) -> None:
    """
    Have the HSRB robot pick up and insert every loose shape that has a matching
    hole into the shape-sorting board, skipping any that don't (e.g. the sphere).

    :param montessori: The Montessori scene, with :attr:`MontessoriWorld.hsrb`
        already spawned.
    """
    context = Context(montessori.world, montessori.hsrb)
    for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape):
        try:
            montessori.board.hole_for(shape)
        except NoMatchingHoleError:
            logger.info("Skipping %s: no matching hole.", shape.name)
            continue

        logger.info("Inserting %s into its matching hole.", shape.name)
        action = InsertMontessoriShapeAction(
            montessori_shape=shape, board=montessori.board, arm=Arms.RIGHT
        )
        with simulated_robot:
            node = execute_single(action, context=context)
            node.perform()


def main() -> None:
    """
    Build the Montessori world, visualize it in RViz, have the robot sort the loose
    shapes into the board, and keep the live viewer open until interrupted.
    """
    logging.basicConfig(level=logging.INFO)

    montessori = MontessoriWorld()

    if hsrb_installed():
        montessori.spawn_hsrb()
    else:
        logger.warning(
            "hsr_description is not installed; spawning the Montessori scene without "
            "the HSRB robot."
        )
    logger.info("Built Montessori world with %d bodies.", len(montessori.world.bodies))

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("montessori_demo")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)

    tf_publisher = TFPublisher(node=node, _world=montessori.world)
    viz_marker_publisher = VizMarkerPublisher(_world=montessori.world, node=node)

    logger.info(
        "Visualizing the Montessori world on topic '%s'.",
        viz_marker_publisher.topic_name,
    )

    if montessori.hsrb is not None:
        _insert_all_shapes(montessori)

    logger.info("Done. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        viz_marker_publisher.stop()
        tf_publisher.stop()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
