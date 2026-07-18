"""
Build the Montessori shape-sorting world and, if ROS 2 is available, an HSRB robot in a
semantic digital twin world, visualize it live in RViz, have the robot sort every loose
shape into its matching hole (physically settling each one under gravity in MuJoCo right
after it is placed, rather than leaving it exactly where it was kinematically teleported
to), and finally physically simulate the finished scene live in MuJoCo.

Run with (the ``experiments`` package must be importable)::

    python -m experiments.montessori.montessori_demo
    python -m experiments.montessori.montessori_demo --headless

.. note::
    ROS 2 (``rclpy``) is optional: without it, the scene is still built and viewable
    through :class:`~experiments.montessori.world.MontessoriWorld` directly, but RViz
    visualization is skipped and, since sorting the shapes requires the ROS-dependent
    CRAM/Giskard motion stack, no robot is spawned, no shapes are inserted, and MuJoCo
    is not started, regardless of whether ``hsr_description`` is installed. With
    ``rclpy`` installed, add a ``MarkerArray`` display in RViz2 for the topic printed
    at startup, with ``DurabilityPolicy.TRANSIENT_LOCAL``, to see the scene. The
    ``mujoco`` dependency (declared by ``semantic_digital_twin``; run ``uv sync`` once
    from the repository root if it is not yet installed) is required either way. The
    HSRB robot additionally requires the ``hsr_description`` ROS package to be built
    and sourced.
"""

from __future__ import annotations

import argparse
import logging
import threading
import time

import mujoco

from experiments.montessori.semantics import MontessoriShape, NoMatchingHoleError
from experiments.montessori.world import MontessoriWorld
from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.utils import clear_memoization_cache
from semantic_digital_twin.adapters.multi_sim import MujocoActuator, MujocoSim
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.utils import hsrb_installed, rclpy_installed
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection,
    ActiveConnection1DOF,
    Connection6DoF,
    FixedConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_digital_twin.world_description.world_entity import Actuator

logger = logging.getLogger(__name__)

ACTUATOR_TIME_CONSTANT = 0.1
"""
MuJoCo actuator ``dynamics_parameters[0]`` for the position-hold actuators added to the
robot's joints.
"""

ARM_ACTUATOR_POSITION_GAIN = 100.0
"""
Proportional gain of the MuJoCo position-hold actuators added to the robot's controlled
joints (arm, wrist, head).
"""

ARM_ACTUATOR_VELOCITY_GAIN = 10.0
"""
Derivative (damping) gain of the MuJoCo position-hold actuators added to the robot's
controlled joints (arm, wrist, head).
"""

BASE_ACTUATOR_POSITION_GAIN = 1.0
"""
Proportional gain of the MuJoCo position-hold actuators added to the mobile base's wheel
joints.

Much lower than :data:`ARM_ACTUATOR_POSITION_GAIN`: the wheels have far less inertia
than an arm link, and holding them with the arm's gain makes the simulation numerically
unstable (``QACC`` diverges within the first few milliseconds and never settles).
"""

BASE_ACTUATOR_VELOCITY_GAIN = 0.1
"""
Derivative (damping) gain of the MuJoCo position-hold actuators added to the mobile
base's wheel joints.
"""

BASE_JOINT_DAMPING = 50.0
"""
Viscous friction (:attr:`JointDynamics.damping`) added to the mobile base's wheel
joints, resisting spinning regardless of the (weak) position-hold actuator.
"""

BASE_JOINT_DRY_FRICTION = 5.0
"""
Dry friction (:attr:`JointDynamics.dry_friction`) added to the mobile base's wheel
joints, resisting spinning regardless of the (weak) position-hold actuator.
"""

MUJOCO_STEP_SIZE = 2e-4
"""
MuJoCo simulation step size used for the finished scene, smaller than MuJoCo's own
default (``1e-3``): the default step is too large for the wheel joints' low inertia
combined with their position hold and contact with the floor, and repeatedly drives
``QACC`` to ``NaN``/``Inf``.
"""


def _insert_all_shapes(montessori: MontessoriWorld, headless: bool) -> None:
    """
    Have the HSRB robot pick up and insert every loose shape that has a matching hole
    into the shape-sorting board, skipping any that don't (e.g. the sphere), physically
    settling each one under gravity in MuJoCo (see :func:`_settle_shape_in_mujoco`)
    immediately after it is placed.

    :param montessori: The Montessori scene, with :attr:`MontessoriWorld.hsrb` already
        spawned and its controlled joints already held (see
        :func:`_hold_controlled_joints_in_mujoco`).
    :param headless: Whether to run the settling MuJoCo simulations without opening a
        viewer window.
    """
    # Imported lazily: coraplex.datastructures.dataclasses pulls in
    # coraplex.plans.executables for GiskardExecutable, which imports rclpy at module
    # level, so this whole chain would make the demo unimportable without ROS 2 even
    # though nothing here runs without it anyway (see rclpy_installed() in main()).
    from coraplex.datastructures.dataclasses import Context
    from coraplex.datastructures.enums import Arms
    from coraplex.execution_environment import simulated_robot
    from coraplex.plans.factories import execute_single
    from experiments.montessori.insert_shape_action import InsertMontessoriShapeAction

    context = Context(
        montessori.world, montessori.hsrb, query_backend=ProbabilisticBackend()
    )
    for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape):
        try:
            montessori.board.hole_for(shape)
        except NoMatchingHoleError:
            logger.info("Skipping %s: no matching hole.", shape.name)
            continue

        logger.info("Inserting %s into its matching hole.", shape.name)
        # World.get_kinematic_structure_entities_of_branch is @memoize'd per
        # (world, root-object) and never invalidated across the attach/detach cycle
        # of the *previous* insertion's pick-and-place, so the next action's
        # gripper-contents query silently returns a stale branch. Clearing it before
        # each insertion forces a fresh read of the actual current world state.
        clear_memoization_cache(montessori.world)
        action = InsertMontessoriShapeAction(
            montessori_shape=shape, board=montessori.board, arm=Arms.RIGHT
        )
        with simulated_robot:
            node = execute_single(action, context=context)
            node.perform()

        logger.info("Settling %s in MuJoCo.", shape.name)
        _settle_shape_in_mujoco(shape, montessori, headless)

        if not action.has_fallen_through_hole():
            logger.warning(
                "%s did not fall through its hole; it may be resting on the board "
                "or wedged in the opening.",
                shape.name,
            )


def _position_hold_actuator(
    position_gain: float, velocity_gain: float
) -> MujocoActuator:
    """
    Build a MuJoCo actuator that holds its degree of freedom at whatever position it had
    when the simulation started, resisting gravity and contacts with a PD law.

    :param position_gain: Proportional gain of the hold.
    :param velocity_gain: Derivative (damping) gain of the hold.
    """
    return MujocoActuator(
        dynamics_type=mujoco.mjtDyn.mjDYN_NONE,
        dynamics_parameters=[ACTUATOR_TIME_CONSTANT] + [0.0] * 9,
        gain_type=mujoco.mjtGain.mjGAIN_FIXED,
        gain_parameters=[position_gain] + [0.0] * 9,
        bias_type=mujoco.mjtBias.mjBIAS_AFFINE,
        bias_parameters=[0, -position_gain, -velocity_gain] + [0.0] * 7,
    )


def _add_position_hold_actuator(
    world: World, dof: DegreeOfFreedom, position_gain: float, velocity_gain: float
) -> None:
    """
    Add a :func:`_position_hold_actuator` for ``dof`` to ``world``.

    :param world: The world to add the actuator to, modified in place.
    :param dof: The degree of freedom to hold.
    :param position_gain: Proportional gain of the hold.
    :param velocity_gain: Derivative (damping) gain of the hold.
    """
    actuator = Actuator()
    actuator.add_dof(dof=dof)
    actuator.simulator_additional_properties.append(
        _position_hold_actuator(position_gain, velocity_gain)
    )
    world.add_actuator(actuator=actuator)


def _base_degrees_of_freedom_without_hardware_interface(
    hsrb: HSRB,
) -> list[DegreeOfFreedom]:
    """
    The degrees of freedom of :func:`_base_connections_without_hardware_interface`.

    :param hsrb: The spawned HSRB robot.
    """
    return [
        connection.raw_dof
        for connection in _base_connections_without_hardware_interface(hsrb)
    ]


def _base_connections_without_hardware_interface(
    hsrb: HSRB,
) -> list[ActiveConnection1DOF]:
    """
    The HSRB's mobile-base joints (drive wheels, passive caster wheels, base roll) that
    have a degree of freedom but, unlike the arm/wrist/head, are not part of
    :attr:`HSRB.degrees_of_freedom_with_hardware_interface`: they are driven indirectly
    through the :class:`OmniDrive` connection rather than controlled directly. Left
    unactuated, MuJoCo's contact and gravity forces spin them up without bound.

    Excludes the gripper's own uncontrolled (mimic/spring) joints, which already move
    together through their mimic relationship and don't need an independent hold.

    :param hsrb: The spawned HSRB robot.
    """
    controlled_dofs = set(hsrb.degrees_of_freedom_with_hardware_interface)
    gripper_dofs = {
        dof
        for connection in hsrb.end_effector.connections
        if isinstance(connection, ActiveConnection)
        for dof in connection.active_dofs
    }

    base_connections = []
    seen_dofs = set(controlled_dofs)
    for connection in hsrb.connections:
        if not isinstance(connection, ActiveConnection1DOF):
            continue
        if connection.raw_dof in seen_dofs or connection.raw_dof in gripper_dofs:
            continue
        seen_dofs.add(connection.raw_dof)
        base_connections.append(connection)
    return base_connections


def _hold_controlled_joints_in_mujoco(hsrb: HSRB) -> None:
    """
    Keep every joint of the robot that would otherwise be left to MuJoCo's own physics
    (arm, wrist, head, and the mobile base's wheels) from sagging or spinning under
    gravity and contacts once MuJoCo starts stepping the world.

    The arm/wrist/head are held with a MuJoCo position-hold actuator
    (:data:`ARM_ACTUATOR_POSITION_GAIN`). The base's wheels additionally get joint
    damping and dry friction (:data:`BASE_JOINT_DAMPING`,
    :data:`BASE_JOINT_DRY_FRICTION`): their low inertia makes the arm's actuator gains
    numerically unstable, and a weak actuator (:data:`BASE_ACTUATOR_POSITION_GAIN`)
    alone is not enough to stop them spinning once the robot has actually driven around
    and is resting at a real, contact-heavy pose rather than its spawn pose.

    :param hsrb: The spawned HSRB robot, modified in place.
    """
    with hsrb._world.modify_world():
        for dof in hsrb.degrees_of_freedom_with_hardware_interface:
            _add_position_hold_actuator(
                hsrb._world, dof, ARM_ACTUATOR_POSITION_GAIN, ARM_ACTUATOR_VELOCITY_GAIN
            )
        for connection in _base_connections_without_hardware_interface(hsrb):
            connection.dynamics.damping = BASE_JOINT_DAMPING
            connection.dynamics.dry_friction = BASE_JOINT_DRY_FRICTION
            _add_position_hold_actuator(
                hsrb._world,
                connection.raw_dof,
                BASE_ACTUATOR_POSITION_GAIN,
                BASE_ACTUATOR_VELOCITY_GAIN,
            )


def _make_shape_movable_in_mujoco(shape: MontessoriShape, world: World) -> None:
    """
    Re-attach a loose shape's body to the world root with a free
    (:class:`Connection6DoF`) joint instead of the rigid connection it currently has,
    preserving its current pose.

    :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction` always re-attaches a
    placed object with a :class:`~semantic_digital_twin.world_description.connections.FixedConnection`
    (a deliberate, framework-wide choice; see the ``# TODO: this shouldn't be fixed but
    6DOF`` comment in ``coraplex.plans.executables.ModelChangeExecutable``), and every
    loose shape starts out :class:`FixedConnection`'d to the world root too. MuJoCo
    treats an unjointed body as welded to its parent, so without this a shape cannot be
    affected by gravity or contacts at all once MuJoCo starts stepping the world.

    :param shape: The shape to make movable.
    :param world: The world the shape belongs to, modified in place.
    """
    current_pose = world.compute_forward_kinematics(world.root, shape.root)
    with world.modify_world():
        world.remove_connection(shape.root.parent_connection)
        world.add_connection(
            Connection6DoF.create_with_dofs(
                world=world,
                parent=world.root,
                child=shape.root,
                parent_T_connection_expression=current_pose,
            )
        )


def _make_all_shapes_movable_in_mujoco(montessori: MontessoriWorld) -> None:
    """
    Apply :func:`_make_shape_movable_in_mujoco` to every loose Montessori shape.

    :param montessori: The Montessori scene, modified in place.
    """
    for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape):
        _make_shape_movable_in_mujoco(shape, montessori.world)


SHAPE_SETTLE_DURATION = 2.0
"""
Real-time seconds a just-inserted shape is given to physically settle under gravity and
contacts in MuJoCo (see :func:`_settle_shape_in_mujoco`) before it is fixed in place and
the next shape is inserted.
"""


def _settle_shape_in_mujoco(
    shape: MontessoriShape, montessori: MontessoriWorld, headless: bool
) -> None:
    """
    Physically settle a single, just-placed shape under gravity and contacts in MuJoCo,
    then fix it in place wherever it comes to rest.

    :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction` only ever teleports
    a shape kinematically to its target pose, so without this every shape is left
    exactly where it was placed rather than where gravity actually settles it (e.g.
    resting on the board's surface instead of having dropped through a hole). Settling
    one shape at a time, right after it is inserted, rather than all of them together
    only once at the very end (see :func:`_simulate_finished_scene_in_mujoco`), also
    avoids MuJoCo resolving several simultaneous tight-clearance contacts in the same
    step, which was observed to make an unrelated shape's contact resolution
    nondeterministic (which of several simultaneously falling shapes gets wedged varied
    from run to run, even though each one settles correctly when dropped alone).

    The robot's controlled joints must already be held (see
    :func:`_hold_controlled_joints_in_mujoco`) before this runs, or they sag/spin under
    gravity for the duration of the settle and are left in that pose once MuJoCo stops.

    :param shape: The just-inserted shape to settle.
    :param montessori: The Montessori scene, modified in place.
    :param headless: Whether to run without opening a MuJoCo viewer window.
    """
    _make_shape_movable_in_mujoco(shape, montessori.world)

    mujoco_sim = MujocoSim(
        world=montessori.world, headless=headless, step_size=MUJOCO_STEP_SIZE
    )
    mujoco_sim.start_simulation()
    time.sleep(SHAPE_SETTLE_DURATION)
    mujoco_sim.stop_simulation()

    montessori.world.update_forward_kinematics()
    settled_pose = montessori.world.compute_forward_kinematics(
        montessori.world.root, shape.root
    )
    with montessori.world.modify_world():
        montessori.world.remove_connection(shape.root.parent_connection)
        montessori.world.add_connection(
            FixedConnection(
                parent=montessori.world.root,
                child=shape.root,
                parent_T_connection_expression=settled_pose,
            )
        )


def _simulate_finished_scene_in_mujoco(
    montessori: MontessoriWorld, headless: bool
) -> MujocoSim:
    """
    Physically simulate the sorted scene in MuJoCo, with the robot's controlled joints
    held in place and the shapes free to move under gravity and contacts.

    Every shape has already individually settled under physics once, right after it was
    inserted (see :func:`_settle_shape_in_mujoco`); this final pass is for live viewing
    of the completed scene, not the shapes' only chance to physically settle.

    :param montessori: The finished Montessori scene, with :attr:`MontessoriWorld.hsrb`
        already spawned.
    :param headless: Whether to run without opening a MuJoCo viewer window.
    :return: The running :class:`MujocoSim`.
    """
    _make_all_shapes_movable_in_mujoco(montessori)

    mujoco_sim = MujocoSim(
        world=montessori.world, headless=headless, step_size=MUJOCO_STEP_SIZE
    )
    mujoco_sim.synchronizer.sync_rate_hz = 20
    mujoco_sim.start_simulation()
    return mujoco_sim


def _parse_arguments() -> argparse.Namespace:
    """
    Parse command-line arguments selecting whether a MuJoCo viewer window is opened.

    :return: The parsed arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run the MuJoCo simulation without opening a viewer window.",
    )
    return parser.parse_args()


def main() -> None:
    """
    Build the Montessori world, visualize it in RViz, have the robot sort the loose
    shapes into the board, physically simulate the finished scene in MuJoCo, and keep
    the live viewer open until interrupted.
    """
    logging.basicConfig(level=logging.INFO)
    arguments = _parse_arguments()

    montessori = MontessoriWorld()

    if hsrb_installed():
        montessori.spawn_hsrb()
        # Must happen before any MuJoCo simulation runs, including the per-shape
        # settling in _insert_all_shapes, or the robot's own joints sag/spin under
        # gravity for that simulation's duration (see
        # _hold_controlled_joints_in_mujoco).
        _hold_controlled_joints_in_mujoco(montessori.hsrb)
    else:
        logger.warning(
            "hsr_description is not installed; spawning the Montessori scene without "
            "the HSRB robot."
        )
    logger.info("Built Montessori world with %d bodies.", len(montessori.world.bodies))

    # Sorting the shapes goes through CRAM's execute_single, which pulls in
    # coraplex.plans.executables for GiskardExecutable, which imports rclpy at module
    # level regardless of whether a real robot ever executes a real ROS 2 motion; RViz
    # visualization needs rclpy directly. Both are skipped without it, rather than the
    # whole demo failing to even import.
    ros_active = rclpy_installed()
    node = executor = thread = tf_publisher = viz_marker_publisher = None
    if ros_active:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
        from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
            VizMarkerPublisher,
        )

        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("montessori_demo")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(
            target=executor.spin, daemon=True, name="rclpy-executor"
        )
        thread.start()
        time.sleep(0.1)

        tf_publisher = TFPublisher(node=node, _world=montessori.world)
        viz_marker_publisher = VizMarkerPublisher(_world=montessori.world, node=node)
        logger.info(
            "Visualizing the Montessori world on topic '%s'.",
            viz_marker_publisher.topic_name,
        )
    else:
        logger.warning("rclpy is not installed; running without RViz visualization.")

    mujoco_sim = None
    if montessori.hsrb is not None and ros_active:
        import experiments.orm.ormatic_interface  # type: ignore

        _insert_all_shapes(montessori, headless=arguments.headless)
        logger.info("Sorting done; starting the MuJoCo simulation.")
        mujoco_sim = _simulate_finished_scene_in_mujoco(
            montessori, headless=arguments.headless
        )
    elif montessori.hsrb is not None:
        logger.warning("rclpy is not installed; skipping sorting and MuJoCo.")

    logger.info("Done. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if mujoco_sim is not None:
            mujoco_sim.stop_simulation()
        if viz_marker_publisher is not None:
            viz_marker_publisher.stop()
        if tf_publisher is not None:
            tf_publisher.stop()
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if ros_active and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
