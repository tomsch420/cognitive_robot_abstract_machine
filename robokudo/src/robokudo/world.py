"""General methods to access the current World. Reasoning about alternate world states is done in the corresponding
Annotators.
"""

import sys
from threading import Lock
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World, Body
from semantic_digital_twin.world_description.connections import Connection6DoF

# Module-level singleton-like variables
this = sys.modules[__name__]

this.world = World()
"""RoboKudo's central world state."""

this.world_entity_tracker = WorldEntityWithIDKwargsTracker.from_world(this.world)
"""RoboKudo's central entity tracker."""

_rk_world_lock = Lock()
"""Lock for safe creation of the central SemDT World and entity tracker."""


def get_world_entity_tracker() -> WorldEntityWithIDKwargsTracker:
    """Get the entity tracker instance of for the current world.

    :return: The current entity tracker instance.
    """
    return this.world_entity_tracker


def init_world_with_entity_tracker() -> WorldEntityWithIDKwargsTracker:
    """Initialize the world and entity tracker and return the entity tracker.

    :return: The newly created entity tracker instance.
    """
    with _rk_world_lock:
        unsafe_clear_world()
        this.world_entity_tracker = WorldEntityWithIDKwargsTracker.from_world(
            world_instance()
        )
    return this.world_entity_tracker


def init_world_entity_tracker_from_world(
    world: World,
) -> WorldEntityWithIDKwargsTracker:
    """Initialize the entity tracker from the given world and return the entity tracker.

    :return: The newly created entity tracker instance.
    """
    with _rk_world_lock:
        this.world_entity_tracker = WorldEntityWithIDKwargsTracker.from_world(world)
    return this.world_entity_tracker


def world_instance() -> World:
    """Return the world state for the currently running perception pipeline.

    .. warning::
    This is NOT necessarily the belief state World based on the previous analysis results.

    :return: A singleton-like World instance.
    """
    if this.world is None:
        with _rk_world_lock:
            # Check again with lock
            if this.world is None:
                this.unsafe_clear_world()

        # Setup of this world is currently the responsibility of the other nodes, loaded URDF
        # and/or camera interface.
    return this.world


def set_world(world: World) -> None:
    """Clear the world state safely by overwriting it with the given instance.

    :param world: The new world state.
    """
    with _rk_world_lock:
        this.unsafe_set_world(world=world)


def unsafe_set_world(world: World) -> None:
    """Unsafely set the world state without acquiring the lock.

    .. warning::
        Always acquire the lock manually before calling this method. Take a look at `this.set_world()` or
        `this.init_world_with_entity_tracker()` for example.

    :param world: The new world state to set.
    """
    this.world = world


def clear_world() -> None:
    """Clear the world state by instantiating a new World."""
    with _rk_world_lock:
        this.unsafe_clear_world()


def unsafe_clear_world() -> None:
    """Unsafely clear the world state without acquiring the lock.

    .. warning::
        Always acquire the lock manually before calling this method. Take a look at `this.clear_world()` or
        `this.world_instance()` for examples.
    """
    with this.world.modify_world():
        this.world.clear()


def world_has_body_by_name(world: World, body_name: str) -> bool:
    """Check whether a body with a given name exists in the given world.

    :param world: The world to check in.
    :param body_name: The body name to search for.
    :return: True if the body exists, False otherwise.
    """
    bodies = world.get_bodies_by_name(name=body_name)
    return len(bodies) > 0


# def add_dummy_frame_if_non_existent(frame_name: str) -> None:
#     if not world_has_body_by_name(world=world_instance(), body_name=frame_name):
#         with world_instance().modify_world():
#             world_instance().add_body(
#                 semantic_digital_twin.world.Body(name=PrefixedName(name=frame_name)))


def setup_world_for_camera_frame(
    world_frame: str,
    camera_frame: str,
) -> None:
    """Set up the world and camera frames if they do not exist yet.

    :param world_frame: The name of the world frame.
    :param camera_frame: The name of the camera frame.
    """
    world = world_instance()
    world_exists = world_has_body_by_name(world=world, body_name=world_frame)
    camera_exists = world_has_body_by_name(world=world, body_name=camera_frame)

    if world_exists and camera_exists:
        return None

    if not world_exists and not camera_exists:
        world_body = Body(name=PrefixedName(name=world_frame))
        world_body.visual.append(
            Mesh.from_trimesh(
                mesh=trimesh.creation.axis(),
                origin=HomogeneousTransformationMatrix(reference_frame=world_body),
            )
        )

        camera_body = Body(name=PrefixedName(name=camera_frame))
        camera_body.visual.append(
            Mesh.from_trimesh(
                mesh=trimesh.creation.axis(),
                origin=HomogeneousTransformationMatrix(reference_frame=camera_body),
            )
        )

        with world.modify_world():
            cam_T_world = Connection6DoF.create_with_dofs(
                parent=world_body,
                child=camera_body,
                world=world,
                name=PrefixedName(name=f"{camera_frame}_T_{world_frame}"),
            )

            world.add_connection(cam_T_world)
        return None

    raise AssertionError(
        f"This method can currently only be called when neither the world or camera frame exist. "
        f"Existence of camera frame: {camera_exists}, world frame: {world_exists}."
    )


def update_connection_transform(
    to_name: PrefixedName,
    from_name: PrefixedName,
    transform: HomogeneousTransformationMatrix,
) -> None:
    world = world_instance()

    connection = world.get_connection_by_name(
        PrefixedName(name=f"{from_name.name}_T_{to_name.name}")
    )

    with world.modify_world():
        connection.origin = transform


def get_object_belief_states() -> Dict[UUID, ObjectBeliefState]:
    return _tracked_objects


def add_object_belief_state(object_belief_state: ObjectBeliefState) -> None:
    world = world_instance()

    _tracked_objects[object_belief_state.uuid] = object_belief_state

    # with world.modify_world():
    #     world_T_object_belief = Connection6DoF.create_with_dofs(
    #         world=world,
    #         parent=world_frame,
    #         child=object_belief.body,
    #     )

    #     world.add_connection(world_T_object_belief)


def get_object_belief_state(uuid: UUID) -> ObjectBeliefState:
    return _tracked_objects[uuid]
