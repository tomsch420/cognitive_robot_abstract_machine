"""
Renders the two Graph of Convex Sets (GCS) demonstration figures used in the thesis'
"Motion Planning with Graphs of Convex Sets" section, both built from the real IAI
kitchen (``semantic_digital_twin/resources/urdf/kitchen.urdf``) rather than a toy
scene, using the current plotting modules
(:mod:`~semantic_digital_twin.world_description.graph_of_convex_sets.plotting` and
:mod:`~semantic_digital_twin.world_description.graph_of_convex_sets.volume_figure`).

Two scenes are rendered:

  * **Floor plan navigation** (:func:`kitchen_navigation_scene`) -- a
    :class:`PlanarGraphOfBoundingBoxes` over the whole kitchen, planning a base
    navigation path to the cabinet holding :data:`DRAWER_BODY_NAME`, from whichever
    location the kitchen's layout makes hardest to reach. Rendered with
    :class:`GraphOfConvexSetsFigure`.

  * **Shelf to counter transport** (:func:`shelf_to_counter_scene`) -- a
    :class:`VolumetricGraphOfBoundingBoxes` plans a path from inside
    :data:`DRAWER_BODY_NAME`'s opened compartment to :data:`COUNTER_BODY_NAME`, the
    kitchen island's countertop, changing height to clear the drawer's guard rails on
    the way out. Rendered with :class:`GraphOfConvexSetsVolumeFigure`.

Run with::

    python -m experiments.graph_of_convex_sets_thesis_figures

To identify a kitchen obstacle by eye (e.g. to point either scene at a different
cabinet), publish the kitchen and the current 2D query to RViz instead::

    python -m experiments.graph_of_convex_sets_thesis_figures --rviz-kitchen
"""

from __future__ import annotations

import argparse
import contextlib
import threading
import time
from pathlib import Path

import rustworkx as rx
from typing_extensions import List, Tuple

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticEnvironmentAnnotation,
)
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import PrismaticConnection
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.graph_of_convex_sets.base import PointT
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    BoxT,
    GraphOfBoundingBoxes,
    PlanarGraphOfBoundingBoxes,
    VolumetricGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.plotting import (
    GraphOfConvexSetsFigure,
    NavigationPath,
    NavigationScene,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.volume_figure import (
    GraphOfConvexSetsVolumeFigure,
    SceneCamera,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)

OUTPUT_DIRECTORY = Path(__file__).parent / "output" / "gcs_thesis_figures"
"""
Directory the rendered figures are written to.
"""

CLEARANCE = 0.1
"""
Amount every obstacle is bloated by while building a graph, standing in for the
robot's own radius.
"""


# %% picking a demonstrative query


def _coordinates_of(
    graph: GraphOfBoundingBoxes[BoxT, PointT], index: int
) -> Tuple[float, float, float]:
    """
    :param graph: The graph holding the convex set.
    :param index: The index of the convex set in that graph.
    :return: The center of the convex set, as plain floats to compare by. The third
        coordinate reads as 0 for a planar graph, whose centers have no z.
    """
    center = graph.graph[index].center
    z = float(center.z) if isinstance(center, Point3) else 0.0
    return float(center.x), float(center.y), z


def hardest_path_query(
    graph: GraphOfBoundingBoxes[BoxT, PointT],
) -> Tuple[PointT, PointT]:
    """
    Pick the query that is hardest to answer: the two convex set centers whose
    shortest path through the graph is the longest one it holds.

    Distance is measured along the graph rather than straight-line, so the query
    lands on the pair the environment actually forces the longest detour between.
    Ties are broken by coordinate so that the same graph always yields the same
    query.

    :param graph: The graph to query.
    :return: The start and the goal.
    """
    path_lengths = rx.all_pairs_dijkstra_path_lengths(
        graph.graph, edge_cost_fn=lambda adjacency: adjacency.distance
    )
    coordinates = {
        index: _coordinates_of(graph, index) for index in graph.graph.node_indices()
    }
    connected_pairs = [
        (source, target)
        for source, targets in path_lengths.items()
        for target in targets
        if source < target
    ]
    if not connected_pairs:
        raise RuntimeError("No two convex sets of this graph are connected.")

    most_distant_pair = max(
        connected_pairs,
        key=lambda pair: (
            path_lengths[pair[0]][pair[1]],
            *sorted((coordinates[pair[0]], coordinates[pair[1]])),
        ),
    )
    start_index, goal_index = sorted(
        most_distant_pair, key=lambda index: coordinates[index]
    )
    return graph.graph[start_index].center, graph.graph[goal_index].center


# %% floor plan navigation in the IAI kitchen


def _minimal_search_space(world: World, xy_widen: float = 1.0) -> BoundingBoxCollection:
    """
    :param world: The world to cover.
    :param xy_widen: Total amount to widen the minimal covering box by in x and y,
        split evenly between both sides. z is left tight to the world's own extent.
    :return: The minimal box covering every obstacle in world, widened in x and y.
    """
    origin = HomogeneousTransformationMatrix(reference_frame=world.root)
    obstacles = SemanticEnvironmentAnnotation(
        root=world.root, _world=world
    ).as_bounding_box_collection_at_origin(origin)
    boxes = list(obstacles)
    half_widen = xy_widen / 2.0
    return BoundingBoxCollection(
        shapes=[
            VolumetricBoundingBox(
                min_x=min(box.min_x for box in boxes) - half_widen,
                min_y=min(box.min_y for box in boxes) - half_widen,
                min_z=min(box.min_z for box in boxes),
                max_x=max(box.max_x for box in boxes) + half_widen,
                max_y=max(box.max_y for box in boxes) + half_widen,
                max_z=max(box.max_z for box in boxes),
                origin=origin,
            )
        ],
        reference_frame=world.root,
    )


def _kitchen_urdf_path() -> Path:
    """
    :return: Path to the full IAI kitchen URDF.
    """
    return (
        Path(__file__).parent
        / ".."
        / ".."
        / ".."
        / "semantic_digital_twin"
        / "resources"
        / "urdf"
        / "kitchen.urdf"
    )


DRAWER_BODY_NAME = "oven_area_area_right_drawer_main"
"""
The body whose opened surface serves as the "shelf" the 3D scene picks an object up
from, and the body the 2D navigation scene below drives the robot to.
"""


def _farthest_reachable_point(
    graph: GraphOfBoundingBoxes[BoxT, PointT], from_point: PointT
) -> PointT:
    """
    :param graph: The graph to search.
    :param from_point: The point to measure distance from; must lie in one of
        graph's convex sets.
    :return: The center of whichever convex set graph forces the longest detour to
        reach from from_point, ties broken by coordinate so the same graph always
        yields the same point.
    """
    from_index = graph.box_to_index_map[graph.node_of_point(from_point)]
    lengths = rx.dijkstra_shortest_path_lengths(
        graph.graph, from_index, edge_cost_fn=lambda adjacency: adjacency.distance
    )
    if not lengths:
        raise RuntimeError("No other convex set is reachable from this point.")
    farthest_index = max(
        lengths, key=lambda index: (lengths[index], _coordinates_of(graph, index))
    )
    return graph.graph[farthest_index].center


def kitchen_navigation_scene() -> NavigationScene:
    """
    :return: The floor plan navigation scene over the full IAI kitchen: a path from
        whichever location the kitchen's layout makes hardest to reach, to the
        cabinet holding :data:`DRAWER_BODY_NAME`, the same shelf the 3D transport
        scene below picks an object up from.
    """
    world = URDFParser.from_file(str(_kitchen_urdf_path())).parse()
    search_space = _minimal_search_space(world)

    graph = PlanarGraphOfBoundingBoxes.navigation_map_from_world(
        world=world, search_space=search_space, bloat_obstacles=CLEARANCE
    )

    shelf_body = next(
        body for body in world.bodies if body.name.name == DRAWER_BODY_NAME
    )
    shelf_x, shelf_y = world.compute_forward_kinematics_np(world.root, shelf_body)[
        :2, 3
    ]
    goal = min(
        graph.graph.nodes(),
        key=lambda node: (float(node.center.x) - shelf_x) ** 2
        + (float(node.center.y) - shelf_y) ** 2,
    ).center
    start = _farthest_reachable_point(graph, goal)
    waypoints = graph.path_from_to(start, goal)

    origin = HomogeneousTransformationMatrix(reference_frame=world.root)
    obstacles = SemanticEnvironmentAnnotation(
        root=world.root, _world=world
    ).as_bounding_box_collection_at_origin(origin)

    return NavigationScene(
        graph_of_convex_sets=graph,
        environment_name="iai_kitchen",
        path=NavigationPath(waypoints),
        obstacles=obstacles,
    )


# %% shelf to counter transport, in the real IAI kitchen


DRAWER_AXIS = (0.999, 0.0, 0.044)
"""
The drawer's real slide axis, read straight from ``kitchen.urdf`` (the joint named
``oven_area_area_right_drawer_joint``).

Works around a bug in :mod:`semantic_digital_twin.adapters.urdf`, which parses a
joint's axis via ``map(int, joint.axis)``: every other drawer in the kitchen has an
exact ``[1, 0, 0]`` axis, where truncating to ``int`` is a no-op, but this joint's
axis is deliberately tilted (its own comment in the URDF: "the axis needs to be
slightly tilted so that the drawers would stay closed in Gazebo"), so the int cast
truncates it to a zero vector and the drawer cannot move at all. Fixed here by
replacing the connection rather than by editing the parser.
"""

DRAWER_OPEN_POSITION = 0.45
"""
How far the drawer is slid out, in meters, close to its 0.48 m upper limit.
"""

COUNTER_BODY_NAME = "kitchen_island_surface"
"""
The kitchen island's countertop, used as the "counter" an object is placed down on.
"""

MANIPULATION_CLEARANCE = 0.015
"""
Clearance obstacles are bloated by for this scene, in place of :data:`CLEARANCE`.

Small enough that the start point still fits inside the drawer's own guard rails,
which leave under 5 cm of headroom above its floor: :data:`CLEARANCE`, sized for a
robot's own body during navigation, would alone consume more height than the
drawer's compartment has, leaving no valid point inside it at all.
"""

ABOVE_SURFACE = 0.01
"""
Height an object is placed above a surface, clear of the clearance margin obstacles
are bloated by.
"""

SEARCH_SPACE_MIN = (-1.6, 0.3, 0.0)
SEARCH_SPACE_MAX = (1.9, 2.95, 1.6)
"""
Extent of the region the scene is built in: tightly wrapping the kitchen island and
oven cabinet footprints (rather than the room around them), so the path between them
fills the frame instead of sitting off to one side of a mostly empty box.
"""

SHELF_TO_COUNTER_CAMERA = SceneCamera(eye=Point3(1.65, -0.66, 2.5))
"""
A steep, elevated viewpoint chosen so both the drawer and the counter, and the path
between them, stay in frame together.
"""


def _open_kitchen_drawer(world: World) -> None:
    """
    Replace :data:`DRAWER_BODY_NAME`'s connection with one carrying its correct axis
    (see :data:`DRAWER_AXIS`), then slide it out to :data:`DRAWER_OPEN_POSITION`.

    :param world: The kitchen world to modify in place.
    """
    drawer = next(body for body in world.bodies if body.name.name == DRAWER_BODY_NAME)
    broken = drawer.parent_connection
    with world.modify_world():
        world.remove_connection(broken)
        fixed = PrismaticConnection.create_with_dofs(
            world=world,
            parent=broken.parent,
            child=broken.child,
            name=broken.name,
            parent_T_connection_expression=broken.parent_T_connection_expression,
            connection_T_child_expression=broken.connection_T_child_expression,
            multiplier=broken.multiplier,
            offset=broken.offset,
            dof_limits=broken.raw_dof.limits,
            axis=Vector3(*DRAWER_AXIS, reference_frame=broken.parent),
        )
        world.add_connection(fixed)
    drawer.parent_connection.position = DRAWER_OPEN_POSITION


def _box_of(
    obstacles: BoundingBoxCollection, world: World, body_name: str
) -> VolumetricBoundingBox:
    """
    :param obstacles: The obstacle collection to search.
    :param world: The world body_name's current position is resolved in.
    :param body_name: Name of the body to find the box of.
    :return: The box in obstacles whose center is closest to body_name's current
        position.
    """
    body = next(b for b in world.bodies if b.name.name == body_name)
    position = world.compute_forward_kinematics_np(world.root, body)[:3, 3]
    return min(
        obstacles,
        key=lambda box: (
            ((box.min_x + box.max_x) / 2 - position[0]) ** 2
            + ((box.min_y + box.max_y) / 2 - position[1]) ** 2
            + ((box.min_z + box.max_z) / 2 - position[2]) ** 2
        ),
    )


def shelf_to_counter_scene() -> NavigationScene:
    """
    :return: The volumetric transport scene over the real IAI kitchen: a path from
        inside the opened oven drawer's compartment to just above the kitchen
        island's countertop.
    """
    world = URDFParser.from_file(str(_kitchen_urdf_path())).parse()
    _open_kitchen_drawer(world)

    origin = HomogeneousTransformationMatrix(reference_frame=world.root)
    search_space = BoundingBoxCollection(
        [VolumetricBoundingBox(*SEARCH_SPACE_MIN, *SEARCH_SPACE_MAX, origin)],
        world.root,
    )
    obstacles = SemanticEnvironmentAnnotation(
        root=world.root, _world=world
    ).as_bounding_box_collection_at_origin(origin)

    drawer_box = _box_of(obstacles, world, DRAWER_BODY_NAME)
    counter_box = _box_of(obstacles, world, COUNTER_BODY_NAME)

    shelf_point = Point3(
        (drawer_box.min_x + drawer_box.max_x) / 2,
        (drawer_box.min_y + drawer_box.max_y) / 2,
        drawer_box.max_z + MANIPULATION_CLEARANCE + ABOVE_SURFACE,
        reference_frame=world.root,
    )
    counter_point = Point3(
        (counter_box.min_x + counter_box.max_x) / 2,
        (counter_box.min_y + counter_box.max_y) / 2,
        counter_box.max_z + MANIPULATION_CLEARANCE + ABOVE_SURFACE,
        reference_frame=world.root,
    )

    graph = VolumetricGraphOfBoundingBoxes.free_space_from_world(
        world=world, search_space=search_space, bloat_obstacles=MANIPULATION_CLEARANCE
    )
    waypoints = graph.path_from_to(shelf_point, counter_point)

    return NavigationScene(
        graph_of_convex_sets=graph,
        environment_name="shelf_to_counter",
        path=NavigationPath(waypoints),
        obstacles=obstacles,
    )


# %% identifying kitchen obstacles by eye


def _nearby_bodies(world: World, x: float, y: float, count: int = 8) -> List[str]:
    """
    :param world: The world to search.
    :param x: The x coordinate to search around.
    :param y: The y coordinate to search around.
    :param count: How many of the closest bodies to report.
    :return: One line per body, nearest first, naming its distance and position.
    """
    ranked = []
    for body in world.bodies:
        try:
            position = world.compute_forward_kinematics(world.root, body).to_position()
        except Exception:
            continue
        body_x, body_y = float(position.x), float(position.y)
        distance = ((body_x - x) ** 2 + (body_y - y) ** 2) ** 0.5
        ranked.append((distance, body.name.name, body_x, body_y))
    ranked.sort(key=lambda entry: entry[0])
    return [
        f"  {distance:5.2f} m  {name:45s} ({body_x:.2f}, {body_y:.2f})"
        for distance, name, body_x, body_y in ranked[:count]
    ]


@contextlib.contextmanager
def _rclpy_node():
    """
    Initialise an rclpy node and spin it in a background thread, the same way
    ``rclpy_node`` in ``test/conftest.py`` does for every ROS test in this repo.

    :raises ValueError: If rclpy is not installed.
    """
    from semantic_digital_twin.utils import rclpy_installed

    if not rclpy_installed():
        raise ValueError("No ros installed")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("gcs_kitchen_debug")

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)
    try:
        yield node
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


def debug_kitchen_in_rviz() -> None:
    """
    Publish the IAI kitchen and the current 2D navigation query's start and goal to
    RViz, and print the bodies closest to each, so a cabinet can be identified by eye
    (and by name) instead of guessed at from the automatically picked query.

    Also prints the bodies nearest each query point, in case RViz is unavailable or a
    quick name lookup is enough.

    In RViz, add:
      * A "TF" display, with the fixed frame set to the printed root frame name.
      * A "MarkerArray" display on ``/semworld/viz_marker`` (set Durability Policy to
        Transient Local) for the kitchen geometry.
      * A "MarkerArray" display on ``/gcs_debug_query`` for the query: a green sphere
        at the start, a red sphere at the goal.

    Runs until interrupted with Ctrl+C.
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray

    from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
        VizMarkerPublisher,
    )

    def _sphere_marker(
        marker_id: int, x: float, y: float, color: Tuple[float, float, float]
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "gcs_debug_query"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        return marker

    world = URDFParser.from_file(str(_kitchen_urdf_path())).parse()
    search_space = _minimal_search_space(world)
    graph = PlanarGraphOfBoundingBoxes.navigation_map_from_world(
        world=world, search_space=search_space, bloat_obstacles=CLEARANCE
    )
    start, goal = hardest_path_query(graph)
    start_x, start_y = float(start.x), float(start.y)
    goal_x, goal_y = float(goal.x), float(goal.y)

    print(f"start=({start_x:.2f}, {start_y:.2f}); bodies nearby:")
    print("\n".join(_nearby_bodies(world, start_x, start_y)))
    print(f"goal=({goal_x:.2f}, {goal_y:.2f}); bodies nearby:")
    print("\n".join(_nearby_bodies(world, goal_x, goal_y)))

    with _rclpy_node() as node:
        tf_publisher = TFPublisher(node=node, _world=world)
        # The RViz "Fixed Frame" must match this exact string, not world.root.name.name:
        # TF frame ids are the body's full PrefixedName ("iai_oven_area/world"), not
        # its bare name ("world") -- a fixed frame set to the bare name does not exist
        # in the published tf tree, so nothing can be transformed into it and RViz
        # silently renders no markers at all.
        frame_id = tf_publisher.tf_model_callback.frame_names.assign(world.root)
        VizMarkerPublisher(_world=world, node=node)

        query_publisher = node.create_publisher(
            MarkerArray,
            "/gcs_debug_query",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        query_publisher.publish(
            MarkerArray(
                markers=[
                    _sphere_marker(0, start_x, start_y, (0.1, 0.8, 0.2)),
                    _sphere_marker(1, goal_x, goal_y, (0.9, 0.1, 0.1)),
                ]
            )
        )

        print(f"\nFixed frame: {frame_id}")
        print(
            "Publishing to RViz: TF, MarkerArray on /semworld/viz_marker "
            "(Durability Policy: Transient Local), MarkerArray on /gcs_debug_query."
        )
        print("Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass


# %% entry point


def main() -> List[Path]:
    """
    Render both figures and write them to :data:`OUTPUT_DIRECTORY`.

    :return: The paths written.
    """
    written = []
    written += GraphOfConvexSetsFigure(kitchen_navigation_scene()).save(
        OUTPUT_DIRECTORY
    )
    written += GraphOfConvexSetsVolumeFigure(
        shelf_to_counter_scene(), camera=SHELF_TO_COUNTER_CAMERA
    ).save(OUTPUT_DIRECTORY)
    for path in written:
        print(path)
    return written


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rviz-kitchen",
        action="store_true",
        help="Publish the IAI kitchen and the current 2D query to RViz for "
        "interactive inspection, instead of rendering figures.",
    )
    arguments = parser.parse_args()
    if arguments.rviz_kitchen:
        debug_kitchen_in_rviz()
    else:
        main()
