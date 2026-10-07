"""
The objects a robot can be asked to pick up, each described by its geometry, its mass,
how it rests on a surface, how its handle is found and a grasp known to lift it.
"""

from __future__ import annotations

import argparse
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path

import coraplex
import experiments
from krrood.exceptions import DataclassException
import numpy as np
import trimesh
from typing_extensions import List, Optional, Type

from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)
from semantic_digital_twin.grasping.handle_finding import (
    ElongatedShape,
    HandleFinder,
    NarrowEndHandleFinder,
    ProtrudingHandleFinder,
)
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.pipeline.mesh_decomposition.coacd import (
    ApproximationMode,
    COACDMeshDecomposer,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bottle,
    Bowl,
    Cereal,
    CheezeIt,
    Container,
    Cookware,
    Cup,
    CuttingKnife,
    Knife,
    Milk,
    Mug,
    Spoon,
    Whisk,
)
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.spatial_types import RotationMatrix
from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection

# %% the geometry of an object


@dataclass
class ObjectGeometry:
    """
    What an object looks like and what it collides as, in meters, in the frame of its
    body.
    """

    visual: trimesh.Trimesh
    """
    The surface the object shows; its handle is found in it.
    """

    collision_parts: List[trimesh.Trimesh]
    """
    The convex parts the object collides as.
    """

    def transformed(self, transform: np.ndarray) -> ObjectGeometry:
        """
        :param transform: A homogeneous transformation.
        :return: A copy of this geometry moved by ``transform``.
        """
        return ObjectGeometry(
            visual=self.visual.copy().apply_transform(transform),
            collision_parts=[
                part.copy().apply_transform(transform) for part in self.collision_parts
            ],
        )


# %% how an object rests on the table


class RestingOrientation(ABC):
    """
    Turns an object's geometry into the orientation it rests on the table in, where the
    z-axis points up and the robot stands towards negative x.
    """

    @abstractmethod
    def rotation(self, geometry: ObjectGeometry) -> np.ndarray:
        """
        :param geometry: The geometry as modeled.
        :return: The homogeneous rotation that turns it into its resting orientation.
        """


@dataclass
class FixedRestingOrientation(RestingOrientation):
    """
    The same rotation for every geometry.
    """

    rotation_matrix: RotationMatrix = field(default_factory=RotationMatrix)
    """
    The rotation; the identity rests the object as it is modeled.
    """

    def rotation(self, geometry: ObjectGeometry) -> np.ndarray:
        return self.rotation_matrix.to_np()


@dataclass
class HandleTowardsRobot(RestingOrientation):
    """
    Lays an elongated object, such as a tool, on its flattest side, its length pointing
    at the robot and its narrower end, taken to be its handle, nearest to the robot.

    The geometry's axes are assumed to run along the object's length, width and
    thickness, as they do for the meshes of tools in the datasets used here.
    """

    end_fraction: float = 0.15
    """
    How much of the object's length, from each end, is compared to find the narrower
    end.
    """

    def rotation(self, geometry: ObjectGeometry) -> np.ndarray:
        vertices = np.concatenate([part.vertices for part in geometry.collision_parts])
        extents = vertices.max(axis=0) - vertices.min(axis=0)
        length_axis = int(np.argmax(extents))
        thickness_axis = int(np.argmin(extents))
        elongated = ElongatedShape(
            points=vertices,
            length_axis=length_axis,
            width_axis=3 - length_axis - thickness_axis,
            end_fraction=self.end_fraction,
        )
        along = np.eye(3)[length_axis]
        if elongated.narrower_end_is_positive():
            along = -along
        up = np.eye(3)[thickness_axis]
        rotation = np.eye(4)
        rotation[0, :3] = along
        rotation[1, :3] = np.cross(up, along)
        rotation[2, :3] = up
        return rotation


# %% describing an object


@dataclass
class ObjectCannotHaveAHandleError(DataclassException):
    """
    Raised when an object description finds a handle for an object whose annotation
    cannot have one.
    """

    object_description: ObjectDescription
    """
    The description that finds a handle.
    """

    def error_message(self) -> str:
        return (
            f"'{self.object_description.body_name}' is described with a handle finder, "
            f"but {self.object_description.semantic_annotation_type.__name__} cannot "
            f"have a handle."
        )

    def suggest_correction(self) -> str:
        return "Annotate the object with a type that has a handle, or find none."


def hollow_object_decomposition() -> COACDMeshDecomposer:
    """
    :return: A decomposition into convex parts fine enough to keep the walls of a hollow
        object thin, so that fingers can straddle them.
    """
    return COACDMeshDecomposer(
        threshold=0.02,
        max_convex_hull=64,
        approximation_mode=ApproximationMode.CONVEX_HULL,
        seed=0,
    )


@dataclass(kw_only=True)
class ObjectDescription(ABC):
    """
    What the simulation needs to know about an object to put it on the table, and what
    an experiment needs to know to grasp it.
    """

    body_name: str
    """
    The name of the object's body in the world and in the simulation.
    """

    semantic_annotation_type: Type[HasGraspCandidates]
    """
    What the object is.
    """

    mass: float
    """
    Mass of the object in kilograms.
    """

    default_grasp: SurfaceGrasp
    """
    A grasp known to lift the object.
    """

    resting_orientation: RestingOrientation = field(
        default_factory=FixedRestingOrientation
    )
    """
    How the object rests on the table.
    """

    handle_finder: Optional[HandleFinder] = None
    """
    Finds the object's handle in its shape, for an object whose annotation can have
    one; ``None`` gives it none.
    """

    def load_geometry(self) -> ObjectGeometry:
        """
        :return: The object's geometry, in the orientation it rests on the table in.
        """
        geometry = self._load_modeled_geometry()
        return geometry.transformed(self.resting_orientation.rotation(geometry))

    @abstractmethod
    def _load_modeled_geometry(self) -> ObjectGeometry:
        """
        :return: The object's geometry, in meters, oriented as its source models it.
        """


@dataclass(kw_only=True)
class MeshFileObjectDescription(ObjectDescription):
    """
    An object whose look is a single mesh file.
    """

    mesh_file: Path
    """
    The mesh the object looks like and, unless :attr:`convex_decomposition` is given,
    collides as.
    """

    meters_per_mesh_unit: float = 1.0
    """
    The length of one unit of the mesh file, in meters.
    """

    convex_decomposition: Optional[COACDMeshDecomposer] = None
    """
    Splits the mesh into convex parts to collide with; ``None`` collides with the mesh
    itself.

    MuJoCo collides with the convex hull of a mesh, which fills a hollow object: no
    finger could reach inside to pinch its wall.
    """

    def _load_modeled_geometry(self) -> ObjectGeometry:
        mesh = trimesh.load_mesh(self.mesh_file)
        mesh.apply_scale(self.meters_per_mesh_unit)
        if self.convex_decomposition is None:
            return ObjectGeometry(visual=mesh, collision_parts=[mesh])
        parts = self.convex_decomposition.apply_to_mesh(Mesh.from_trimesh(mesh=mesh))
        return ObjectGeometry(
            visual=mesh, collision_parts=[part.mesh for part in parts]
        )


@dataclass(kw_only=True)
class RoboCasaObjectDescription(ObjectDescription):
    """
    An object of the RoboCasa dataset, which comes with its own convex collision parts.
    """

    category: str
    """
    The RoboCasa object category, the name of the directory its models lie in.
    """

    instance_index: int = 0
    """
    Which of the category's models to take.
    """

    loader: RoboCasaDatasetLoader = field(default_factory=RoboCasaDatasetLoader)
    """
    Reads the dataset's models; its directory is where the dataset lies.
    """

    def instance(self, instance_index: int) -> RoboCasaObjectDescription:
        """
        :param instance_index: Which of the category's models to take.
        :return: This description for another model of the same category, keeping the
            mass, the default grasp and the way its handle is found.
        """
        return replace(
            self,
            instance_index=instance_index,
            body_name=f"{self.body_name}_{instance_index}",
        )

    def _load_modeled_geometry(self) -> ObjectGeometry:
        world = self.loader.load_object(self.category, self.instance_index)
        object_body = world.bodies_with_collision[0]
        bodies = [
            object_body,
            *world.compute_descendent_child_kinematic_structure_entities(object_body),
        ]
        visual_parts = []
        collision_parts = []
        for body in bodies:
            object_T_body = world.compute_forward_kinematics_np(object_body, body)
            visual_parts += self._meshes_in_object_frame(body.visual, object_T_body)
            collision_parts += self._meshes_in_object_frame(
                body.collision, object_T_body
            )
        return ObjectGeometry(
            visual=trimesh.util.concatenate(visual_parts),
            collision_parts=collision_parts,
        )

    @staticmethod
    def _meshes_in_object_frame(
        shapes: ShapeCollection, object_T_body: np.ndarray
    ) -> List[trimesh.Trimesh]:
        """
        :param shapes: The shapes of one body.
        :param object_T_body: Where that body sits in the object's frame.
        :return: The shapes that are meshes, in the object's frame. Other shapes, such
            as the boxes RoboCasa marks regions with, are left out.
        """
        return [
            shape.mesh.copy().apply_transform(object_T_body @ shape.origin.to_np())
            for shape in shapes
            if isinstance(shape, Mesh)
        ]


# %% the objects


def coraplex_object_mesh(file_name: str) -> Path:
    """
    :param file_name: The name of a mesh file among coraplex's objects.
    :return: The path of that file.
    """
    return (
        Path(coraplex.__file__).resolve().parents[2]
        / "resources"
        / "objects"
        / file_name
    )


def ycb_object_mesh(file_name: str) -> Path:
    """
    :param file_name: The name of a model file of the YCB object set that the
        repository's segmind package ships with one of its recorded episodes.
    :return: The path of that file.
    """
    return (
        Path(experiments.__file__).resolve().parents[3]
        / "segmind"
        / "resources"
        / "fame_episodes"
        / "alessandro_sliding_bueno"
        / "models"
        / file_name
    )


class PickUpObject(Enum):
    """
    The objects whose default grasp was seen to lift them.
    """

    BOWL = MeshFileObjectDescription(
        body_name="bowl",
        mesh_file=coraplex_object_mesh("bowl.stl"),
        semantic_annotation_type=Bowl,
        mass=0.25,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.75, depth=0.002, pitch=0.0, roll=0.0
        ),
        convex_decomposition=hollow_object_decomposition(),
    )
    """
    A small scanned bowl, 14 cm across and 7 cm high, with a wall about 4 mm thick.
    """

    LARGE_BOWL = MeshFileObjectDescription(
        body_name="large_bowl",
        mesh_file=coraplex_object_mesh("apartment_bowl.stl"),
        semantic_annotation_type=Bowl,
        mass=0.35,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.9, depth=0.0015, pitch=0.5, roll=0.0
        ),
        meters_per_mesh_unit=0.001,
        convex_decomposition=hollow_object_decomposition(),
    )
    """
    A bowl 17 cm across and 10 cm high, with a wall about 3 mm thick.

    Taken at its rim from straight above, it tips out of the fingers; the default grasp
    tilts towards it.
    """

    CUP = MeshFileObjectDescription(
        body_name="cup",
        mesh_file=coraplex_object_mesh("jeroen_cup.stl"),
        semantic_annotation_type=Cup,
        mass=0.15,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.92, depth=0.0015, pitch=0.0, roll=0.0
        ),
        convex_decomposition=hollow_object_decomposition(),
    )
    """
    A cup 7 cm across and 16 cm high, with a wall about 3 mm thick.
    """

    MILK = MeshFileObjectDescription(
        body_name="milk",
        mesh_file=coraplex_object_mesh("milk.stl"),
        semantic_annotation_type=Milk,
        mass=0.5,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.7, depth=0.032, pitch=0.0, roll=0.0
        ),
    )
    """
    A milk carton with a square base of 6.4 cm and 19 cm high.
    """

    CEREAL = MeshFileObjectDescription(
        body_name="cereal",
        mesh_file=coraplex_object_mesh("breakfast_cereal.stl"),
        semantic_annotation_type=Cereal,
        mass=0.4,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.8, depth=0.073, pitch=0.0, roll=math.pi / 2
        ),
    )
    """
    A cereal box, 15 cm wide, 6 cm deep and 22 cm high, standing with its wide side to
    the robot; the default grasp closes across its depth.
    """

    BOTTLE = MeshFileObjectDescription(
        body_name="bottle",
        mesh_file=coraplex_object_mesh("Static_CokeBottle.stl"),
        semantic_annotation_type=Bottle,
        mass=0.5,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.85, depth=0.02, pitch=0.0, roll=0.0
        ),
    )
    """
    A plastic bottle 9.4 cm across and 29 cm high, tapering to a neck 4 cm across; the
    default grasp takes the neck.
    """

    SPOON = MeshFileObjectDescription(
        body_name="spoon",
        mesh_file=coraplex_object_mesh("spoon.stl"),
        semantic_annotation_type=Spoon,
        mass=0.04,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.3, depth=0.11, pitch=0.0, roll=math.pi / 2
        ),
        convex_decomposition=hollow_object_decomposition(),
        handle_finder=NarrowEndHandleFinder(),
    )
    """
    A spoon 22 cm long lying on its back, its handle towards the robot; the default
    grasp pinches the handle at its balance point, 11 cm from its end.
    """

    WHISK = MeshFileObjectDescription(
        body_name="whisk",
        mesh_file=coraplex_object_mesh("whisk.stl"),
        semantic_annotation_type=Whisk,
        mass=0.1,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.45, depth=0.16, pitch=0.0, roll=math.pi / 2
        ),
        convex_decomposition=hollow_object_decomposition(),
        handle_finder=NarrowEndHandleFinder(),
    )
    """
    A whisk 30 cm long lying on its side, its handle, 2.4 cm thick, towards the robot;
    the default grasp pinches the handle where it meets the wires, 16 cm from its end.
    """

    KNIFE = MeshFileObjectDescription(
        body_name="knife",
        mesh_file=coraplex_object_mesh("big-knife.stl"),
        semantic_annotation_type=CuttingKnife,
        mass=0.2,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.5, depth=0.13, pitch=0.0, roll=math.pi / 2
        ),
        resting_orientation=FixedRestingOrientation(
            RotationMatrix.from_rpy(roll=math.pi / 2, pitch=0.0, yaw=0.0)
        ),
        convex_decomposition=hollow_object_decomposition(),
        handle_finder=NarrowEndHandleFinder(),
    )
    """
    A kitchen knife 37 cm long lying flat, its handle towards the robot; the mesh stands
    on the blade's edge, so it is turned onto its side.

    The default grasp pinches the handle 13 cm from its end, near the blade; the blade
    then swings down about the fingers.
    """

    YCB_CRACKER_BOX = MeshFileObjectDescription(
        body_name="ycb_cracker_box",
        mesh_file=ycb_object_mesh("obj_000001.ply"),
        semantic_annotation_type=CheezeIt,
        mass=0.411,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.85, depth=0.036, pitch=0.0, roll=0.0
        ),
        meters_per_mesh_unit=0.001,
    )
    """
    The YCB cracker box, 16 cm wide, 7 cm deep and 21 cm high, with its narrow side to
    the robot; the default grasp closes across its depth.
    """

    YCB_BOWL = MeshFileObjectDescription(
        body_name="ycb_bowl",
        mesh_file=ycb_object_mesh("obj_000003.ply"),
        semantic_annotation_type=Bowl,
        mass=0.147,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.9, depth=0.0007, pitch=0.5, roll=0.0
        ),
        meters_per_mesh_unit=0.001,
        convex_decomposition=hollow_object_decomposition(),
    )
    """
    The YCB bowl, 16 cm across and 5.5 cm high, with a wall about 1.5 mm thick.
    """

    YCB_MUG = MeshFileObjectDescription(
        body_name="ycb_mug",
        mesh_file=ycb_object_mesh("obj_000004.ply"),
        semantic_annotation_type=Mug,
        mass=0.118,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.92, depth=0.0007, pitch=0.0, roll=0.0
        ),
        meters_per_mesh_unit=0.001,
        convex_decomposition=hollow_object_decomposition(),
        handle_finder=ProtrudingHandleFinder(),
    )
    """
    The YCB mug, 8 cm across and 8 cm high, its handle away from the robot; the default
    grasp pinches the rim.
    """

    ROBOCASA_SPOON = RoboCasaObjectDescription(
        body_name="robocasa_spoon",
        category="spoon",
        semantic_annotation_type=Spoon,
        mass=0.04,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.95, depth=0.08, pitch=0.6, roll=math.pi / 2
        ),
        resting_orientation=HandleTowardsRobot(),
        handle_finder=NarrowEndHandleFinder(),
    )
    """
    A spoon of the RoboCasa dataset, lying with its handle towards the robot.
    """

    ROBOCASA_KNIFE = RoboCasaObjectDescription(
        body_name="robocasa_knife",
        category="knife",
        semantic_annotation_type=Knife,
        mass=0.08,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.5, depth=0.08, pitch=0.0, roll=math.pi / 2
        ),
        resting_orientation=HandleTowardsRobot(),
        handle_finder=NarrowEndHandleFinder(),
    )
    """
    A table knife of the RoboCasa dataset, lying with its handle towards the robot.
    """

    ROBOCASA_WOODEN_SPOON = RoboCasaObjectDescription(
        body_name="robocasa_wooden_spoon",
        category="wooden_spoon",
        semantic_annotation_type=Cookware,
        mass=0.05,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.71, depth=0.2, pitch=0.0, roll=math.pi / 2
        ),
        resting_orientation=HandleTowardsRobot(),
    )
    """
    A wooden spoon of the RoboCasa dataset, lying with its handle towards the robot.
    """

    ROBOCASA_PEELER = RoboCasaObjectDescription(
        body_name="robocasa_peeler",
        category="peeler",
        semantic_annotation_type=Cookware,
        mass=0.05,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.4, depth=0.12, pitch=0.0, roll=math.pi / 2
        ),
        resting_orientation=HandleTowardsRobot(),
    )
    """
    A peeler of the RoboCasa dataset, lying with its handle towards the robot.
    """

    ROBOCASA_TONGS = RoboCasaObjectDescription(
        body_name="robocasa_tongs",
        category="tongs",
        semantic_annotation_type=Cookware,
        mass=0.1,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.27, depth=0.13, pitch=0.0, roll=math.pi / 2
        ),
        resting_orientation=HandleTowardsRobot(),
    )
    """
    A pair of tongs of the RoboCasa dataset, lying with its handle towards the robot.
    """

    ROBOCASA_MUG = RoboCasaObjectDescription(
        body_name="robocasa_mug",
        category="mug",
        semantic_annotation_type=Mug,
        mass=0.3,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi / 2, height=0.9, depth=0.005, pitch=0.0, roll=0.0
        ),
        handle_finder=ProtrudingHandleFinder(),
    )
    """
    A mug of the RoboCasa dataset, 11 cm across and 6 cm high, its handle towards the
    robot; the default grasp pinches the rim on the mug's side.
    """

    ROBOCASA_CUP = RoboCasaObjectDescription(
        body_name="robocasa_cup",
        category="cup",
        semantic_annotation_type=Cup,
        mass=0.15,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.9, depth=0.001, pitch=0.0, roll=0.0
        ),
    )
    """
    A cup of the RoboCasa dataset.
    """

    ROBOCASA_CAN = RoboCasaObjectDescription(
        body_name="robocasa_can",
        category="can",
        semantic_annotation_type=Container,
        mass=0.35,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.8, depth=0.034, pitch=0.0, roll=0.0
        ),
    )
    """
    A can of the RoboCasa dataset.
    """

    ROBOCASA_WATER_BOTTLE = RoboCasaObjectDescription(
        body_name="robocasa_bottled_water",
        category="bottled_water",
        semantic_annotation_type=Bottle,
        mass=0.3,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.8, depth=0.015, pitch=0.0, roll=0.0
        ),
    )
    """
    A bottle of water of the RoboCasa dataset.
    """

    ROBOCASA_JUG = RoboCasaObjectDescription(
        body_name="robocasa_jug",
        category="jug",
        semantic_annotation_type=Container,
        mass=0.4,
        default_grasp=SurfaceGrasp(
            azimuth=math.pi, height=0.92, depth=0.014, pitch=0.0, roll=0.0
        ),
    )
    """
    A jug of the RoboCasa dataset.
    """


# %% choosing an object on the command line


@dataclass
class ObjectChoice:
    """
    Lets a command line choose the object to pick up: one of :class:`PickUpObject`, and
    for a RoboCasa object optionally another model of its category.
    """

    parser: argparse.ArgumentParser
    """
    The command line's parser.
    """

    def add_arguments(self) -> None:
        """
        Add the arguments choosing the object to :attr:`parser`.
        """
        self.parser.add_argument(
            "--object",
            choices=[pick_up_object.name.lower() for pick_up_object in PickUpObject],
            default=PickUpObject.BOWL.name.lower(),
            help="the object to pick up",
        )
        self.parser.add_argument(
            "--instance",
            type=int,
            help="for a RoboCasa object, which model of its category to take",
        )

    def description(self, arguments: argparse.Namespace) -> ObjectDescription:
        """
        :param arguments: The parsed command line.
        :return: The description of the chosen object. Choosing a model of an object
            that is not a RoboCasa object ends the program with a usage error.
        """
        description = PickUpObject[arguments.object.upper()].value
        if arguments.instance is None:
            return description
        if not isinstance(description, RoboCasaObjectDescription):
            self.parser.error("--instance needs a RoboCasa object")
        return description.instance(arguments.instance)
