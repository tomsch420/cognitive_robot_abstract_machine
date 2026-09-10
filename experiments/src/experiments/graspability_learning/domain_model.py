"""
Domain model for grasp-candidate generation and physics-verified grasp trials.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar, Iterator

from semantic_digital_twin.api import BodySpecification, Connection6DoFSpecification
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody, IsPerceivable
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk, Cup
from semantic_digital_twin.robots.robot_parts import EndEffector
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.spatial_types.spatial_types import Pose

CUBE_MESH_FILENAME = "cube.obj"
MILK_MESH_FILENAME = "milk.stl"
CUP_MESH_FILENAME = "jeroen_cup.stl"


class GraspableKind(Enum):
    """
    Which concrete geometry a GraspCandidate/GraspTrialResult was generated for.

    A real Enum, not a plain str: krrood's FeatureExtractor only treats
    `(int, float, bool, Enum)` (random_events.variable.compatible_types) as
    compatible scalar column types for a JointProbabilityTree -- a str field would be
    silently skipped by _process_attributes, never reaching the RSPN at all. Enum is
    the same mechanism EQL's EntityQueryLanguageGenerativeBackend already relies on
    for categorical sampling, so this reuses an established convention rather than
    introducing a new one.
    """

    CUBE = "cube"
    MILK = "milk"
    CUP = "cup"


@dataclass(eq=False)
class IsGraspable(HasRootBody, ABC):
    """
    Mixin for semantic annotations that can generate candidate grasps for themselves.
    """

    kind: ClassVar[GraspableKind]

    # Whether the grasp orientation should be the *level* (de-tilted) top-down flip
    # rather than the cube's raw, deliberately tilted one. False (raw) is correct for
    # the cube specifically, because the cube itself is placed pitched -90 degrees
    # (lying on its side); an upright object needs the level orientation instead --
    # see stage1_fixed_sample.default_grasp_pose's `level` parameter for the full
    # empirical justification (confirmed both ways: removing the tilt regresses the
    # cube's grasp, and keeping it starves milk/cup of aperture margin they can't spare).
    level_grasp: ClassVar[bool] = False

    @abstractmethod
    def grasp_height(self) -> float:
        """
        :return: World-frame Z of this object's grasp point (its own vertical
            center), given where stage1_fixed_sample.build_world placed it. Every
            IsGraspable object is placed so its base touches the table (world Z=0),
            so this is always just half of the object's own vertical extent -- see
            each subclass's implementation.
        """

    def _default_approach_pose(self) -> Pose:
        """
        A real SE(3) Pose of the gripper ("hand") relative to "link0", solved by
        Giskard at this object's own grasp height -- see
        stage1_fixed_sample.default_grasp_pose. A Pose's reference_frame is a
        specific Body of one World instance, so it must be derived from -- and is
        only usable with -- the same world this object lives in.
        """
        from experiments.graspability_learning.stage1_fixed_sample import default_grasp_pose

        return default_grasp_pose(
            self.root._world, grasp_height=self.grasp_height(), level=type(self).level_grasp
        )

    def grasp_candidates(
        self, end_effector: EndEffector, amount: int = 1
    ) -> Iterator["GraspCandidate"]:
        """
        Yield `amount` candidates with aperture/closing_effort drawn independently and
        uniformly at random. This sampling logic is identical for every IsGraspable
        type -- aperture/closing_effort are limits of the Panda gripper itself, not of
        the object being grasped -- so it lives here once instead of being copied per
        subclass; what actually varies per type is `grasp_height`/`kind`, both
        supplied by the concrete subclass.

        Deliberately plain `random.uniform`, not krrood's EQL `ProbabilisticBackend`
        (which an earlier version of this method used, built from
        `a(GraspCandidate)(aperture=..., closing_effort=...).where(...)`): verified
        directly (by sampling and printing raw (aperture, closing_effort) pairs with
        no physics involved) that its `FullyFactorizedRegistry` does NOT sample the
        two continuous variables independently -- every one of the 15 highest-aperture
        samples out of 170 also had closing_effort within 5.0-5.6, the very bottom of
        its intended [5, 60] range, instead of spread across it. That silently
        eliminated the entire (high aperture, high effort) region of the design space
        this project's whole premise depends on being independently explorable (heavy
        objects like the milk carton need both to succeed at all -- confirmed
        separately with true independent sampling, aperture=0.0394/effort=25.97
        succeeds where the same aperture at effort ~5 does not), which is what
        actually caused milk's real-physics success rate to come out as exactly 0 over
        1700 EQL-sampled trials. Root cause traced to `fully_factorized()`
        (krrood/parametrization/model_registries.py) modeling every continuous
        variable as `GaussianDistribution(location=0.0, scale=1.0)` regardless of the
        query's `where()` bounds; not chased further into how that then gets mapped
        through the bounds and correlated across variables -- a bug worth reporting
        upstream, but not this project's to fix. EQL/`RelationalCircuitRegistry` is
        still used, correctly, by generate_from_learned_circuit.py: that backend
        grounds an already-fitted circuit instead of `fully_factorized`, and isn't
        affected by this.
        """
        import random

        approach_pose = self._default_approach_pose()
        for _ in range(amount):
            yield GraspCandidate(
                graspable=self,
                aperture=random.uniform(0.0, 0.04),
                closing_effort=random.uniform(5.0, 60.0),
                end_effector=end_effector,
                approach_pose=approach_pose,
                graspable_kind=type(self).kind,
            )


@dataclass(eq=False)
class Cube(IsGraspable, IsPerceivable):
    """
    A single mesh-backed cube.
    """

    kind: ClassVar[GraspableKind] = GraspableKind.CUBE

    @classmethod
    def parent_connection_specification(cls) -> Connection6DoFSpecification:
        return Connection6DoFSpecification()

    @classmethod
    def get_default_root_kinematic_structure_entity_specification(
        cls,
        name: str | None = None,
        scale: Scale | None = None,
        connection_specification=None,
    ) -> BodySpecification:
        import os

        mesh_path = os.path.join(os.path.dirname(__file__), "resources", CUBE_MESH_FILENAME)
        return BodySpecification.mesh(
            name,
            filename=mesh_path,
            scale=scale or Scale(1.0, 1.0, 1.0),
            connection_specification=connection_specification,
        )

    def grasp_height(self) -> float:
        # The cube mesh is 0.04 x 0.04 x 0.06; stage1_fixed_sample.build_world spawns
        # it lying on its side (its 0.04 dimension vertical), centered on its own
        # origin, base touching the table -- so its center (the grasp point) sits at
        # exactly half that: 0.02. See build_world's own placement for the full
        # reasoning (stability, not just grasp height).
        return 0.02


@dataclass(eq=False)
class GraspableMilk(IsGraspable, Milk):
    """
    A real milk-carton mesh (semantic_digital_twin's own `Milk` semantic annotation,
    mixed with grasp-candidate generation) -- one of the two IAI-kitchen-scene
    objects that are both small enough for the Panda's parallel gripper and shipped
    with a real mesh asset in this codebase (robokudo's `world_iai_kitchen20`
    descriptor places this same milk.stl in the IAI kitchen scene; see
    stage1_fixed_sample.py for the measured footprint/height that make it graspable
    at all -- its ~6.5cm footprint leaves only a few mm of clearance against the
    gripper's 8cm max opening, unlike the cube's comfortably smaller 4cm).
    """

    kind: ClassVar[GraspableKind] = GraspableKind.MILK
    level_grasp: ClassVar[bool] = True

    @classmethod
    def parent_connection_specification(cls) -> Connection6DoFSpecification:
        return Connection6DoFSpecification()

    @classmethod
    def get_default_root_kinematic_structure_entity_specification(
        cls,
        name: str | None = None,
        scale: Scale | None = None,
        connection_specification=None,
    ) -> BodySpecification:
        import os

        mesh_path = os.path.join(os.path.dirname(__file__), "resources", MILK_MESH_FILENAME)
        return BodySpecification.mesh(
            name,
            filename=mesh_path,
            scale=scale or Scale(1.0, 1.0, 1.0),
            connection_specification=connection_specification,
        )

    def grasp_height(self) -> float:
        from experiments.graspability_learning.stage1_fixed_sample import MILK_GRASP_HEIGHT

        return MILK_GRASP_HEIGHT


@dataclass(eq=False)
class GraspableCup(IsGraspable, Cup):
    """
    A real cup mesh (`jeroen_cup.stl`, bundled with semantic_digital_twin) mixed with
    grasp-candidate generation -- the other real-mesh IAI-kitchen-sized object small
    enough for this gripper. Its ~7cm footprint is the tightest of the three: only
    0.5cm of clearance remains against the gripper's 8cm max opening, so most sampled
    apertures below the very top of the allowed range are expected to fail here.
    """

    kind: ClassVar[GraspableKind] = GraspableKind.CUP
    level_grasp: ClassVar[bool] = True

    @classmethod
    def parent_connection_specification(cls) -> Connection6DoFSpecification:
        return Connection6DoFSpecification()

    @classmethod
    def get_default_root_kinematic_structure_entity_specification(
        cls,
        name: str | None = None,
        scale: Scale | None = None,
        connection_specification=None,
    ) -> BodySpecification:
        import os

        mesh_path = os.path.join(os.path.dirname(__file__), "resources", CUP_MESH_FILENAME)
        return BodySpecification.mesh(
            name,
            filename=mesh_path,
            scale=scale or Scale(1.0, 1.0, 1.0),
            connection_specification=connection_specification,
        )

    def grasp_height(self) -> float:
        from experiments.graspability_learning.stage1_fixed_sample import CUP_GRASP_HEIGHT

        return CUP_GRASP_HEIGHT


@dataclass
class GraspCandidate:
    """
    One sampled, unexecuted grasp attempt -- the input to a physics trial.
    """

    graspable: IsGraspable
    aperture: float
    closing_effort: float
    # Defaulted (and declared last, as Python's dataclass rules require after
    # non-defaulted fields) so an EQL Match can construct a GraspCandidate from just
    # graspable/aperture/closing_effort -- see IsGraspable.grasp_candidates for why
    # end_effector/approach_pose/graspable_kind are excluded from that query and
    # patched on after.
    end_effector: EndEffector = None
    approach_pose: Pose = None
    graspable_kind: GraspableKind = GraspableKind.CUBE


@dataclass
class GraspTrialResult:
    """
    Measured outcome of executing a GraspCandidate in MuJoCo.
    """

    candidate: GraspCandidate
    success: bool
    # Defaulted (and declared last, as Python's dataclass rules require after
    # non-defaulted fields) for the same reason as GraspCandidate's end_effector/
    # approach_pose: RelationalProbabilisticCircuit.ground() unconditionally calls
    # query.construct_instance() on a query that only specifies candidate/success (used
    # for its own exchangeable-parts handling, irrelevant here since this schema has
    # none), so this class must be constructible from just those two.
    max_translation_slip: float = None
    max_rotation_slip: float = None
    contact_count: int = None
    held_duration: float = None
