"""
Learning a model of where grasps lift an object, from the attempts of one task, and
handing it to the object's annotation.

The model is a relational probabilistic circuit over
:class:`~semantic_digital_twin.grasping.surface_grasp.SurfaceGrasp` and its result,
fitted with a joint probability tree. Asked through the statement of where the
annotation may be grasped, with lifting required, it answers with grasps it expects to
lift the object.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

from typing_extensions import List, Optional

from experiments.grasp_learning.records import GraspAttempt, GraspLearningTask
from krrood.parametrization.model_registries import RelationalCircuitRegistry
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% the model


@dataclass
class GraspModel:
    """
    A model of where grasps lift an object, learned for one task.
    """

    annotation_type: str
    """
    See :attr:`~experiments.grasp_learning.records.GraspLearningTask.annotation_type`.
    """

    grasped_part: str
    """
    See :attr:`~experiments.grasp_learning.records.GraspLearningTask.grasped_part`.
    """

    gripper: str
    """
    See :attr:`~experiments.grasp_learning.records.GraspLearningTask.gripper`.
    """

    circuit: str
    """
    The fitted circuit over the grasp and its result, as JSON.
    """

    number_of_attempts: int
    """
    How many attempts the model was learned from.
    """

    lift_rate_of_attempts: float
    """
    The share of those attempts that lifted the object.
    """

    verified_lift_rate: Optional[float] = None
    """
    The share of attempts with grasps drawn from this model that lifted the object;
    ``None`` until the model is verified.
    """

    @property
    def task(self) -> GraspLearningTask:
        """
        :return: The task this model was learned for.
        """
        return GraspLearningTask(
            annotation_type=self.annotation_type,
            grasped_part=self.grasped_part,
            gripper=self.gripper,
        )

    def model_registry(self) -> RelationalCircuitRegistry:
        """
        :return: A model registry that answers statements about surface grasps with this
            model.
        """
        return RelationalCircuitRegistry(
            RelationalProbabilisticCircuit(
                SurfaceGrasp,
                class_probabilistic_circuit=ProbabilisticCircuit.from_json(
                    json.loads(self.circuit)
                ),
            )
        )


# %% learning


@dataclass
class GraspModelLearner:
    """
    Learns a grasp model from the attempts of one task.
    """

    minimum_attempts_per_leaf: int = 10
    """
    The fewest attempts a leaf of the joint probability tree describes, which keeps the
    model from fitting single attempts.
    """

    def learn(
        self, task: GraspLearningTask, attempts: List[GraspAttempt]
    ) -> GraspModel:
        """
        :param task: The task the attempts belong to.
        :param attempts: Attempts of that task, each grasp with its result.
        :return: The model learned from them, not yet verified.
        """
        grasps = [attempt.grasp for attempt in attempts]
        relational_circuit = RelationalProbabilisticCircuit(
            SurfaceGrasp,
            learning_method=JointProbabilityTree(
                min_samples_per_leaf=self.minimum_attempts_per_leaf
            ),
        ).fit(grasps)
        return GraspModel(
            annotation_type=task.annotation_type,
            grasped_part=task.grasped_part,
            gripper=task.gripper,
            circuit=json.dumps(
                relational_circuit.class_probabilistic_circuit.to_json()
            ),
            number_of_attempts=len(attempts),
            lift_rate_of_attempts=sum(grasp.result.lifted for grasp in grasps)
            / len(grasps),
        )


# %% handing models to annotations


@dataclass
class GraspModelLibrary:
    """
    The grasp models learned so far, handed to the annotations of the objects they were
    learned for.
    """

    models: List[GraspModel] = field(default_factory=list)
    """
    The models, of any tasks.
    """

    def model_for(
        self, graspable: HasGraspCandidates, gripper: str
    ) -> Optional[GraspModel]:
        """
        :param graspable: The object to grasp.
        :param gripper: The name of the type of the gripper that grasps.
        :return: The latest model learned for grasping the object's grasped part with
            that gripper; ``None`` if there is none.
        """
        task = GraspLearningTask(
            annotation_type=type(graspable).__name__,
            grasped_part=type(graspable.grasped_part()).__name__,
            gripper=gripper,
        )
        matching = [model for model in self.models if model.task == task]
        if not matching:
            return None
        return matching[-1]

    def grasp_candidates(
        self, graspable: HasGraspCandidates, gripper: str
    ) -> List[GraspCandidate]:
        """
        :param graspable: The object to grasp.
        :param gripper: The name of the type of the gripper that grasps.
        :return: Grasps the learned model expects to lift the object; without a model,
            the grasps the annotation offers by itself.
        """
        model = self.model_for(graspable, gripper)
        if model is None:
            return graspable.grasp_candidates()
        return graspable.grasp_candidates(model.model_registry(), require_lifting=True)
