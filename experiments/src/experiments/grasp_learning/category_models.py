"""
The grasp models of RoboCasa object categories, each learned from the attempts on its
own objects.

Categories without an annotation of their own share their task with others, such as
every fruit, so a category's model is learned from its own objects' attempts rather than
from every attempt of its task.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Dict

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.models import GraspModel, GraspModelLearner
from experiments.physical_pick_up.objects import GraspableRoboCasaObject

# %% models per category


@dataclass
class CategoryGraspModels:
    """
    Learns the grasp model of a RoboCasa object category on first request, from the
    attempts stored on the category's objects.
    """

    database: GraspDatabase
    """
    The database holding the attempts.
    """

    learner: GraspModelLearner = field(default_factory=GraspModelLearner)
    """
    Learns a model from a category's attempts.
    """

    learned: Dict[str, GraspModel | None] = field(default_factory=dict, init=False)
    """
    The models learned so far by category; ``None`` for a category without a model.
    """

    def model_for(self, category: str) -> GraspModel | None:
        """
        :param category: The name RoboCasa gives an object category.
        :return: The model learned from the attempts on the category's objects; ``None``
            if the category is not graspable or no attempt was stored on its objects.
        """
        if category not in self.learned:
            self.learned[category] = self._learn(category)
        return self.learned[category]

    def _learn(self, category: str) -> GraspModel | None:
        """
        :param category: The name RoboCasa gives an object category.
        :return: The model of the category, see :meth:`model_for`.
        """
        graspable = GraspableRoboCasaObject.__members__.get(category.upper())
        if graspable is None:
            return None
        attempts = self.database.attempts_on_category(graspable.value)
        if not attempts:
            return None
        return self.learner.learn(attempts[0].task, attempts)
