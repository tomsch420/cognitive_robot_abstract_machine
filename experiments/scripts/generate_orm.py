import logging
from pathlib import Path

import experiments
import experiments.control_loop_experiments.benchmark
import experiments.control_loop_experiments.scenarios
import coraplex.orm.ormatic_interface

from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.utils import classes_of_module
import experiments.control_loop_experiments.control_loop_profiler
from experiments.grasp_learning.category_models import CategoryGraspModels
from experiments.grasp_learning.database import GraspDatabase
from experiments.robocasa_atomic_tasks.close_oven_door_trials import (
    CloseOvenDoorPerformer,
    CloseOvenDoorSetup,
    CloseOvenDoorSpace,
    CloseOvenDoorTrialsInADatabase,
)
from experiments.robocasa_atomic_tasks.open_oven_door_trials import (
    OpenOvenDoorPerformer,
    OpenOvenDoorSetup,
    OpenOvenDoorSpace,
    OpenOvenDoorTrialsInADatabase,
)
from experiments.robocasa_atomic_tasks.oven_door_trials import (
    OvenDoorScene,
    OvenDoorSetup,
    StandOnTheFloor,
)

# benchmarking measures a running system instead of describing it
ignored_classes = set(classes_of_module(experiments.control_loop_experiments.scenarios))
ignored_classes |= set(
    classes_of_module(experiments.control_loop_experiments.benchmark)
)
ignored_classes |= set(
    classes_of_module(experiments.control_loop_experiments.control_loop_profiler)
)
# services that read and keep records rather than records themselves; mapping them made
# the interface import the modules that import it
ignored_classes |= {
    GraspDatabase,
    CategoryGraspModels,
    CloseOvenDoorPerformer,
    CloseOvenDoorSetup,
    CloseOvenDoorSpace,
    CloseOvenDoorTrialsInADatabase,
    OpenOvenDoorPerformer,
    OpenOvenDoorSetup,
    OpenOvenDoorSpace,
    OpenOvenDoorTrialsInADatabase,
    OvenDoorScene,
    OvenDoorSetup,
    StandOnTheFloor,
}

# Create an ORMatic object with the classes to be mapped
ormatic = ORMatic.from_package(
    packages=[experiments],
    ormatic_interface_dependencies=[coraplex.orm.ormatic_interface],
    ignored_classes=ignored_classes,
    type_mappings={},
)
logging.getLogger("krrood").setLevel(logging.DEBUG)

# Generate the ORM classes
ormatic.make_all_tables()

ormatic_interface_path = (
    Path(__file__).parent.parent
    / "src"
    / "experiments"
    / "orm"
    / "ormatic_interface.py"
)
with open(ormatic_interface_path, "w") as f:
    ormatic.to_sqlalchemy_file(file=f)
