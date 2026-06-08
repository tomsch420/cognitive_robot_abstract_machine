import datetime
import logging
import random
import os
import time

from dataclasses import is_dataclass, dataclass
from typing import Type, List

import numpy as np
import tempfile
import shutil
import experiments
import experiments.ormatic_interface
import pycram.orm.ormatic_interface
from krrood.class_diagrams import ClassDiagram
from krrood.ormatic.data_access_objects.alternative_mappings import AlternativeMapping
from krrood.ormatic.helper import get_classes_of_ormatic_interface
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.type_dict import TypeDict
from krrood.ormatic.utils import classes_of_package
from krrood.utils import recursive_subclasses
from pycram.orm.model import NumpyType

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the pycram package
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------


# import classes from the existing interface
classes, alternative_mappings, type_mappings = get_classes_of_ormatic_interface(
    experiments.ormatic_interface
)
classes = set(classes)

alternative_mappings += [am for am in recursive_subclasses(AlternativeMapping)]
alternative_mappings = list(set(alternative_mappings))
# keep only dataclasses that are NOT AlternativeMapping subclasses
classes = {
    c for c in classes if is_dataclass(c) and not issubclass(c, AlternativeMapping)
}
classes |= {am.original_class() for am in recursive_subclasses(AlternativeMapping)}

alternative_mappings = [
    am
    for am in recursive_subclasses(AlternativeMapping)
    if am.original_class() in classes
]


@dataclass
class ScalabilityExperiment:
    total_duration: datetime.timedelta
    class_diagram_creation_duration: datetime.timedelta
    ormatic_reasoning_duration: datetime.timedelta
    writing_to_file_duration: datetime.timedelta
    number_of_classes: int
    number_of_associations: int
    number_of_inheritances: int


def perform_experiment(
    classes: List[Type], class_drop_probability: float = 0.3
) -> ScalabilityExperiment:

    begin = datetime.datetime.now()

    filtered_classes = [
        c for c in classes if random.uniform(0, 1) > class_drop_probability
    ]
    # create the new ormatic interface
    class_diagram = ClassDiagram(
        list(sorted(filtered_classes, key=lambda c: c.__name__, reverse=True))
    )

    class_diagram_creation_time = datetime.datetime.now()

    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(
        class_diagram,
        type_mappings=TypeDict(type_mappings),
        alternative_mappings=alternative_mappings,
    )

    # Generate the ORM classes
    ormatic.make_all_tables()

    ormatic_reasoning_time = datetime.datetime.now()

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".py") as f:
        temp_path = f.name
        ormatic.to_sqlalchemy_file(f)

    writing_to_file_time = datetime.datetime.now()

    return ScalabilityExperiment(
        total_duration=writing_to_file_time - begin,
        class_diagram_creation_duration=class_diagram_creation_time - begin,
        ormatic_reasoning_duration=ormatic_reasoning_time - class_diagram_creation_time,
        writing_to_file_duration=writing_to_file_time - ormatic_reasoning_time,
        number_of_classes=len(filtered_classes),
        number_of_associations=len(class_diagram.associations),
        number_of_inheritances=len(class_diagram.inheritance_relations),
    )


def main():
    experiment = perform_experiment(classes, 0.3)
    print(experiment)


if __name__ == "__main__":
    main()
