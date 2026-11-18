"""
ORMatic-specific check that a field typed with a mixin can store an object whose class
inherits the mixin next to the base it is stored as.

A table inherits from one parent table only, the class's primary base, so the table of
the mixin is no ancestor of the object's table: the field has to point at the nearest
table every class with the mixin shares.
"""

from __future__ import annotations

import importlib.util
import sys
from types import ModuleType

import pytest
from sqlalchemy import inspect, select
from sqlalchemy.orm import configure_mappers, sessionmaker

from krrood.class_diagrams.class_diagram import ClassDiagram
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.ormatic.helper import (
    OrmaticInterfaceInformation,
    get_classes_of_ormatic_interface,
)
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.utils import create_engine
from ..dataset.mixin_typed_field_classes import (
    HandTool,
    HandleInUse,
    Hammer,
    HasHandle,
    HasPairOfJaws,
    JawsInUse,
    Pliers,
    Tongs,
)

# %% a mixin every subclass inherits next to its primary base


def generated_module(*, instance: ORMatic, directory, name: str) -> ModuleType:
    """
    :param instance: The ORMatic whose tables are made.
    :param directory: Where the interface is written.
    :param name: The name the interface is imported under.
    :return: The interface, imported.
    """
    instance.make_all_tables()
    interface_file = directory / f"{name}.py"
    with open(interface_file, "w") as f:
        instance.to_sqlalchemy_file(file=f)
    spec = importlib.util.spec_from_file_location(name, interface_file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def tools_module(tmp_path_factory) -> ModuleType:
    """
    An interface mapping the tools with jaws, generated and imported once.
    """
    return generated_module(
        instance=ORMatic(
            class_dependency_graph=ClassDiagram(
                classes=[HasPairOfJaws, HandTool, Pliers, Tongs]
            )
        ),
        directory=tmp_path_factory.mktemp("mixin_typed_field_tools"),
        name="mixin_typed_field_tools_interface",
    )


@pytest.fixture(scope="module")
def module(tmp_path_factory, tools_module) -> ModuleType:
    """
    An interface built on :func:`tools_module` that maps only the class referring to the
    tools by their mixin, as a package that uses another package's classes does.
    """
    tools = get_classes_of_ormatic_interface(tools_module)
    generated = generated_module(
        instance=ORMatic(
            class_dependency_graph=ClassDiagram(classes=[HasPairOfJaws, JawsInUse]),
            interface_information=OrmaticInterfaceInformation(
                type_mappings=tools.type_mappings,
                alternative_mappings=tools.alternative_mappings,
                externally_mapped_classes=tools.externally_mapped_classes,
            ),
        ),
        directory=tmp_path_factory.mktemp("mixin_typed_field"),
        name="mixin_typed_field_interface",
    )
    get_classes_of_ormatic_interface(generated)
    configure_mappers()
    return generated


def test_a_mixin_typed_field_points_at_the_table_its_subclasses_share(
    module, tools_module
):
    """
    The field refers to the table of the primary base every class with the mixin has,
    although that table is mapped by the interface the field's interface builds on.
    """
    relationships = inspect(module.JawsInUseDAO).relationships

    assert relationships["closing"].mapper.class_ is tools_module.HandToolDAO


def test_a_mixin_typed_field_of_the_same_interface_points_at_the_shared_table():
    """
    The field refers to the shared table also where the tools are mapped by the same
    interface as the field.
    """
    instance = ORMatic(
        class_dependency_graph=ClassDiagram(
            classes=[HasPairOfJaws, HandTool, Pliers, Tongs, JawsInUse]
        )
    )
    instance.make_all_tables()
    table = instance.wrapped_tables[
        instance.class_dependency_graph.get_wrapped_class(clazz=JawsInUse)
    ]

    assert {relationship.type for relationship in table.relationships} >= {
        "Mapped[HandToolDAO]"
    }


def test_an_object_in_a_mixin_typed_field_round_trips(module):
    """
    Objects of two classes with the mixin are stored in the fields typed with it and
    read back as what they were.
    """
    engine = create_engine("sqlite:///:memory:")
    module.Base.metadata.create_all(engine)
    session = sessionmaker(engine)()
    original = JawsInUse(
        closing=Pliers(tool_name="pliers", jaw_gap=0.02),
        waiting=[Tongs(tool_name="tongs", jaw_gap=0.1)],
    )

    session.add(to_dao(original))
    session.commit()
    session.expunge_all()
    restored = session.scalars(select(module.JawsInUseDAO)).one().from_dao()

    assert restored == original


# %% a mixin that can be made itself


def test_a_mixin_that_can_be_made_itself_keeps_its_own_table():
    """
    An object of the mixin itself is stored in its own table, which the tables of its
    subclasses do not descend from, so the field keeps pointing at that table.
    """
    instance = ORMatic(
        class_dependency_graph=ClassDiagram(
            classes=[HasHandle, HandTool, Hammer, HandleInUse]
        )
    )
    instance.make_all_tables()
    [held] = instance.wrapped_tables[
        instance.class_dependency_graph.get_wrapped_class(clazz=HandleInUse)
    ].relationships

    assert held.type == "Mapped[HasHandleDAO]"
