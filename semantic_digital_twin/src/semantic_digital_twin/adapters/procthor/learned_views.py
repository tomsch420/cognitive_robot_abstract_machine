from dataclasses import dataclass, field
from typing import List, Self

from sqlalchemy.orm import Session

from semantic_digital_twin.orm.ormatic_interface import (
    InsideOfDAO,
    RootedSemanticAnnotationDAO,
)


@dataclass
class AnnotatedInsideOfView:
    """
    An enrichment to the InsideOfDAO that also contains the semantic annotations of the two sides of the relation.
    This is aggregated from a database.
    """

    inside_of_dao: InsideOfDAO
    """
    The raw inside of relation.
    """

    body_side_semantic_annotation: List[RootedSemanticAnnotationDAO] = field(
        default_factory=list
    )
    """
    The annotations that involve the `self.inside_of_dao.body` side of the relation.
    """

    other_side_semantic_annotation: List[RootedSemanticAnnotationDAO] = field(
        default_factory=list
    )
    """
    The annotations that involve the `self.inside_of_dao.other` side of the relation.
    """

    @classmethod
    def from_database(cls, session: Session) -> List[Self]:
        """
        :param session: The SQLAlchemy session to query the database
        :return: a list of AnnotatedInsideOfDAO objects from the database.
        """
