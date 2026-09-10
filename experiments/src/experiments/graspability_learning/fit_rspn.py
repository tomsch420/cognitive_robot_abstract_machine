"""
Fit a RelationalProbabilisticCircuit over the multi-geometry grasp dataset persisted
by generate_dataset.py (Cube, milk carton, cup).
"""
import sys
sys.path.insert(0, "experiments/src")

from sqlalchemy import select
from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine

import experiments.orm.ormatic_interface as orm_interface
from experiments.orm.ormatic_interface import GraspTrialResultDAO
from experiments.graspability_learning.domain_model import GraspTrialResult
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)

DB_PATH = "experiments/src/experiments/graspability_learning/resources/grasp_dataset_multi.sqlite"


def main():
    engine = create_engine(f"sqlite:///{DB_PATH}")
    session = sessionmaker(engine)()
    instances = list(session.scalars(select(GraspTrialResultDAO)).all())
    print(f"Loaded {len(instances)} GraspTrialResultDAO instances")

    model = RelationalProbabilisticCircuit(GraspTrialResult)
    model.fit(instances)

    circuit = model.class_probabilistic_circuit
    print("Fitted circuit is_valid:", circuit.is_valid())
    print("Variables:")
    for v in circuit.variables:
        print(" -", v.name)


if __name__ == "__main__":
    main()
