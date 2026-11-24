"""
Try closing RoboCasa's oven door with the PandaOmron from stands, grasps and base
motions drawn uniformly, in worker processes, and keep every trial in a database.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from experiments.robocasa_atomic_tasks.close_oven_door_trials import (
    CloseOvenDoorSetup,
    CloseOvenDoorSpace,
)
from krrood.parametrization.model_registries import UniformPriorRegistry
from krrood.parametrization.trials.campaigns import TrialCampaign
from krrood.parametrization.trials.parameter_spaces import DrawnParameters
from krrood_compute_resources.workers import WorkerPool

# %% running the campaign


def main() -> None:
    """
    Run the campaign as the command line asks.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True, help="SQLite file")
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--gigabytes-per-worker", type=float, default=10.0)
    parser.add_argument("--draw-seed", type=int, default=0)
    arguments = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(process)d %(name)s: %(message)s"
    )
    arguments.database.parent.mkdir(parents=True, exist_ok=True)
    campaign = TrialCampaign(
        setup=CloseOvenDoorSetup(database_url=f"sqlite:///{arguments.database}"),
        parameter_source=DrawnParameters(
            parameter_space=CloseOvenDoorSpace(),
            model_registry=UniformPriorRegistry(),
            number_of_draws=arguments.trials,
            draw_seed=arguments.draw_seed,
        ),
        worker_pool=WorkerPool(
            number_of_workers=arguments.workers,
            memory_per_worker=int(arguments.gigabytes_per_worker * 1024**3),
        ),
    )
    kept = campaign.run()
    logging.info("Kept %d trials in %s", kept, arguments.database)


if __name__ == "__main__":
    main()
