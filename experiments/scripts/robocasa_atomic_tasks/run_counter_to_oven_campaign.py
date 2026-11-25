"""
Try moving the object of RoboCasa's PickPlaceCounterToOven onto the oven tray with the
PandaOmron, with how close it comes to the kitchen and where it carries the object drawn
uniformly, in worker processes, and keep every trial in a database.

The same draw seed draws the same parameters for every RoboCasa seed, so campaigns over
several seeds try each parameters in each of their episodes.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from experiments.robocasa_atomic_tasks.counter_to_oven_trials import (
    CounterToOvenSetup,
    CounterToOvenSpace,
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
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--gigabytes-per-worker", type=float, default=6.0)
    parser.add_argument("--draw-seed", type=int, default=0)
    parser.add_argument(
        "--seed", type=int, default=0, help="seed RoboCasa samples the episode with"
    )
    arguments = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(process)d %(name)s: %(message)s"
    )
    arguments.database.parent.mkdir(parents=True, exist_ok=True)
    campaign = TrialCampaign(
        setup=CounterToOvenSetup(
            database_url=f"sqlite:///{arguments.database}", seed=arguments.seed
        ),
        parameter_source=DrawnParameters(
            parameter_space=CounterToOvenSpace(),
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
