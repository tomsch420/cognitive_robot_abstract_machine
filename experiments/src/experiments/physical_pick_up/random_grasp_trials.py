"""
The PR2 tries grasps on an object that the statement of where the object's annotation
may be grasped leaves to chance, and records what each one did.

Run it with ``python -m experiments.physical_pick_up.random_grasp_trials``; see
``--help``.

The records are what a model of grasps is learned from. Once one is, the same statement
asks it for grasps instead of chance, through a
:class:`~krrood.parametrization.model_registries.DictRegistry` mapping
:class:`~semantic_digital_twin.grasping.surface_grasp.SurfaceGrasp` to the model, and a
nested ``result=a(GraspResult)(lifted=True, ...)`` asks for grasps the model predicts
to lift the object.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from datetime import timedelta
from pathlib import Path

from typing_extensions import Optional

from experiments.physical_pick_up.objects import ObjectChoice
from experiments.physical_pick_up.pr2_pick_up import PR2PickUpExperiment
from experiments.physical_pick_up.scene import ObjectOnTableScene
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.grasp_trials import GraspTrier, GraspTrials
from semantic_digital_twin.grasping.surface_grasp import GraspResult

# %% the PR2 trying grasps


@dataclass
class PR2GraspTrier(GraspTrier):
    """
    Tries every grasp with the PR2 of an experiment, starting each attempt from the same
    state of the world.
    """

    experiment: PR2PickUpExperiment
    """
    The robot, the table and the object the grasps are tried with.
    """

    video_directory: Optional[Path] = None
    """
    Where to film each attempt; ``None`` films nothing.
    """

    _number_of_attempts: int = field(default=0, init=False)
    """
    How many grasps were tried so far.
    """

    def try_grasp(self, grasp: GraspCandidate) -> GraspResult:
        video_path = (
            None
            if self.video_directory is None
            else self.video_directory / f"trial_{self._number_of_attempts:03d}.mp4"
        )
        self._number_of_attempts += 1
        return self.experiment.run(grasp, video_path)


# %% the command line


def main() -> None:
    """
    Run the trials from the command line, printing each and writing them to a file of
    one JSON record per line.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    object_choice = ObjectChoice(parser)
    object_choice.add_arguments()
    parser.add_argument("--trials", type=int, default=GraspTrials.number_of_trials)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("grasp_trials.jsonl"),
        help="file to write the tried grasps to",
    )
    parser.add_argument("--videos", type=Path, help="directory to film each trial in")
    arguments = parser.parse_args()
    if arguments.videos is not None:
        arguments.videos.mkdir(parents=True, exist_ok=True)
    experiment = PR2PickUpExperiment(
        scene=ObjectOnTableScene(
            object_description=object_choice.description(arguments)
        ),
        time_limit=timedelta(seconds=20),
    )
    trials = GraspTrials(
        graspable=experiment.scene.graspable,
        trier=PR2GraspTrier(experiment=experiment, video_directory=arguments.videos),
        number_of_trials=arguments.trials,
    )
    lifted = 0
    number = 0
    with arguments.output.open("w") as records:
        for number, record in enumerate(trials.run(), start=1):
            grasp = record.grasp
            lifted += grasp.result.lifted
            records.write(json.dumps(asdict(record)) + "\n")
            records.flush()
            print(
                f"{number:3d} lifted={grasp.result.lifted!s:5} "
                f"rise={grasp.result.object_rise:+.3f} m "
                f"slip={grasp.result.translational_slip:.3f} m/"
                f"{grasp.result.rotational_slip:.2f} rad  "
                f"azimuth={grasp.azimuth:.2f} height={grasp.height:.2f} "
                f"depth={grasp.depth:.3f} pitch={grasp.pitch:.2f} roll={grasp.roll:.2f}",
                flush=True,
            )
    print(f"lifted the {arguments.object} in {lifted} of {number} trials")


if __name__ == "__main__":
    main()
