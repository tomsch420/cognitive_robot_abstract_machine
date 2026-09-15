"""
What it costs, in time and in hardware, to answer a long-term-memory question by
showing a vision-language model the episode's video instead of asking the SQL backend.

Paired with :mod:`experiments.paper.queries`, which already reports the SQL backend's
own per-question latency (:class:`~experiments.paper.queries.BackendLatency`) off
:class:`~experiments.episodes.episode.RecordedQuery`. This module measures the same
kind of cost for a backend that is not wired into that recording path at all: the VLM
never runs inside a trial, so there is no ``RecordedQuery`` for it to be read back from,
and nothing here writes one -- it measures a call made directly against the model.
"""

from __future__ import annotations

import base64
import time
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import cv2
import requests
from krrood.exceptions import DataclassException
from typing_extensions import List, Optional, Sequence

from experiments.episodes.episode import RecordedTrial
from experiments.experiment_definitions import ExperimentResult

# %% where the model measured here is served from

DEFAULT_OLLAMA_URL = "http://ollama:11434"
"""
Where the VLM this experiment measures is served from, resolved by the ``ollama``
container's own DNS alias.

Reachable from inside the cram container only because the two containers were joined
onto the ``open-webui_default`` network (``docker network connect open-webui_default
tom-cram-icra-experiments-simulation-pipeline``) -- the ollama container's other,
host-published address is not reliably reachable from another container's bridge
namespace (hairpin NAT gap), only from the docker host itself. A local ``nvidia-smi``
would not be reachable either way, which is why hardware use is read back from the
serving engine (:meth:`VLMQueryExperiment.resident_vram_bytes`) rather than queried
locally.
"""

DEFAULT_MODEL_NAME = "qwen3-vl:latest"
"""
The only vision-capable model this Ollama instance currently serves.
"""

VLM_FRAME_RATE_HZ = 1.0
"""
Frames per second pulled from the video, matching the rate episodes are already
recorded at, so the model is shown the same temporal resolution the run was filmed
with rather than a resampling of it.
"""

# %% how much of what CRAM itself knows the model is given alongside the video


class VLMContextLevel(StrEnum):
    """
    How much of CRAM's own knowledge of the scene the model is handed alongside the
    episode's frames, from none of it to all of it -- the independent variable the
    three variants of this experiment vary.
    """

    NONE = "none"
    """
    The video and the questions, nothing else -- what a model with no access to CRAM at
    all would be given.
    """

    URDF = "urdf"
    """
    The video, the questions, and the robot's URDF -- the kinematic structure (links,
    joints, their limits) CRAM itself loads the scene's embodiment from, but nothing
    about what actually happened during this particular episode.
    """

    FULL_WORLD_HISTORY = "full_world_history"
    """
    The video, the questions, the robot's URDF (as in :attr:`URDF`), and this episode's
    tick-by-tick segmind event log -- the same events the SQL backend answers its own
    questions from (:mod:`experiments.eql_experiments.query_benchmark`). The model is
    handed CRAM's own ground truth directly rather than being asked to reconstruct it
    from pixels, which is closer to an upper bound on what the video-only variants could
    ever achieve than a third way of answering the same question.
    """


@dataclass(frozen=True)
class VLMContext:
    """
    The extra text one context level hands the model alongside the episode's frames.

    A value rather than a free function so :attr:`VLMQueryExperimentResult.
    context_level` and its text are always constructed together and cannot drift apart
    -- a caller cannot pass :data:`VLMContextLevel.URDF` while forgetting to build the
    URDF text that level is supposed to carry.
    """

    level: VLMContextLevel
    text: str

    @classmethod
    def none(cls) -> VLMContext:
        """
        No extra context: the video and the questions alone.
        """
        return cls(level=VLMContextLevel.NONE, text="")

    @classmethod
    def urdf(cls, urdf_text: str) -> VLMContext:
        """
        The robot's URDF, verbatim, introduced so the model knows what it is reading.

        :param urdf_text: The URDF document, already expanded from any ``xacro`` macros
            it was authored with -- ``xacro <path> `` produces this.
        """
        return cls(
            level=VLMContextLevel.URDF,
            text=(
                "Here is the URDF description of the robot in the scene: its links, "
                "joints, and their limits. This describes the robot's embodiment only, "
                "not what happens in the video.\n\n%s"
            )
            % urdf_text,
        )

    @classmethod
    def full_world_history(cls, urdf_text: str, trials: Sequence[RecordedTrial]) -> VLMContext:
        """
        The robot's URDF plus this episode's trials' tick-by-tick segmind event log, as
        CRAM itself recorded them -- the same events :class:`~experiments.
        eql_experiments.query_benchmark.SQLQueryExperiment` reads its own answers from.

        Embodiment (the URDF) and history (the events) are combined here rather than
        history alone, since a model told only "what happened" with no idea what the
        robot even is would be missing exactly the fact :attr:`VLMContextLevel.URDF`
        supplies on its own -- this level is meant to be the most CRAM can hand the
        model, not history in isolation.

        :param urdf_text: The URDF document, already expanded from any ``xacro`` macros
            it was authored with -- ``xacro <path>`` produces this.
        :param trials: The episode's recorded trials, e.g. from
            :meth:`~experiments.episodes.long_term_memory.LongTermMemory.recall_trials`.
        """
        sections = [cls._render_trial(number, trial) for number, trial in enumerate(trials, start=1)]
        return cls(
            level=VLMContextLevel.FULL_WORLD_HISTORY,
            text=(
                "Here is the URDF description of the robot in the scene: its links, "
                "joints, and their limits.\n\n%s\n\n"
                "Here is CRAM's own recorded history of this episode: for each trial, "
                "every event its monitoring (segmind) detected, in order. This is "
                "ground truth, not an interpretation of the video.\n\n%s"
            )
            % (urdf_text, "\n\n".join(sections)),
        )

    @staticmethod
    def _render_trial(number: int, trial: RecordedTrial) -> str:
        """
        One trial's segmind event log, as the lines a reader (or a model) is shown.

        :param number: This trial's position among the episode's trials, for a label.
        :param trial: The trial to render.
        """
        lines = ["Trial %d (outcome=%s, duration=%.2fs):" % (
            number, trial.outcome.name, trial.duration
        )]
        if not trial.ticks:
            lines.append("  No ticks were recorded.")
        for tick in trial.ticks:
            if not tick.events:
                continue
            events = ", ".join(
                "%s(%s)" % (type(event).__name__, event.tracked_object.name.name)
                for event in tick.events
            )
            lines.append("  t=%.2fs: %s" % (tick.moment, events))
        return "\n".join(lines)


# %% what can go wrong asking it


@dataclass
class VLMContextExceeded(DataclassException):
    """
    Raised when an episode's video and questions do not fit the context requested for
    it, so the model refused the request rather than silently dropping frames.
    """

    episode_identifier: str
    """
    The episode whose video did not fit.
    """

    requested_context_tokens: int
    """
    The context window the request asked for.
    """

    server_message: str
    """
    What the serving engine said was too big, verbatim.
    """

    def error_message(self) -> str:
        return "Episode %s did not fit a %d-token context: %s" % (
            self.episode_identifier,
            self.requested_context_tokens,
            self.server_message,
        )

    def suggest_correction(self) -> str:
        return (
            "Raise context_tokens if the model and the GPU it runs on have room for "
            "it (see VLMQueryExperiment.resident_vram_bytes to check what raising it "
            "costs), or sample the video's frames more sparsely so fewer of them have "
            "to fit."
        )


@dataclass
class VLMVideoUnreadable(DataclassException):
    """
    Raised when an episode's video file could not be opened to sample frames from.
    """

    video_path: Path
    """
    The video that could not be read.
    """

    def error_message(self) -> str:
        return "Could not open %s to read its frames." % self.video_path

    def suggest_correction(self) -> str:
        return (
            "Check the path exists and is a video OpenCV's build was compiled to "
            "decode -- a container-specific codec gap, not a missing file, is the "
            "usual cause when the path is otherwise correct."
        )


@dataclass
class VLMNotResident(DataclassException):
    """
    Raised when the model this experiment just called is not the one the serving
    engine reports having loaded, so its VRAM use cannot be read back.
    """

    model_name: str
    """
    The model that was asked for.
    """

    def error_message(self) -> str:
        return "%s is not resident on the serving engine right after answering." % (
            self.model_name
        )

    def suggest_correction(self) -> str:
        return (
            "Another request may have evicted it between answering and this check; "
            "call resident_vram_bytes() immediately after run(), before anything else "
            "reaches the same engine."
        )


# %% one call's worth of measurements


@dataclass
class VLMQueryExperimentResult(ExperimentResult):
    """
    Time and hardware measurements from asking one vision-language model every
    long-term-memory question at once about one episode's video.

    One row per episode rather than per question: every question about an episode is
    asked in a single call (see :class:`VLMQueryExperiment.run`), because the fixed
    cost of loading the model and encoding the video's frames is paid once per call, not
    once per question, and folding six calls into one is what a fair comparison against
    the SQL backend's per-episode cost requires.
    """

    episode_identifier: str
    """
    The episode the video and the answers belong to.
    """

    model_name: str
    """
    The model that answered, named as Ollama names it.
    """

    context_level: str
    """
    How much of CRAM's own knowledge of the scene was handed to the model alongside the
    video, as :class:`VLMContextLevel` names it -- the independent variable this
    experiment's three variants vary; every other field is how much that costed.
    """

    context_text_bytes: int
    """
    The size of the extra context text handed to the model, zero for
    :attr:`VLMContextLevel.NONE`. The storage-cost analogue of
    :attr:`video_file_size_bytes` for whatever context was added on top of the video.
    """

    number_of_questions: int
    """
    How many questions were asked together in this call.
    """

    number_of_frames: int
    """
    How many frames of the video were shown to the model.
    """

    video_duration_seconds: float
    """
    How much of the episode the shown frames span, at :data:`VLM_FRAME_RATE_HZ`.
    """

    video_file_size_bytes: int
    """
    The recorded video's size on disk -- the storage cost of this representation,
    against which the SQL backend's few rows of ticks and events are the comparison.
    """

    requested_context_tokens: int
    """
    The context window asked for (``num_ctx``). The frames and questions have to fit
    inside it or the call is refused outright rather than truncated silently, so this
    is a real ceiling this run either did or did not meet.
    """

    prompt_token_count: int
    """
    How much of the requested context the frames and question text actually used.
    """

    answer_token_count: int
    """
    How much of the requested context the answer used.
    """

    wall_clock_duration: float
    """
    Seconds this measurement's own request/response round trip took, start to finish --
    the number a caller waiting on an answer actually experiences.
    """

    model_load_duration: float
    """
    Seconds Ollama reports spent loading the model; zero when it was already resident
    from a previous call.
    """

    prompt_processing_duration: float
    """
    Seconds Ollama reports spent encoding the frames and the question text.
    """

    answer_generation_duration: float
    """
    Seconds Ollama reports spent producing the answer.
    """

    vram_bytes_at_requested_context: int
    """
    What the model occupies in GPU memory once loaded at
    :attr:`requested_context_tokens`, read back from the serving engine rather than
    estimated -- the KV-cache the context window reserves grows with the context asked
    for, regardless of how much of it a given call actually fills, so this is a property
    of the request's size, not of this one answer.
    """

    answer: str = field(repr=False)
    """
    What the model answered, kept verbatim for the correctness grading that is done by
    a person reading this against the video, not by this experiment.
    """


# %% asking it, and measuring what that cost


@dataclass
class VLMQueryExperiment:
    """
    Asks one vision-language model every long-term-memory question at once about one
    episode's video, over Ollama's HTTP API, and measures what that cost in time and in
    GPU memory.
    """

    ollama_url: str = DEFAULT_OLLAMA_URL
    model_name: str = DEFAULT_MODEL_NAME
    frame_rate_hz: float = VLM_FRAME_RATE_HZ

    def run(
        self,
        episode_identifier: str,
        video_path: Path,
        questions: Sequence[str],
        context_tokens: int,
        context: Optional[VLMContext] = None,
    ) -> VLMQueryExperimentResult:
        """
        :param episode_identifier: The episode ``video_path`` and ``questions`` belong
            to, kept on the result so many episodes' rows can be told apart.
        :param video_path: The episode's recorded video.
        :param questions: The questions' English text (e.g. every
            :class:`~experiments.questions.long_term_memory.LongTermMemoryQuestion`
            subclass's ``.english``), asked together in one call.
        :param context_tokens: The context window to request from the model. Must hold
            the video's frames, the question text, ``context`` and the answer, or the
            call is refused outright (:class:`VLMContextExceeded`) rather than
            truncated.
        :param context: How much of CRAM's own knowledge of the scene to hand the model
            alongside the video -- :data:`VLMContextLevel.NONE` (:meth:`VLMContext.none`)
            if omitted, matching what a model with no access to CRAM would be given.
        :raises VLMContextExceeded: If the frames, questions and context did not fit.
        :raises VLMVideoUnreadable: If the video could not be opened.
        """
        context = context or VLMContext.none()
        frames = self._extract_frames(video_path)
        prompt = self._prompt_for(questions, len(frames), context.text)

        began = time.perf_counter()
        response = requests.post(
            "%s/api/chat" % self.ollama_url,
            json={
                "model": self.model_name,
                "messages": [{"role": "user", "content": prompt, "images": frames}],
                "options": {"num_ctx": context_tokens},
                "stream": False,
            },
            timeout=None,
        ).json()
        wall_clock_duration = time.perf_counter() - began

        if "error" in response:
            raise VLMContextExceeded(
                episode_identifier=episode_identifier,
                requested_context_tokens=context_tokens,
                server_message=response["error"],
            )

        return VLMQueryExperimentResult(
            episode_identifier=episode_identifier,
            model_name=self.model_name,
            context_level=context.level.value,
            context_text_bytes=len(context.text.encode()),
            number_of_questions=len(questions),
            number_of_frames=len(frames),
            video_duration_seconds=round(len(frames) / self.frame_rate_hz, 2),
            video_file_size_bytes=video_path.stat().st_size,
            requested_context_tokens=context_tokens,
            prompt_token_count=response["prompt_eval_count"],
            answer_token_count=response["eval_count"],
            wall_clock_duration=round(wall_clock_duration, 3),
            model_load_duration=round(response.get("load_duration", 0) / 1e9, 3),
            prompt_processing_duration=round(
                response["prompt_eval_duration"] / 1e9, 3
            ),
            answer_generation_duration=round(response["eval_duration"] / 1e9, 3),
            vram_bytes_at_requested_context=self.resident_vram_bytes(),
            answer=response["message"]["content"],
        )

    def resident_vram_bytes(self) -> int:
        """
        What :attr:`model_name` currently occupies in GPU memory, as the serving engine
        itself reports it, right after a call has answered.

        Read back rather than measured locally with ``nvidia-smi``: whatever runs this
        experiment is not guaranteed to share a host, let alone a GPU, with
        :attr:`ollama_url`, and Ollama's own bookkeeping is the one place both sides can
        reach.

        :raises VLMNotResident: If the serving engine no longer lists the model loaded.
        """
        models = requests.get("%s/api/ps" % self.ollama_url).json().get("models", [])
        for model in models:
            if model["name"] == self.model_name:
                return model["size_vram"]
        raise VLMNotResident(model_name=self.model_name)

    def _extract_frames(self, video_path: Path) -> List[str]:
        """
        The video's frames, base64-encoded, sampled at :attr:`frame_rate_hz`.

        Read directly with OpenCV rather than shelling out to ``ffmpeg``, which the
        container this runs in does not have installed.

        :param video_path: The video to sample.
        :raises VLMVideoUnreadable: If the video could not be opened at all.
        """
        capture = cv2.VideoCapture(str(video_path))
        if not capture.isOpened():
            raise VLMVideoUnreadable(video_path=video_path)

        native_fps = capture.get(cv2.CAP_PROP_FPS) or self.frame_rate_hz
        frame_stride = max(1, round(native_fps / self.frame_rate_hz))

        frames: List[str] = []
        frame_index = 0
        try:
            while True:
                read, frame = capture.read()
                if not read:
                    break
                if frame_index % frame_stride == 0:
                    encoded, buffer = cv2.imencode(".jpg", frame)
                    if encoded:
                        frames.append(base64.b64encode(buffer).decode())
                frame_index += 1
        finally:
            capture.release()
        return frames

    @staticmethod
    def _prompt_for(
        questions: Sequence[str], frame_count: int, context_text: str
    ) -> str:
        """
        The questions, folded into one prompt that tells the model what the frames are,
        with whatever extra context this variant hands it first.

        :param questions: The questions' English text, asked in the order given.
        :param frame_count: How many frames the prompt is describing, so the model is
            told what it was shown rather than left to guess.
        :param context_text: Extra context to put before the frames are introduced;
            empty for the no-context variant, in which case nothing is added.
        """
        numbered_questions = "\n".join(
            "%d. %s" % (number, question)
            for number, question in enumerate(questions, start=1)
        )
        frames_and_questions = (
            "These are %d frames, in order, sampled from the complete video "
            "recording of one robot sorting-task episode. Answer every question "
            "below, briefly, in the same order:\n%s"
        ) % (frame_count, numbered_questions)
        if not context_text:
            return frames_and_questions
        return "%s\n\n%s" % (context_text, frames_and_questions)
