"""
What it costs, in time and in hardware, to answer a long-term-memory question by asking
the EQL/SQL backend the episode's recorded rows, instead of showing a vision-language
model its video.

Paired with :mod:`experiments.vlm_experiments.query_benchmark`, which measures the same
kind of cost for the VLM backend. The two results classes share a field for every
concept both backends genuinely have (``wall_clock_duration``, ``number_of_questions``,
...); where a concept is backend-specific -- a GPU's resident VRAM has no SQL-side
counterpart, three-phase EQL timing has no VLM-side counterpart -- each class keeps its
own field rather than forcing a shared shape onto a cost the other backend does not pay.
:class:`~experiments.experiment_definitions.ExperimentsTable` only ever holds rows of one
type (see ``RowsOfDifferingTypes``), so the two are always read and reported as two
tables, set side by side, never merged into one.
"""

from __future__ import annotations

import resource
import time
from dataclasses import dataclass, field

from krrood.ormatic.data_access_objects.from_dao import FromDataAccessObjectState
from krrood.ormatic.eql_interface import eql_to_sql
from typing_extensions import List, Sequence, Tuple

from experiments.episodes.long_term_memory import LongTermMemory
from experiments.experiment_definitions import ExperimentResult
from experiments.questions.long_term_memory import LongTermMemoryQuestion

# %% one episode's worth of measurements


@dataclass
class SQLQueryExperimentResult(ExperimentResult):
    """
    Time and hardware measurements from asking the EQL/SQL backend every long-term-
    memory question about one recorded episode.

    One row per episode, asked as N separate queries rather than folded into one call
    the way the VLM's frames are: the SQL backend pays no per-call fixed cost worth
    amortising across questions the way loading a model onto a GPU does, so summing N
    separate askings is both the honest cost and the one the running pipeline already
    pays when a trial is actually queried.
    """

    episode_identifier: str
    """
    The episode the questions were asked about.
    """

    backend_name: str
    """
    Named the way :class:`~experiments.episodes.episode.AnsweredPredicate` already
    names backends elsewhere in this codebase, so a row here reads consistently with
    one read off a live run's own recorded queries.
    """

    number_of_questions: int
    """
    How many questions this row's durations were summed over.
    """

    number_of_ticks: int
    """
    How many Tick rows the episode's trials held -- the SQL-side analogue of
    :attr:`~experiments.vlm_experiments.query_benchmark.VLMQueryExperimentResult.
    number_of_frames`: both are how much of the recording the backend had to look
    through to answer.
    """

    number_of_events: int
    """
    How many events, across every tick, the episode's trials held.
    """

    wall_clock_duration: float
    """
    Seconds every question in this row took, summed -- the SQL-side analogue of
    :attr:`~experiments.vlm_experiments.query_benchmark.VLMQueryExperimentResult.
    wall_clock_duration`.
    """

    translation_duration: float
    """
    Seconds spent turning EQL into SQL (:func:`~krrood.ormatic.eql_interface.
    eql_to_sql`'s own ``build``/``translate`` step), summed over every question -- the
    SQL-side analogue of the VLM's prompt-processing time, since both are the cost of
    turning the question into something the backend can actually run.
    """

    execution_duration: float
    """
    Seconds spent actually running the translated SQL against the database
    (``EQLTranslator.evaluate()``), summed over every question.
    """

    object_construction_duration: float
    """
    Seconds spent turning the rows the database returned back into domain objects
    (``row.from_dao(...)``), summed over every question -- the SQL-side analogue of the
    VLM's answer-generation time, since both are the step that produces what a caller
    actually reads.
    """

    peak_memory_bytes: int
    """
    This process's peak resident set size so far (``ru_maxrss``), read from the
    operating system rather than estimated -- the SQL-side analogue of
    :attr:`~experiments.vlm_experiments.query_benchmark.VLMQueryExperimentResult.
    vram_bytes_at_requested_context`.

    ..note:: Unlike the VLM's VRAM figure, this is not attributable to the query alone:
        it is the whole interpreter's peak since process start, and this backend runs
        inside the same already-loaded, ROS- and torch-carrying process as everything
        else in the pipeline, not a dedicated serving process the way the VLM has. Read
        it as an upper bound on what answering cost, not as this backend's own
        footprint in isolation.
    """

    answers: str = field(repr=False)
    """
    Every question's English text and what its query selected, one per line, kept for
    the correctness grading done by a person reading it against the same episode's
    video, not by this experiment.

    What is recorded is the query's own raw selection (the domain objects
    :meth:`~experiments.episodes.long_term_memory.LongTermMemory.answer` returns), not
    a question's own :meth:`~experiments.questions.question.Question.ask`
    post-processing (e.g. the boolean a yes/no question collapses its solutions to) --
    collecting that too would mean answering every question twice, once for the timed
    phases here and once more through ``ask``, double-counting exactly the cost this
    experiment exists to measure.
    """


# %% asking it, and measuring what that cost


@dataclass
class SQLQueryExperiment:
    """
    Asks the EQL/SQL backend every long-term-memory question about one episode, one
    query at a time, and measures what that cost in time and in process memory.
    """

    backend_name: str = "SQL"

    def run(
        self,
        episode_identifier: str,
        memory: LongTermMemory,
        questions: Sequence[LongTermMemoryQuestion],
    ) -> SQLQueryExperimentResult:
        """
        :param episode_identifier: The episode ``questions`` are asked about, kept on
            the result so many episodes' rows can be told apart.
        :param memory: The long-term memory ``questions`` are put to.
        :param questions: The questions to ask, in the order they are asked.
        """
        number_of_ticks, number_of_events = self._recording_size(
            memory, episode_identifier
        )

        translation_duration = 0.0
        execution_duration = 0.0
        object_construction_duration = 0.0
        answered_lines: List[str] = []

        wall_clock_began = time.perf_counter()
        for question in questions:
            eql_query = question.query(memory)

            with memory.results_database.open_session() as session:
                translation_began = time.perf_counter()
                translated = eql_to_sql(eql_query, session)
                translation_ended = time.perf_counter()

                rows = translated.evaluate()
                execution_ended = time.perf_counter()

                conversion_state = FromDataAccessObjectState()
                answer = [row.from_dao(conversion_state) for row in rows]
                construction_ended = time.perf_counter()

            translation_duration += translation_ended - translation_began
            execution_duration += execution_ended - translation_ended
            object_construction_duration += construction_ended - execution_ended
            answered_lines.append("%s\n   -> %r" % (question.english, answer))
        wall_clock_duration = time.perf_counter() - wall_clock_began

        return SQLQueryExperimentResult(
            episode_identifier=episode_identifier,
            backend_name=self.backend_name,
            number_of_questions=len(questions),
            number_of_ticks=number_of_ticks,
            number_of_events=number_of_events,
            wall_clock_duration=round(wall_clock_duration, 3),
            translation_duration=round(translation_duration, 3),
            execution_duration=round(execution_duration, 3),
            object_construction_duration=round(object_construction_duration, 3),
            peak_memory_bytes=self._peak_memory_bytes(),
            answers="\n".join(answered_lines),
        )

    @staticmethod
    def _recording_size(
        memory: LongTermMemory, episode_identifier: str
    ) -> Tuple[int, int]:
        """
        How many ticks and events the episode's trials hold, read once up front rather
        than timed alongside the questions -- descriptive of the input, like the VLM
        benchmark's own frame count, not part of the work being measured.

        :param memory: The long-term memory to recall the episode's trials from.
        :param episode_identifier: The episode to measure.
        """
        trials = memory.recall_trials(episode_identifier)
        number_of_ticks = sum(len(trial.ticks) for trial in trials)
        number_of_events = sum(
            len(tick.events) for trial in trials for tick in trial.ticks
        )
        return number_of_ticks, number_of_events

    @staticmethod
    def _peak_memory_bytes() -> int:
        """
        This process's peak resident set size so far, in bytes.

        Linux reports ``ru_maxrss`` in kilobytes (macOS reports it in bytes already);
        this backend only ever runs inside the Linux cram container, so no platform
        branch is needed.
        """
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
