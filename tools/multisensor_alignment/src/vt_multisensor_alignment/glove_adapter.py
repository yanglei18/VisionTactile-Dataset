"""Typed GloveState selection without inventing source times or joint poses."""

from __future__ import annotations

from dataclasses import replace

from .identity import check_identity
from .model import ClockObservation, GenericSample, MessageRef


GLOVE_STATE_TYPE = "vt_glove_msgs/msg/GloveState"


class GloveStateAdapter:
    """Keep valid live runs and their conservative temporal support.

    Invalid events may have no source time (or repeat the last source time).
    Closing at the last valid source and reopening at the next valid source
    prevents nearest/previous from silently filling that unknown interval.
    Source timestamps are never replaced with bag or publication timestamps.
    """

    def __init__(self, stream_name: str, *, expected_hand: str | None = None,
                 expected_frame_id: str | None = None,
                 reference_node_id: int | None = None) -> None:
        self.stream_name = stream_name
        self.expected_hand = expected_hand
        self.expected_frame_id = expected_frame_id
        if reference_node_id is not None and (type(reference_node_id) is not int
                                               or not 1 <= reference_node_id <= 12):
            raise ValueError("reference_node_id must be from 1 to 12")
        self.reference_node_id = reference_node_id
        self._identity: tuple[str, str] | None = None
        self.clock_observations: dict[str, list[ClockObservation]] = {}
        self._samples: dict[int, GenericSample] = {}
        self._run: list[GenericSample] = []
        self._context: tuple[str, int, int, tuple[tuple[int, int], ...]] | None = None
        self._bounded_start = False
        self._last_valid_time: int | None = None

    def _close_run(self, *, barrier: bool) -> None:
        if self._run:
            lower = self._run[0].timestamp_ns if self._bounded_start else None
            upper = self._run[-1].timestamp_ns if barrier else None
            for sample in self._run:
                self._samples[sample.timestamp_ns] = replace(
                    sample, valid_from_ns=lower, valid_until_ns=upper
                )
            self._run.clear()
        if barrier:
            self._bounded_start = True
            self._context = None

    def add(
        self,
        message: object,
        reference: MessageRef,
        *,
        header_stamp_ns: int,
    ) -> None:
        check_identity(message, reference.topic, expected_hand=self.expected_hand,
                       expected_frame_id=self.expected_frame_id)
        identity = (str(getattr(message, "hand", "")), str(getattr(message.header, "frame_id", "")))
        if self._identity is not None and identity != self._identity:
            raise ValueError(f"GloveState identity changed on {reference.topic}")
        reference_node = int(message.reference_node_id)
        if not 1 <= reference_node <= 12:
            raise ValueError(f"GloveState reference_node_id is invalid on {reference.topic}")
        if self.reference_node_id is not None and reference_node != self.reference_node_id:
            raise ValueError(f"GloveState reference_node_id mismatch on {reference.topic}")
        realtime = int(message.host_realtime_ns)
        monotonic = int(message.host_monotonic_ns)
        session = str(message.session_id)
        live = str(message.source_mode) == "live"
        timing_valid = (
            bool(message.time_valid)
            and realtime > 0
            and monotonic > 0
            and str(message.clock_domain) == "CLOCK_MONOTONIC"
            and str(message.timing_source) == "host_receive"
        )
        if reference.source_timestamp_ns != realtime or header_stamp_ns != realtime:
            raise ValueError(f"GloveState source/header timestamp mismatch on {reference.topic}")
        self._identity = identity
        if live and timing_valid and session:
            key = f"glove:{self.stream_name}:{session}"
            observations = self.clock_observations.setdefault(key, [])
            observation = ClockObservation(realtime, monotonic)
            # The solver can emit several results for one input sample. This
            # is not a clock reversal or extra measurement.
            if not observations or observation != observations[-1]:
                observations.append(observation)
        if not live or not timing_valid or not session or not bool(message.pose_valid):
            self._close_run(barrier=True)
            return

        node_ids = tuple(int(value) for value in message.source_node_ids)
        epochs = tuple(int(value) for value in message.source_filter_epochs)
        if len(node_ids) != len(epochs):
            raise ValueError(f"GloveState source epoch arrays mismatch on {reference.topic}")
        source_realtime = tuple(int(value) for value in message.source_realtime_ns)
        source_monotonic = tuple(int(value) for value in message.source_monotonic_ns)
        if (len(set(node_ids)) != len(node_ids) or reference_node not in node_ids
                or len(source_realtime) != len(node_ids) or len(source_monotonic) != len(node_ids)):
            raise ValueError(f"GloveState reference source arrays mismatch on {reference.topic}")
        palm_index = node_ids.index(reference_node)
        if (source_realtime[palm_index] != realtime or source_monotonic[palm_index] != monotonic):
            raise ValueError(f"GloveState reference source clock mismatch on {reference.topic}")
        context = (session, int(message.calibration_version), reference_node,
                   tuple(sorted(zip(node_ids, epochs))))
        if self._context is not None and context != self._context:
            self._close_run(barrier=True)
        if self._last_valid_time is not None and realtime < self._last_valid_time:
            raise ValueError(f"GloveState host realtime is not increasing: {self.stream_name}")
        self._context = context
        sample = GenericSample(self.stream_name, realtime, reference)
        if self._run and self._run[-1].timestamp_ns == realtime:
            self._run[-1] = sample
        else:
            self._run.append(sample)
        self._last_valid_time = realtime

    def finish(self) -> tuple[GenericSample, ...]:
        self._close_run(barrier=False)
        # Last output wins for the same source timestamp, including when a
        # calibration/session transition happens without a new palm sample.
        return tuple(self._samples[key] for key in sorted(self._samples))
