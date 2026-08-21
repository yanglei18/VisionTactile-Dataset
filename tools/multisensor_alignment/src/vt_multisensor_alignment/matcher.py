from __future__ import annotations

from bisect import bisect_left, bisect_right
from collections.abc import Sequence

from .model import GenericSample, InterpolatedPose, TimedPose


def _timestamps(values: Sequence[int], context: str) -> tuple[int, ...]:
    result = tuple(values)
    if any(type(value) is not int or value < 0 for value in result):
        raise ValueError(f"{context} must contain non-negative integers")
    if any(left >= right for left, right in zip(result, result[1:])):
        raise ValueError(f"{context} must be strictly increasing")
    return result


def match_nearest_unique(
    reference_timestamps_ns: Sequence[int],
    candidate_timestamps_ns: Sequence[int],
    *,
    max_delta_ns: int,
) -> tuple[int | None, ...]:
    references = _timestamps(reference_timestamps_ns, "reference timestamps")
    candidates = _timestamps(candidate_timestamps_ns, "candidate timestamps")
    if type(max_delta_ns) is not int or max_delta_ns < 0:
        raise ValueError("max_delta_ns must be a non-negative integer")
    # Candidate edges form an ordered bipartite graph. A Fenwick tree computes
    # the maximum-cardinality monotonic chain, then minimizes its total absolute
    # residual. This avoids a locally nearest choice consuming the only sample
    # that can cover the following reference frame.
    chain = tuple[int, int, int | None]
    empty: chain = (0, 0, None)
    tree: list[chain] = [empty for _ in range(len(candidates) + 1)]
    edges: list[tuple[int, int, int | None]] = []

    def better(left: chain, right: chain) -> chain:
        left_key = (left[0], -left[1])
        right_key = (right[0], -right[1])
        if left_key != right_key:
            return left if left_key > right_key else right
        if left[2] is None:
            return right
        if right[2] is None:
            return left
        return left if left[2] < right[2] else right

    def query(candidate_count: int) -> chain:
        result = empty
        index = candidate_count
        while index > 0:
            result = better(result, tree[index])
            index -= index & -index
        return result

    def update(candidate_index: int, value: chain) -> None:
        index = candidate_index + 1
        while index < len(tree):
            tree[index] = better(tree[index], value)
            index += index & -index

    for reference_index, reference in enumerate(references):
        lower = bisect_left(candidates, reference - max_delta_ns)
        upper = bisect_right(candidates, reference + max_delta_ns)
        pending: list[tuple[int, chain]] = []
        for candidate_index in range(lower, upper):
            previous = query(candidate_index)
            edge_index = len(edges)
            edges.append(
                (reference_index, candidate_index, previous[2])
            )
            pending.append(
                (
                    candidate_index,
                    (
                        previous[0] + 1,
                        previous[1]
                        + abs(candidates[candidate_index] - reference),
                        edge_index,
                    ),
                )
            )
        for candidate_index, value in pending:
            update(candidate_index, value)

    result: list[int | None] = [None] * len(references)
    edge_index = query(len(candidates))[2]
    while edge_index is not None:
        reference_index, candidate_index, edge_index = edges[edge_index]
        result[reference_index] = candidate_index
    return tuple(result)


def _pose_timestamps(samples: Sequence[TimedPose]) -> tuple[int, ...]:
    if any(not isinstance(value, TimedPose) for value in samples):
        raise TypeError("samples must contain TimedPose values")
    timestamps = _timestamps(
        (value.host_realtime_ns for value in samples), "pose timestamps"
    )
    roles = {value.role for value in samples}
    identities = {value.tracker_id for value in samples}
    if len(roles) > 1 or len(identities) > 1:
        raise ValueError("pose samples must have one stable role and tracker identity")
    return timestamps


def interpolate_pose(
    samples: Sequence[TimedPose],
    timestamp_ns: int,
    max_gap_ns: int,
) -> InterpolatedPose | None:
    if type(timestamp_ns) is not int or timestamp_ns < 0:
        raise ValueError("timestamp_ns must be a non-negative integer")
    if type(max_gap_ns) is not int or max_gap_ns < 0:
        raise ValueError("max_gap_ns must be a non-negative integer")
    if not samples:
        return None
    timestamps = _pose_timestamps(samples)
    position = bisect_left(timestamps, timestamp_ns)
    if position < len(samples) and timestamps[position] == timestamp_ns:
        sample = samples[position]
        return InterpolatedPose(
            timestamp_ns=timestamp_ns,
            transform=sample.transform,
            bracket_gap_ns=0,
            before_sequence=sample.reference.sequence,
            after_sequence=sample.reference.sequence,
        )
    if position == 0 or position == len(samples):
        return None
    before = samples[position - 1]
    after = samples[position]
    gap = after.host_realtime_ns - before.host_realtime_ns
    if gap > max_gap_ns:
        return None
    fraction = (timestamp_ns - before.host_realtime_ns) / gap
    return InterpolatedPose(
        timestamp_ns=timestamp_ns,
        transform=before.transform.interpolate(after.transform, fraction),
        bracket_gap_ns=gap,
        before_sequence=before.reference.sequence,
        after_sequence=after.reference.sequence,
    )


def select_generic_sample(
    samples: Sequence[GenericSample],
    *,
    timestamp_ns: int,
    strategy: str,
    max_delta_ns: int,
) -> int | None:
    if type(timestamp_ns) is not int or timestamp_ns < 0:
        raise ValueError("timestamp_ns must be a non-negative integer")
    if type(max_delta_ns) is not int or max_delta_ns < 0:
        raise ValueError("max_delta_ns must be a non-negative integer")
    if strategy not in {"nearest", "previous"}:
        raise ValueError("strategy must be nearest or previous")
    if any(not isinstance(value, GenericSample) for value in samples):
        raise TypeError("samples must contain GenericSample values")
    timestamps = _timestamps(
        (value.timestamp_ns for value in samples), "generic timestamps"
    )
    if not timestamps:
        return None
    if strategy == "previous":
        selected = bisect_right(timestamps, timestamp_ns) - 1
        if selected < 0:
            return None
    else:
        insertion = bisect_left(timestamps, timestamp_ns)
        choices = {
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(timestamps)
        }
        selected = min(
            choices,
            key=lambda index: (abs(timestamps[index] - timestamp_ns), index),
        )
    if abs(timestamps[selected] - timestamp_ns) > max_delta_ns:
        return None
    return selected
