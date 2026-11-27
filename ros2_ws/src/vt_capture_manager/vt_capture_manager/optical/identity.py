"""Explicit hand labels for at most two visually identical cubes.

Associations are conditional on bounded relative motion between camera frames.
A physically different cube appearing at exactly the previous cube's position
without an observable break cannot be distinguished by vision alone. The
caller resets on camera/session changes, feeds frames including empty results,
and checks host-side freshness before accepting an operator bind.

A labelled cube that stops being detected keeps its label for a short window.
It is re-identified with the same bounded-motion envelope, measured from where
and when it was last seen, only if no other cube could explain the detection.
Beyond one frame-to-frame envelope, the other physical cube must be accounted
for (tracked, or hidden with a known last position); otherwise the operator
binds again.
"""

from collections import deque
from dataclasses import dataclass
from itertools import count
from math import dist, isfinite


HANDS = ("left", "right")


@dataclass(frozen=True)
class HandObservation:
    status: str
    observation: object | None = None


class BilateralTracker:
    """Carry the operator's hand labels only along unique 3-D associations."""

    # Frame-to-frame association covers camera intervals up to MAX_GAP_NS.
    # Motion outside the association envelope, centres closer than one nominal
    # cube edge, or non-monotonic time can break identity. A hidden labelled
    # cube is re-identified for at most REACQUIRE_WINDOW_NS after last seen.
    MAX_GAP_NS = 100_000_000
    REACQUIRE_WINDOW_NS = 500_000_000
    MAX_RELATIVE_SPEED_M_S = 2.0
    MEASUREMENT_SLACK_M = 0.008
    MIN_SEPARATION_M = 0.030
    MAX_CLICK_AGE_NS = 400_000_000
    HIT_RADIUS_PX = 16

    def __init__(self):
        self.reset()

    def reset(self):
        self._stamp_ns = None
        self._observations = ()
        self._centres = ()
        self._indices = {}
        # hand -> (last centre, stamp at which it was last detected)
        self._hidden = {}
        self._tokens = ()
        self._new_tokens = count()
        self._history = deque(maxlen=64)
        self._selectable = ()
        self._result = {hand: HandObservation("unbound") for hand in HANDS}
        return dict(self._result)

    def _break_tracks(self, status):
        for hand in (*self._indices, *self._hidden):
            self._result[hand] = HandObservation(status)
        self._indices.clear()
        self._hidden.clear()
        self._clear_frame()

    def _clear_frame(self):
        self._centres = self._observations = self._tokens = self._selectable = ()
        self._history.clear()

    def _hide(self, hand, old, last_seen_ns):
        self._hidden[hand] = (self._centres[old], last_seen_ns)
        self._result[hand] = HandObservation("lost")

    def _radius(self, elapsed_ns):
        return self.MEASUREMENT_SLACK_M + self.MAX_RELATIVE_SPEED_M_S * elapsed_ns / 1_000_000_000

    def _reacquire(self, stamp_ns, centres, observations, unexplained):
        """Return hidden labels to detections that only their own cube can explain."""
        for hand, (_, seen) in tuple(self._hidden.items()):
            if stamp_ns - seen > self.REACQUIRE_WINDOW_NS:
                del self._hidden[hand]
                self._result[hand] = HandObservation("unbound")
        reach = {hand: tuple(j for j in unexplained if dist(centre, centres[j]) <= self._radius(stamp_ns - seen))
                 for hand, (centre, seen) in self._hidden.items()}
        for hand, options in reach.items():
            if len(options) != 1 or any(options[0] in other for h, other in reach.items() if h != hand):
                continue
            other = next(h for h in HANDS if h != hand)
            if (stamp_ns - self._hidden[hand][1] > self.MAX_GAP_NS
                    and other not in self._indices and other not in self._hidden):
                # The other cube is unaccounted for and could be this detection.
                continue
            self._indices[hand] = options[0]
            self._result[hand] = HandObservation("tracked", observations[options[0]])
        for hand in self._indices:
            self._hidden.pop(hand, None)

    @staticmethod
    def _centre(observation):
        point = tuple(float(value) for value in observation.translation_m)
        if len(point) != 3 or not all(isfinite(value) for value in point) or point[2] <= 0:
            raise ValueError("Invalid centre in camera optical coordinates")
        return point

    def update(self, stamp_ns, observations):
        """Consume this camera frame and return the current per-hand status."""
        last = self._stamp_ns
        if type(stamp_ns) is not int or stamp_ns <= 0:
            self._stamp_ns = None
            self._break_tracks("unbound")
            return dict(self._result)
        self._stamp_ns = stamp_ns
        if last is not None and stamp_ns <= last:
            self._break_tracks("unbound")
            return dict(self._result)
        if last is not None and stamp_ns - last > self.MAX_GAP_NS:
            # No frame-to-frame association across this camera interval; the
            # labelled cubes may still be re-identified from their last centres.
            for hand, old in tuple(self._indices.items()):
                self._hide(hand, old, last)
            self._indices.clear()
            self._clear_frame()

        observations = tuple(observations)
        try:
            centres = tuple(self._centre(obs) for obs in observations)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._break_tracks("ambiguous")
            return dict(self._result)
        if len(centres) > len(HANDS) or (len(centres) == 2 and dist(*centres) <= self.MIN_SEPARATION_M):
            self._break_tracks("ambiguous")
            return dict(self._result)

        elapsed_s = 0 if last is None else (stamp_ns - last) / 1_000_000_000
        radius = self.MEASUREMENT_SLACK_M + self.MAX_RELATIVE_SPEED_M_S * elapsed_s
        # Include anonymous old cubes. A named cube cannot claim an observation
        # that could also have come from an anonymous cube.
        forward = [tuple(j for j, new in enumerate(centres) if dist(old, new) <= radius)
                   for old in self._centres]
        backward = [tuple(i for i, options in enumerate(forward) if j in options)
                    for j in range(len(centres))]
        pairs = {i: matches[0] for i, matches in enumerate(forward)
                 if len(matches) == 1 and len(backward[matches[0]]) == 1}
        for hand, old in tuple(self._indices.items()):
            if old in pairs:
                new = pairs[old]
                self._indices[hand] = new
                self._result[hand] = HandObservation("tracked", observations[new])
            else:
                del self._indices[hand]
                if forward[old]:
                    self._result[hand] = HandObservation("ambiguous")
                else:
                    self._hide(hand, old, last)

        inherited_tokens = {new: self._tokens[old] for old, new in pairs.items()}
        self._tokens = tuple(inherited_tokens[j] if j in inherited_tokens else next(self._new_tokens)
                             for j in range(len(centres)))
        self._selectable = tuple(j in inherited_tokens or not matches for j, matches in enumerate(backward))
        # Only detections that no cube visible in the previous frame can reach.
        self._reacquire(stamp_ns, centres, observations, [j for j, matches in enumerate(backward) if not matches])
        self._centres, self._observations = centres, observations
        pixels = tuple(getattr(obs, "center_px", None) for obs in observations)
        self._history.append((stamp_ns, pixels, self._tokens, self._selectable))
        return dict(self._result)

    def _bind_index(self, hand, index):
        if not self._selectable[index]:
            raise ValueError("目标的运动对应关系有歧义，请将两手分开后重新选择")
        if self._indices.get(hand) != index and (hand in self._indices or index in self._indices.values()):
            raise ValueError("该手或方块已经绑定到另一个目标")
        self._indices[hand] = index
        self._hidden.pop(hand, None)
        self._result[hand] = HandObservation("tracked", self._observations[index])
        return dict(self._result)

    def bind(self, hand, observation=None):
        """Label one current observation explicitly, or the only unassigned cube.

        For sequential setup, show one named hand, bind it, then keep it visible
        while bringing the other hand into view and binding the remaining cube.
        """
        if hand not in HANDS:
            raise ValueError("请选择左手或右手")
        if observation is not None:
            for j, obs in enumerate(self._observations):
                if obs is observation:
                    return self._bind_index(hand, j)
            raise ValueError("所选方块不属于当前图像")
        if hand in self._indices:
            return dict(self._result)
        unassigned = set(range(len(self._observations))) - set(self._indices.values())
        if len(unassigned) != 1:
            raise ValueError("绑定时须恰有一个可区分且尚未绑定的方块")
        return self._bind_index(hand, unassigned.pop())

    def bind_at(self, hand, center_px, stamp_ns):
        """Bind a clicked overlay centre in its original image/frame coordinates.

        A candidate must stay uniquely associated from the clicked frame to the
        latest processed frame. A newly arrived cube cannot inherit an old click.
        Hit-testing uses a 16-pixel radius in source resolution.
        """
        if hand not in HANDS:
            raise ValueError("请选择左手或右手")
        if (type(stamp_ns) is not int or self._stamp_ns is None
                or not 0 <= self._stamp_ns - stamp_ns <= self.MAX_CLICK_AGE_NS):
            raise ValueError("点击的图像已过期")
        try:
            point = tuple(float(v) for v in center_px)
            if len(point) != 2 or not all(isfinite(value) for value in point):
                raise ValueError
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("点击坐标无效") from exc
        for stamp, pixels, tokens, selectable in reversed(self._history):
            if stamp != stamp_ns:
                continue
            hits = [i for i, px in enumerate(pixels)
                    if px is not None and dist(point, px) <= self.HIT_RADIUS_PX]
            if len(hits) != 1 or not selectable[hits[0]]:
                raise ValueError("请点击一个可区分的方块中心")
            token = tokens[hits[0]]
            if token not in self._tokens:
                raise ValueError("选中的方块在这段时间已丢失或产生歧义，请重新点击")
            return self._bind_index(hand, self._tokens.index(token))
        raise ValueError("所选图像不在当前相机的最近检测记录中")
