"""Track cumulative source counters across session boundaries in recording reports."""

class SourceCounter:
    def __init__(self) -> None:
        self.context: object = None
        self.highwater: int | None = None
        self.increases = self.initial_values_sum = self.regressions = 0

    def add(self, value: int, context: object) -> None:
        if self.context != context or self.highwater is None:
            self.context, self.highwater = context, value
            # A nonzero first observation may predate this bag: never silently
            # count it as loss that occurred during the recording.
            self.initial_values_sum += value
            return
        if value < self.highwater:
            self.regressions += 1
        else:
            self.increases += value - self.highwater
            self.highwater = value

    def document(self) -> dict:
        return {"observed_increase": self.increases,
                "initial_values_sum_not_attributed_to_recording": self.initial_values_sum,
                "counter_regressions": self.regressions}
