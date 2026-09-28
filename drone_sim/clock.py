"""Shared deterministic clock; seconds are derived from integer ticks, never accumulated."""
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class FixedStepClock:
    dt_s: float = .02
    tick: int = 0

    def __post_init__(self):
        if not isfinite(self.dt_s) or self.dt_s <= 0:
            raise ValueError('Clock step must be finite and positive')
        if type(self.tick) is not int or self.tick < 0 or not isfinite(self.tick*self.dt_s):
            raise ValueError('Clock tick and duration must be valid')

    @property
    def time_s(self):
        return self.tick*self.dt_s

    def advance(self):
        return FixedStepClock(self.dt_s,self.tick+1)
