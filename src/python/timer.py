"""Simple Timer"""

from collections.abc import Callable
from datetime import timedelta
from time import monotonic

from numpy import clip, floor

from .utils import parse_none


class TimeDelta(timedelta):
    ONE_SECOND = timedelta(seconds=1)
    ONE_MINUTE = timedelta(minutes=1)
    ONE_DAY = timedelta(days=1)
    ONE_HOUR = timedelta(hours=1)

    def __init__(self, *args, **kwargs) -> None:
        super().__init__()

    def __str__(self) -> str:
        total_us = int(self.total_seconds() * 1_000_000)

        hours, remainder = divmod(total_us, 3_600_000_000)
        minutes, remainder = divmod(remainder, 60_000_000)
        seconds, microseconds = divmod(remainder, 1_000_000)

        return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{microseconds:06d}"

    def __add__(self, value):
        return self.from_timedelta(super().__add__(value))

    def __sub__(self, value):
        return self.from_timedelta(super().__sub__(value))

    def __neg__(self):
        return self.from_timedelta(super().__neg__())

    def __pos__(self):
        return self.from_timedelta(super().__pos__())

    def __mul__(self, value):
        return self.from_timedelta(super().__mul__(value))

    __rmul__ = __mul__

    def __truediv__(self, value):
        return super().__truediv__(value)

    def __mod__(self, value):
        return self.from_timedelta(super().__mod__(value))

    def __divmod__(self, value):
        q, r = super().__divmod__(value)
        return q, self.from_timedelta(r)

    def __abs__(self):
        return self.from_timedelta(super().__abs__())

    def total_minutes(self):
        return self / self.ONE_MINUTE

    def total_hours(self):
        return self / self.ONE_HOUR

    def total_days(self):
        return self / self.ONE_DAY

    @classmethod
    def from_timedelta(cls, td):
        return TimeDelta(
            days=td.days, seconds=td.seconds, microseconds=td.microseconds
        )


TZERO = TimeDelta(0)

type TimeDeltaArg = TimeDelta | int | None
type TimeDeltaConvert = Callable[..., TimeDelta]


def parse_tdelta(t: TimeDeltaArg, convert: TimeDeltaConvert = TimeDelta):
    t = parse_none(t, TZERO)
    return t if isinstance(t, TimeDelta) else convert(t)


def seconds(t: TimeDeltaArg = 0):
    return parse_tdelta(t, lambda t: TimeDelta(seconds=t))


def minutes(t: TimeDeltaArg = 0):
    return parse_tdelta(t, lambda t: TimeDelta(minutes=t))


def hours(t: TimeDeltaArg = 0):
    return parse_tdelta(t, lambda t: TimeDelta(hours=t))


def days(t: TimeDeltaArg = 0):
    return parse_tdelta(t, lambda t: TimeDelta(days=t))


class Timer:
    def __init__(self, duration=None):
        self._start = self.monotonic()
        self._duration = None if duration is None else minutes(duration)

    @property
    def elapsed(self) -> TimeDelta:
        return self.monotonic() - self._start

    @property
    def start(self) -> TimeDelta:
        return self._start

    @property
    def duration(self) -> TimeDelta | None:
        return self._duration

    @property
    def remaining(self) -> TimeDelta | None:
        if self.duration is None:
            return None
        return max(TZERO, self.duration - self.elapsed)

    @property
    def finished(self) -> bool | None:
        return None if self.remaining is None else self.remaining <= TZERO

    def reset(self) -> None:
        self._start = self.monotonic()

    def monotonic(self) -> TimeDelta:
        return TimeDelta(seconds=monotonic())


class PulseWidthModulationTimer(Timer):
    def __init__(
        self,
        period,
        duty_cycle,
        max_cycles: int | None = None,
        max_duration: TimeDeltaArg = None,
    ):
        self._period = minutes(period)
        # 0–100, pulse at start of cycle
        self._duty_cycle = float(clip(duty_cycle, 0, 100))

        max_duration = None if max_duration is None else minutes(max_duration)
        max_cycles = None if max_cycles is None else max_cycles
        if max_duration is not None and max_cycles is not None:
            n1 = int(floor(max_duration / self.period))
            ncycles = min(n1, max_cycles)
        elif max_cycles is not None:
            ncycles = max_cycles
        elif max_duration is not None:
            ncycles = int(floor(max_duration / self.period))
        else:
            ncycles = None

        self._ncycles = ncycles

        super().__init__()

    @property
    def ncycles(self) -> int | None:
        return self._ncycles

    @property
    def duration(self) -> TimeDelta | None:
        if self.ncycles is None:
            return None
        return self.period * self.ncycles

    @property
    def period(self) -> TimeDelta:
        return self._period

    @property
    def duty_cycle(self) -> float:
        return self._duty_cycle

    @property
    def pulse_duration(self) -> TimeDelta:
        # minutes
        return self.period * self.duty_cycle / 100

    @property
    def period_elapsed(self) -> TimeDelta:
        return self.elapsed % self.period

    @property
    def duty_cycle_elapsed(self) -> float:
        return self.period_elapsed / self.period * 100

    @property
    def current_cycle(self) -> int:
        return self.elapsed // self.period

    @property
    def pulse(self) -> bool:
        return TZERO <= self.period_elapsed < self.pulse_duration
