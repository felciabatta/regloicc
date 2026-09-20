"""RegloICC Datatypes"""

from typing import Any

from numpy import clip, format_float_scientific, uint

from src.python.utils import parse_none

from .constants import (
    CCW,
    CW,
    DIR_KEY,
    DT1_MAX,
    DT2_MAX,
    DT2_WIDTH,
    DT3_MAX,
    DT3_WIDTH,
    DT4_MAX,
    DT4_WIDTH,
    DT5_MAX,
    DT5_WIDTH,
    DT6_MAX,
    DT6_WIDTH,
    DT_ZERO,
    EMPTY,
    ENGLISH,
    ERR_CHN_SETTING,
    ERR_CYCLE_ZERO,
    ERR_MAX_RATE,
    ERR_MAX_VOL,
    FAILURE,
    ISO_LANG_KEY,
    LANG_KEY,
    MODE_FLOWRATE,
    MODE_RPM,
    MODE_TIME,
    MODE_TIME_PAUSE,
    MODE_VOL_PAUSE,
    MODE_VOL_RATE,
    MODE_VOL_TIME,
    NEGATIVE,
    POSITIVE,
    SP,
    STATUS_CHARS,
    STATUS_DESC_KEY,
    STR_LANG_KEY,
    SUCCESS,
    TIMETYPE_MAX,
    VOLTYPE_MAX,
    VOLTYPE_MIN,
    ICC_UNADDRESSED,
    ICC_PUMP_ADDRESSED,
    ICC_CHANNEL_ADDRESSED,
    ICC_PROXY_ADDRESSED,
    ICC_CHANNEL_PREFERRED,
    ICC_PROXY_PREFERRED,
    LEGACY_ADDRESSING,
    CHANNEL_ADDRESSING,
    LEGACY_DEFAULT,
    LEGACY_PROXY,
)


class RegloDataType:
    def __init__(self, value, width) -> None:
        self._value = value
        self._width = uint(width)
        self.check_width()
        pass

    def __len__(self):
        return self.width

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self._value!r}, width={self._width})"

    def __str__(self) -> str:
        # print("Executing RegloDataType.__str__()")
        return str(self._value)

    @property
    def value(self):
        return self._value

    @property
    def width(self):
        return self._width

    def check_width(self) -> None:
        len_str = len(str(self))
        assert (
            len_str == self._width
        ), f"Data string must be length {self._width} — got length {len_str}!"
        pass


class BooleanType(RegloDataType):
    """A single character indicating True or False."""

    def __init__(self, value: bool | int = False) -> None:
        super().__init__(bool(value), width=uint(1))

    def __str__(self) -> str:
        # print("Executing BooleanType.__str__()")
        return f"{self._value:01d}"

    def __repr__(self) -> str:
        return f"BooleanType({self._value})"

    @classmethod
    def from_str(cls, s):
        return cls(int(s))


class DirectionType(RegloDataType):
    """A single character indicating direction."""

    def __init__(self, value: str | int = CCW) -> None:
        # value = value.upper() if isinstance(value, str) else value
        super().__init__(DIR_KEY[value], width=uint(1))

    def __str__(self) -> str:
        return f"{self._value}"

    def __repr__(self) -> str:
        cls = type(self)
        if cls is DirectionType:
            key = {CCW: "CCW", CW: "CW"}
            return f"DirectionType({key[self._value]})"
        else:
            return f"{cls.__name__}()"

    @classmethod
    def from_str(cls, s):
        return cls(s)


class CounterClockwiseType(DirectionType):
    def __init__(self) -> None:
        super().__init__(CCW)


class ClockwiseType(DirectionType):
    def __init__(self) -> None:
        super().__init__(CW)


class DiscreteType(RegloDataType):
    def __init__(
        self,
        value: uint | int | None = DT_ZERO,
        width: uint | int = 1,
        vmin: int | uint | None = DT_ZERO,
        vmax: int | uint | None = None,
    ) -> None:
        self._vmin = uint(vmin) if isinstance(vmin, int) else vmin
        self._vmax = uint(vmax) if isinstance(vmax, int) else vmax
        if isinstance(value, (int, uint)):  # type: ignore
            value = uint(clip(value, self.vmin, self.vmax))
        super().__init__(value, width)
        pass

    def __str__(self) -> str:
        if self.value is None:
            return EMPTY
        else:
            return f"{self.value:0{self.width}}"

    def __repr__(self) -> str:
        name = type(self).__name__
        isbase = name == DiscreteType.__name__
        args = f", width={self.width}, vmax={self.vmax}" if isbase else ""
        return f"{name}({self.value}{args})"

    @property
    def vmin(self):
        return self._vmin

    @property
    def vmax(self):
        return self._vmax

    @property
    def vrange(self):
        return (self.vmin, self.vmax)

    @classmethod
    def from_str(cls, s, *args, **kwargs):
        return cls(uint(s), *args, **kwargs)


class QuantitativeDiscreteType(DiscreteType):
    """Discrete type representing measurable quantities."""

    _unit_name = ""

    def __init__(
        self,
        value=DT_ZERO,
        width=1,
        vmin=DT_ZERO,
        vmax=None,
        unit_name=_unit_name,
    ) -> None:
        self._unit_name = unit_name
        super().__init__(value, width, vmin, vmax)

    def __int__(self):
        return int(self.value)

    def __eq__(self, value):
        return int(self) == value

    def __gt__(self, other):
        return int(self) > other

    def __ge__(self, other):
        return int(self) >= other

    def __lt__(self, other):
        return int(self) < other

    def __le__(self, other):
        return int(self) <= other

    @property
    def unit_name(self):
        return self._unit_name


class DiscreteType1(QuantitativeDiscreteType):
    """Up to four characters representing a discrete integer value in base 10.

    Unused digits are not returned.

    - Width: 1 to 4
    - Range: 0 to 9999
    """

    def __init__(self, value=DT_ZERO) -> None:
        w = clip(len(str(value)), 1, 4)
        super().__init__(value, width=w, vmax=DT1_MAX)


class DiscreteType2(QuantitativeDiscreteType):
    """Four characters representing a discrete integer value in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 4
    - Range: 0 to 9999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT2_WIDTH, vmax=DT2_MAX)


class DiscreteType3(QuantitativeDiscreteType):
    """Six characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 6
    - Range: 0 to 999999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT3_WIDTH, vmax=DT3_MAX)


class DiscreteType4(QuantitativeDiscreteType):
    """Ten characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 10
    - Range: 0 to 4294967295 (ie uint32)
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT4_WIDTH, vmax=DT4_MAX)


class DiscreteType5(QuantitativeDiscreteType):
    """Three characters in base 10.
    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 3
    - Range: 0 to 999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT5_WIDTH, vmax=DT5_MAX)


class DiscreteType6(QuantitativeDiscreteType):
    """Five characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 5
    - Range: 0 to 99999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT6_WIDTH, vmax=DT6_MAX)


class FractionalType(RegloDataType):
    """`X…X.XX` — The width of the integer portion of the value is variable.

    The decimal point and two digits to the right of the decimal point are
    always provided.

    - Width: Variable
    """

    def __init__(self, value: float | int = 0) -> None:
        value = round(float(value), 2)
        len_str = len(self.val2str(value))
        super().__init__(value, width=len_str)

    def val2str(self, x):
        return f"{x:.2f}"

    def __str__(self) -> str:
        return self.val2str(self._value)

    def __repr__(self) -> str:
        return f"FractionalType({self._value})"

    @classmethod
    def from_str(cls, s):
        return cls(float(s))


class VolumeType1(RegloDataType):
    """`mmmmEse` — represents the scientific notation of `m.mmm × 10se`.

    - Width: 7
    - Units: mL

    For example, `1.200 × 10-2` is represented with `1200E-2`.
    Note that the decimal point is inferred after the first character.

    - `mmmm`: 4 character mantissa
    - `E`: Single character literal `E`
    - `s`: Sign for exponent (`+` or `-`)
    - `e`: Single digit exponent
    """

    _width = 7

    def __init__(self, value=float()) -> None:
        # input(repr(value))
        value = 0 if value == 0 else clip(value, VOLTYPE_MIN, VOLTYPE_MAX)
        super().__init__(value, width=type(self)._width)

    def float2sci(self, x):
        s = format_float_scientific(x, precision=3, min_digits=3, exp_digits=1)
        return s.upper()

    def __str__(self) -> str:
        return self.float2sci(self._value).replace(".", "")

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.float2sci(self._value)})"

    @classmethod
    def from_str(cls, s):
        m, mmmEse = s[0], s[1:]
        s = f"{m}.{mmmEse}"
        return cls(float(s))


class VolumeType2(VolumeType1):
    """`mmmmse` — represents the scientific notation of `m.mmm × 10se`.

    - Width: 6
    - Units: mL

    For example, `1.200 × 10-2` is represented with `1200-2`.
    Note that the decimal point is inferred after the first character.

    - `mmmm`: 4 character mantissa
    - `s`: Sign for exponent (`+` or `-`)
    - `e`: Single digit exponent
    """

    _width = 6

    def __str__(self) -> str:
        return super().__str__().replace("E", "")

    @classmethod
    def from_str(cls, s):
        m, mmm, se = s[0], s[1:4], s[-2:]
        s = f"{m}.{mmm}E{se}"
        return cls(float(s))


class TimeType(QuantitativeDiscreteType):
    """Right-justified sequence of numeric characters representing time.

    :param: value:
        Unsignedinteger representing time, where `tt.seconds == value * unit`.
    :param: width:
        Unsignedinteger representing allocating width of `str(tt)`.
    :param: unit:
        Number of seconds represented by one integer unit.
    """

    _unit_name = "s"

    def __init__(
        self, value=uint(), width=1, unit=1.0, unit_name=_unit_name
    ) -> None:
        self._unit = unit  # seconds
        super().__init__(
            value, width=width, vmax=TIMETYPE_MAX, unit_name=unit_name
        )

    @property
    def unit(self):
        return self._unit

    @property
    def seconds(self):
        return float(self._value * self._unit)

    @property
    def minutes(self):
        return float(self.seconds / 60)

    @property
    def hours(self):
        return float(self.hours / 60)

    def __repr__(self) -> str:
        name = type(self).__name__
        width_arg = (
            f", width={self._width}" if (name == TimeType.__name__) else ""
        )
        return f"{name}({self._value}{width_arg}, unit={self.unit})"


class TimeType1(TimeType):
    """Up to eight characters representing the time in 0 to 999 hours,
    with a resolution of 0.1 sec.

    Unused digits are not returned.

    - Width: 1 to 8
    - Units: 0.1 second (default)
    - Range: 0 to 35,964,000 (0 to 999 hr)
    """

    _unit_name = "s/10"

    def __init__(self, value=DT_ZERO, unit=0.1) -> None:
        w = clip(len(str(value)), 1, 8)
        super().__init__(value, width=w, unit=unit)


class TimeType2(TimeType):
    """Eight characters representing the time in 0 to 999 hours,
    with a resolution of 0.1 sec.

    The value is right-justified.
    Unused digits to the left are filled with zeros.

    - Width: 8
    - Units: 0.1 second (default)
    - Range: 0 to 35,964,000 (0 to 999 hr)
    """

    _unit_name = "s/10"

    def __init__(self, value=DT_ZERO, unit=0.1) -> None:
        super().__init__(value, width=8, unit=unit)


class StringType(RegloDataType):
    """String of length 0 to 64 characters.

    The string is delimited by the delimiting character of the message type
     that it is contained in.
    For *device command* requests, this is [CR].
    For *data responses*, this is [SP].
    The string cannot contain these characters.
    Other printable ASCII characters are okay.

    - Width: 0 to 64 characters
    - Range: ASCII (excluding [CR], [SP])
    """

    def __init__(self, value="") -> None:
        w = len(value)

        err_len = f"StringType must be 0–64 characters in length; got {w}"
        assert 0 <= w <= 64, err_len

        isvalid_chr = value.isascii() and not ("\r" in value)
        err_chr = (
            f"{type(self).__name__} can only contain ASCII characters"
            "(excluding `\\r`, ` `)"
        )
        assert isvalid_chr, err_chr

        super().__init__(value, width=w)
        pass

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self._value!r})"

    @classmethod
    def from_str(cls, s):
        return cls(s)


class CharType(StringType):
    def __init__(self, value) -> None:
        w = len(value)
        assert w == 1, "CharType must be a single character"
        super().__init__(value)


class StatusType(CharType):
    def __init__(self, value, *, desc="") -> None:
        self._desc = desc
        assert value in STATUS_CHARS, f"value must be one of {STATUS_CHARS}"
        return super().__init__(value)

    def __repr__(self) -> str:
        cls = type(self)
        if issubclass(cls, StatusType) and cls is not StatusType:
            return f"{cls.__name__}()"
        else:
            return f"StatusType({self.value!r})"

    @property
    def desc(self):
        return self._desc

    @classmethod
    def from_str(cls, s):
        assert s.isascii(), "Encountered non-ASCII response."
        if len(s) > 1:
            print("Warning: Encountered multi-character response.")
        elif len(s) == 0:
            print("Warning: Encountered empty response.")

        if s == SUCCESS:
            return SuccessType()
        elif s == FAILURE:
            return FailureType()
        elif s == NEGATIVE:
            return NegativeType()
        elif s == POSITIVE:
            return PositiveType()
        else:
            return MiscType.from_str(s)


class StandardStatusType(StatusType):
    _value = SUCCESS

    def __init__(self) -> None:
        value = self._value
        desc = STATUS_DESC_KEY[value]
        super().__init__(value, desc=desc)


class SuccessType(StandardStatusType):
    _value = SUCCESS


class FailureType(StandardStatusType):
    _value = FAILURE


class NegativeType(StandardStatusType):
    _value = NEGATIVE


class PositiveType(StandardStatusType):
    _value = POSITIVE


class MiscType(RegloDataType):
    def __repr__(self) -> str:
        return f"MiscType({self.value}, width={self.width})"

    def __str__(self) -> str:
        return str(self.value)

    @classmethod
    def from_str(cls, s: str):
        if s == "":
            return MissingType.from_str(s)
        else:
            return UnknownType.from_str(s)


class MissingType(MiscType):
    def __init__(self) -> None:
        super().__init__(None, width=0)

    def __repr__(self) -> str:
        return "MissingType()"

    def __str__(self) -> str:
        return str()

    @classmethod
    def from_str(cls, s: str = ""):
        assert (
            s == ""
        ), "Expected empty str '', cannot be converted to MissingType"
        return MissingType()


class UnknownType(MiscType):
    def __init__(self, value) -> None:
        # force width ≥ 1
        width = max(len(str(value)), 1)
        super().__init__(value, width=width)

    def __repr__(self) -> str:
        return f"UnknownType({self.value!r})"

    @classmethod
    def from_str(cls, s: str):
        return UnknownType(s)


class ByteCodeType(CharType):
    _aliases = ("",)

    def __init__(self, value, *, aliases=_aliases, desc="") -> None:
        self._aliases = aliases
        self._desc = desc
        super().__init__(value)

    def __repr__(self) -> str:
        v, alias, desc = self.value, self.aliases, self.desc
        return f"ModeType({v!r}, alias={alias!r}, desc={desc!r})"

    @property
    def aliases(self):
        return self._aliases

    @property
    def desc(self):
        return self._desc


class StandardByteCodeType(ByteCodeType):
    _value = "L"
    _aliases = ("RPM",)
    _desc = "Revolutions per minute"

    def __init__(self) -> None:
        super().__init__(self.value, aliases=self._aliases, desc=self._desc)

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"


class ModeRPM(StandardByteCodeType):
    _value = MODE_RPM
    _aliases = ("RPM",)
    _desc = "Dispense at a fixed rate, in revolutions per minute"


class ModeFlowrate(StandardByteCodeType):
    _value = MODE_FLOWRATE
    _aliases = ("Flow Rate", "Flowrate")
    _desc = "Dispense at a fixed rate, in mL per minute"


class ModeVolumeAtRate(StandardByteCodeType):
    _value = MODE_VOL_RATE
    _aliases = ("Volume (at Rate)", "Volume at rate")
    _desc = "Dispense a fixed volume at a preset flowrate/RPM"


class ModeVolumeOverPeriod(StandardByteCodeType):
    _value = MODE_VOL_TIME
    _aliases = ("Volume (over Time)", "Volume over period")
    _desc = "Dispense a fixed volume over a preset period"


class ModeVolumeAtRateCycle(StandardByteCodeType):
    _value = MODE_VOL_PAUSE
    _aliases = ("Volume+Pause", "Volume at rate (cycle)")
    _desc = (
        "Dispense a fixed volume at a preset flowrate/RPM "
        "for a number of cycles"
    )


class ModeRateOverPeriod(StandardByteCodeType):
    _value = MODE_TIME
    _aliases = ("Time", "Time (at Rate)", "Rate over period")
    _desc = "Dispense over a fixed period at a preset flowrate/RPM"


class ModeRateOverPeriodCycle(StandardByteCodeType):
    _value = MODE_TIME_PAUSE
    _aliases = ("Time+Pause", "Rate over period (cycle)")
    _desc = (
        "Dispense over a fixed period at a preset flowrate/RPM "
        "for a number of cycles"
    )


class CycleErrorType(StandardByteCodeType):
    _value = ERR_CYCLE_ZERO
    _aliases = (ERR_CYCLE_ZERO, "Cycle error")
    _desc = "Cycle count of 0"


class FlowErrorType(StandardByteCodeType):
    _value = ERR_MAX_RATE
    _aliases = (ERR_MAX_RATE, "Flowrate error")
    _desc = "Max flow rate exceeded or flow rate is set to 0"


class VolumeErrorType(StandardByteCodeType):
    _value = ERR_MAX_VOL
    _aliases = (ERR_MAX_VOL, "Volume error")
    _desc = "Max volume exceeded"


class LimitingValueUndefinedType(StringType):
    pass


class LimitingFlowrateType(VolumeType1):
    pass


class LimitingVolumeType(VolumeType1):
    pass


class LanguageType(DiscreteType):
    """A single numeric character representing a language.

    - Width: 1
    - Range: 0 to 3

    Options are

    - `0` for English
    - `1` for French
    - `2` for Spanish
    - `3` for German
    """

    def __init__(self, value=ENGLISH) -> None:
        value = value.lower() if isinstance(value, str) else value
        isvalid = value in LANG_KEY
        assert isvalid, f"Invalid language, choose from {set(LANG_KEY)}"
        super().__init__(LANG_KEY[value], width=1, vmax=3)

    @property
    def name(self):
        return STR_LANG_KEY[self._value]

    @property
    def iso(self):
        return ISO_LANG_KEY[self._value]

    def __repr__(self) -> str:
        return f"LanguageType({self.name.capitalize()!r})"


class AddressType(DiscreteType):
    """A single character representing a pump or channel address.

    - Width: 1
    - Range: 0 to 8
    """

    _vmin = 0
    _vmax = 8
    _width = 1

    def __init__(self, value: int | None = LEGACY_DEFAULT) -> None:
        super().__init__(
            value, width=self._width, vmin=self._vmin, vmax=self._vmax
        )

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self._value})"


class PumpAddressType(AddressType):
    _vmin = 1
    _vmax = 8


class ChannelAddressType(AddressType):
    _vmin = 1
    _vmax = 4


class ProxyAddressType(AddressType):
    _vmax = _vmin = 0

    def __init__(self) -> None:
        super().__init__(LEGACY_PROXY)


class ChannelOrPxAddressType(AddressType):
    _vmin = 0
    _vmax = 4

    def __init__(self, value=LEGACY_DEFAULT) -> None:
        if value == 0:
            print(
                "Warning: channel address (1–4) is preferred; pump behaviour "
                "may be unreliable when using placeholder address (0)"
            )
        super().__init__(value)


class ProxyOrChAddressType(AddressType):
    _vmin = 0
    _vmax = 4

    def __init__(self, value=LEGACY_PROXY) -> None:
        if value != LEGACY_PROXY:
            print(
                f"Warning: placeholder address ({LEGACY_PROXY}) is preferred"
            )
        super().__init__(value)


class UnAddressedType(AddressType):
    _vmax = _vmin = None
    _width = 0

    def __init__(self) -> None:
        super().__init__(None)

    def __repr__(self) -> str:
        return "UnAddressedType()"


class DiameterType(RegloDataType):
    def __init__(self, value: float, unit_name="mm") -> None:
        value = round(clip(value, 0.0, None), 2)
        self.unit_name = unit_name.strip()
        super().__init__(value, width=7)

    def __str__(self) -> str:
        return f"{self.value:.2f} {self.unit_name}"

    def __repr__(self) -> str:
        return f"DiameterType({self.value:.2f}, unit_name={self.unit_name!r})"

    @classmethod
    def from_str(cls, s: str):
        value, unit = s.split(SP, 1)
        return cls(float(value), unit_name=unit.strip())
