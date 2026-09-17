"""RegloICC Datatypes"""

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
    ENGLISH,
    FAILURE,
    ISO_LANG_KEY,
    LANG_KEY,
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
        value: uint | int = DT_ZERO,
        width: uint | int = 1,
        vmin: int | uint = DT_ZERO,
        vmax: int | uint | None = None,
    ) -> None:
        self._vmin = uint(vmin)
        self._vmax = uint(vmax) if isinstance(vmax, int) else vmax
        _value = uint(clip(value, self._vmin, self._vmax))
        super().__init__(_value, width)
        pass

    @property
    def vmin(self):
        return self._vmin

    @property
    def vmax(self):
        return self._vmax

    @property
    def vrange(self):
        return (self._vmin, self._vmax)

    def __str__(self) -> str:
        return f"{self._value:0{self._width}}"

    def __repr__(self) -> str:
        name = type(self).__name__
        isbase = name == DiscreteType.__name__
        args = f", width={self._width}, vmax={self._vmax}" if isbase else ""
        return f"{name}({self._value}{args})"

    @classmethod
    def from_str(cls, s, *args, **kwargs):
        return cls(uint(s), *args, **kwargs)


class DiscreteType1(DiscreteType):
    """Up to four characters representing a discrete integer value in base 10.

    Unused digits are not returned.

    - Width: 1 to 4
    - Range: 0 to 9999
    """

    def __init__(self, value=DT_ZERO) -> None:
        w = clip(len(str(value)), 1, 4)
        super().__init__(value, width=w, vmax=DT1_MAX)


class DiscreteType2(DiscreteType):
    """Four characters representing a discrete integer value in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 4
    - Range: 0 to 9999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT2_WIDTH, vmax=DT2_MAX)


class DiscreteType3(DiscreteType):
    """Six characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 6
    - Range: 0 to 999999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT3_WIDTH, vmax=DT3_MAX)


class DiscreteType4(DiscreteType):
    """Ten characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 10
    - Range: 0 to 4294967295 (ie uint32)
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT4_WIDTH, vmax=DT4_MAX)


class DiscreteType5(DiscreteType):
    """Three characters in base 10.
    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 3
    - Range: 0 to 999
    """

    def __init__(self, value=DT_ZERO) -> None:
        super().__init__(value, width=DT5_WIDTH, vmax=DT5_MAX)


class DiscreteType6(DiscreteType):
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


class TimeType(DiscreteType):
    """Right-justified sequence of numeric characters representing time.

    :param: value:
        Unsignedinteger representing time, where `tt.seconds == value * unit`.
    :param: width:
        Unsignedinteger representing allocating width of `str(tt)`.
    :param: unit:
        Number of seconds represented by one integer unit.
    """

    def __init__(
        self, value: uint | int = uint(), width: uint | int = 1, unit=1.0
    ) -> None:
        self._unit = unit  # seconds
        super().__init__(value, width=width, vmax=TIMETYPE_MAX)

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

        isvalid_chr = value.isascii() and not ("\r" in value or " " in value)
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
    - Range: 1 to 8
    """

    def __init__(self, value=1) -> None:
        super().__init__(value, width=1, vmin=1, vmax=8)

    def __repr__(self) -> str:
        return f"AddressType({self._value})"


class DiameterType(RegloDataType):
    def __init__(self, value: float, unit="mm") -> None:
        value = round(clip(value, 0.0, None), 2)
        self.unit = unit.strip()
        super().__init__(value, width=7)

    def __str__(self) -> str:
        return f"{self._value:.2f} {self.unit}"

    def __repr__(self) -> str:
        return f"DiameterType({self._value:.2f}, unit={self.unit!r})"

    @classmethod
    def from_str(cls, s: str):
        value, unit = s.split(SP, 1)
        return cls(float(value), unit=unit.strip())


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
