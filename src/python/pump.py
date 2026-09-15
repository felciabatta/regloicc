"""Reglo ICC Peristaltic Pump Control Library.

For managing the communication with a reglo icc with 3 independent channels.

Example
-------

```python
# Initialize the pump with the specified COM port
pump = RegloICC('COM3') # Replace 'COM5' with your actual COM port

# Start channel 3

pump.start_channel(channel)
mmmm, sgn, e = flux2mmmmse(50, from_unit="uL/min")
pump.set_direction(1, 0)
pump.set_mode(1, 1)
pump.set_flowrate(1, mmmm, sgn, e)
print(pump.get_flowrate(1))
pump.start_channel(1)
time.sleep(10)
# Stop channel 3
pump.stop_channel(1)
print('finito')

del pump

print('end')
```

References
----------

- [Original Code](https://blog.darwin-microfluidics.com/how-to-control-the-reglo-icc-pump-using-python-and-matlab/#protocol)
- [Manual](https://digitalassets.avantorsciences.com/adaptivemedia/rendition?id=17a108781e743ebcc52b17648b4016c56e87bc94&vid=64b21a6c8bfebf1fd2cafd071229d6e0939145eb&prid=original&clid=SAPDAM)
- [Product Page](https://www.avantorsciences.com/us/en/product/39213403/masterflex-ismatec-reglo-independent-channel-control-icc-peristaltic-pumps-avantor)
"""

import time
from datetime import datetime

from numpy import clip, format_float_scientific, sign, uint
from pandas import DataFrame
from pandas import concat as pdcat
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial

from .utils import get_port, parse_none

# ASCII DEFINTIONS
CR = "\r"
LF = "\n"
SP = " "
VB = "|"
CRLF = CR + LF
AT = "@"
CARET = "^"

# DATA CHARACTERS
SUCCESSFULL = "*"
UNSUCCESSFULL = "#"
NEGATIVE = "–"
POSITIVE = "+"

CW = "J"
CCW = "K"

ERR_CHN_SETTING = NEGATIVE
ERR_CYCLE_ZERO = "C"
ERR_MAX_RATE = "R"
ERR_MAX_VOL = "V"

MODE_RPM = "L"
MODE_FLOWRATE = "M"
MODE_VOL_RATE = "O"
MODE_VOL_TIME = "G"
MODE_VOL_PAUSE = "Q"
MODE_TIME = "N"
MODE_TIME_PAUSE = "P"

PUMP_RUNNING = POSITIVE
PUMP_STOPPED = NEGATIVE

FS_OPEN = POSITIVE
FS_GROUNDED = NEGATIVE

STOPPED_PUMP_COMPLETE = "A"
STOPPED_CALI_COMPLETE = "B"
STOPPED_MANUAL = "1"
STOPPED_OVERHEAT = "2"
STOPPED_OVERCURRENT = "3"
CSTATUS_PUMPING = "A"
CSTATUS_PAUSED_BTWN_CYCLES = "B"
CSTATUS_STOPPED = "C"
CSTATUS_CALI_PUMPING = "D"
CSTATUS_CALI_PENDING = "E"


# 1.0 COMMUNICATIONS MANAGEMENT
GET_CHN_ADR = "~"  # 1.1
SET_CHN_ADR = GET_CHN_ADR  # 1.2
GET_EVT_MSG = "xE"  # 1.3
SET_EVT_MSG = GET_EVT_MSG  # 1.4
GET_SERIAL_VER = "x!"  # 1.5

# 2.0 PUMP DRIVE
SET_START = "H"  # 2.1
SET_STOP = "I"  # 2.2
SET_PAUSE = "xI"  # 2.3
GET_DIR = "xD"  # 2.4
SET_CW = CW  # 2.5
SET_CCW = CCW  # 2.6
GET_DRIVE_ERR = "xe"  # 2.7

# 3.0 OPERATIONAL MODE & SETTINGS
GET_MODE = "xM"  # 3.1
SET_MODE_RPM = MODE_RPM  # 3.2
SET_MODE_FLOWRATE = MODE_FLOWRATE  # 3.3
SET_MODE_VOL_RATE = MODE_VOL_RATE  # 3.4
SET_MODE_VOL_TIME = MODE_VOL_TIME  # 3.5
SET_MODE_VOL_PAUSE = MODE_VOL_PAUSE  # 3.6
SET_MODE_TIME = MODE_TIME  # 3.7
SET_MODE_TIME_PAUSE = MODE_TIME_PAUSE  # 3.8
GET_RATE_OTHER = "xf"  # 3.9
SET_RATE_OTHER = GET_RATE_OTHER  # 3.10
GET_RPM = "S"  # 3.11
SET_RPM = GET_RPM  # 3.12
GET_FLOWRATE = "f"  # 3.13
SET_FLOWRATE = GET_FLOWRATE  # 3.14
GET_VOL = "v"  # 3.15
SET_VOL = GET_VOL  # 3.16
GET_RUN_TIME = "xT"  # 3.17
SET_RUN_TIME = GET_RUN_TIME  # 3.18
GET_PAUSE_TIME = "xP"  # 3.19
SET_PAUSE_TIME = GET_PAUSE_TIME  # 3.20
GET_CYCLE_COUNT = r"\""  # 3.21
SET_CYCLE_COUNT = GET_CYCLE_COUNT  # 3.22
GET_MAX_FLOWRATE = "?"  # 3.23
GET_MAX_FLOWRATE_CALIBRATED = "!"  # 3.24
GET_DISPENSE_TIME_FLOWRATE = "xv"  # 3.25
GET_DISPENSE_TIME_RPM = "xw"  # 3.26

# 4.0 CONFIGURATION
GET_DIAMETER = "+"  # 4.1
SET_DIAMETER = GET_DIAMETER  # 4.2
GET_BACKSTEP = "%"  # 4.3
SET_BACKSTEP = GET_BACKSTEP  # 4.4
RESET_CONFIG_DATA = "0"  # 4.5

# 5.0 CALIBRATION
GET_CAL_DIR = "xR"  # 5.1
SET_CAL_DIR = GET_CAL_DIR  # 5.2
GET_TARGET_VOL = "xU"  # 5.3
SET_TARGET_VOL = GET_TARGET_VOL  # 5.4
SET_ACTUAL_VOL = "xV"  # 5.5
GET_CAL_TIME = "xW"  # 5.6
SET_CAL_TIME = GET_CAL_TIME  # 5.7
GET_RUNTIME_SINCE = "xX"  # 5.8
SET_CAL_START = "xY"  # 5.9
SET_CAL_CANCEL = "xZ"  # 5.10

# 6.0 SYSTEM
GET_FIRMWARE = "("  # 6.1
SET_FACTORY_ROLLVOL = "xt"  # 6.2
SAVE_SET_ROLLVOL = "xs"  # 6.3
RESET_ROLLVOL = "xu"  # 6.4
SET_PUMP_NAME = "xN"  # 6.5
GET_SERIALNO = "xS"  # 6.6
SET_SERIALNO = GET_SERIALNO  # 6.7
GET_LANG = "xL"  # 6.8
SET_LANG = GET_LANG  # 6.9
GET_CHANNELNO = "xA"  # 6.10
SET_CHANNELNO = GET_CHANNELNO  # 6.11
GET_ROLLERNO = "xB"  # 6.12
SET_ROLLERNO = GET_ROLLERNO  # 6.13
GET_REV_SINCE_RESET = "xC"  # 6.14
GET_VOL_SINCE_RESET = "xG"  # 6.15
GET_TIME_SINCE_RESET = "xJ"  # 6.16
SET_GUI_ON = "A"  # 6.17
SET_GUI_OFF = "B"  # 6.18
SET_DISPLAY_NUM = "D"  # 6.19
SET_DISPLAY_CHR = "DA"  # 6.20
GET_RUN_STATUS = "E"  # 6.21
GET_MODEL_INFO = "#"  # 6.22
GET_HEAD_CODE = ")"  # 6.23
SET_HEAD_CODE = GET_HEAD_CODE  # 6.24
GET_TIME_10THS = "V"  # 6.25
SET_TIME_10THS = GET_TIME_10THS  # 6.26
SET_TIME_MINS = "VM"  # 6.27
SET_TIME_HRS = "VH"  # 6.28
GET_ROLLSTEP_LO = "U"  # 6.29
SET_ROLLSTEP_LO = GET_ROLLSTEP_LO  # 6.30
GET_ROLLSTEP_HI = "u"  # 6.31
SET_ROLLSTEP_HI = GET_ROLLSTEP_HI  # 6.32
GET_ROLLVOL_CAL = "r"  # 6.33
SET_ROLLVOL_CAL = GET_ROLLVOL_CAL  # 6.34
RESET_CAL = "000000"  # 6.35
GET_PAUSE_10THS = "T"  # 6.36
SET_PAUSE_10THS = GET_PAUSE_10THS  # 6.37
SET_PAUSE_MINS = "TM"  # 6.38
SET_PAUSE_HRS = "TH"  # 6.39
GET_VOLALL_SINCE_RESET = ":"  # 6.40
SAVE_SET_ALL = "*"  # 6.41
GET_FOOTSWITCH_STATUS = "C"  # 6.42

# TUBE IDs
TUBE_IDS = {
    0: 0.13,
    1: 0.19,
    2: 0.25,
    3: 0.38,
    4: 0.44,
    5: 0.51,
    6: 0.57,
    7: 0.64,
    8: 0.76,
    9: 0.89,
    10: 0.95,
    11: 1.02,
    12: 1.09,
    13: 1.14,
    14: 1.22,
    15: 1.30,
    16: 1.42,
    17: 1.52,
    18: 1.65,
    19: 1.75,
    20: 1.85,
    21: 2.06,
    22: 2.29,
    23: 2.54,
    24: 2.79,
    25: 3.17,
}

# STATUS RESPONSE
STATUS_DESC_KEY = {
    SUCCESSFULL: "The command was executed successfully.",
    UNSUCCESSFULL: "The command was not executed successfully.",
    NEGATIVE: "Negative. See command definition for context.",
    POSITIVE: "Positive. See command definition for context.",
}

# DISCRETETYPE CONSTANTS
DT_ZERO = uint()

DT1_MAX = 9999

DT2_WIDTH = 4
DT2_MAX = 9999

DT3_WIDTH = 6
DT3_MAX = 999999

DT4_WIDTH = 10
DT4_MAX = 2**32 - 1  # 4,294,967,295; ie max uint32

DT5_WIDTH = 3
DT5_MAX = 999

DT6_WIDTH = 5
DT6_MAX = 99999

# VOLTYPE CONSTANTS
VOLTYPE_MIN = 1.000e-9
VOLTYPE_MAX = 9.999e9
TIMETYPE_MAX = 35964000

# DATATYPE KEYS
DIR_KEY = {
    CCW: CCW,
    CW: CW,
    +1: CCW,
    -1: CW,
    "ccw": CCW,
    "cw": CW,
}
STR_ENGLISH = "english"
STR_FRENCH = "french"
STR_SPANISH = "spanish"
STR_GERMAN = "german"
ISO_ENGLISH = "en"
ISO_FRENCH = "fr"
ISO_SPANISH = "es"
ISO_GERMAN = "de"
ENGLISH = 0
FRENCH = 1
SPANISH = 2
GERMAN = 3
LANG_KEY = {
    ENGLISH: ENGLISH,
    FRENCH: FRENCH,
    SPANISH: SPANISH,
    GERMAN: GERMAN,
    STR_ENGLISH: ENGLISH,
    STR_FRENCH: FRENCH,
    STR_SPANISH: SPANISH,
    STR_GERMAN: GERMAN,
    ISO_ENGLISH: ENGLISH,
    ISO_FRENCH: FRENCH,
    ISO_SPANISH: SPANISH,
    ISO_GERMAN: GERMAN,
}
STR_LANG_KEY = {
    ENGLISH: STR_ENGLISH,
    FRENCH: STR_FRENCH,
    SPANISH: STR_SPANISH,
    GERMAN: STR_GERMAN,
    STR_ENGLISH: STR_ENGLISH,
    STR_FRENCH: STR_FRENCH,
    STR_SPANISH: STR_SPANISH,
    STR_GERMAN: STR_GERMAN,
    ISO_ENGLISH: STR_ENGLISH,
    ISO_FRENCH: STR_FRENCH,
    ISO_SPANISH: STR_SPANISH,
    ISO_GERMAN: STR_GERMAN,
}
ISO_LANG_KEY = {
    ENGLISH: ISO_ENGLISH,
    FRENCH: ISO_FRENCH,
    SPANISH: ISO_SPANISH,
    GERMAN: ISO_GERMAN,
    STR_ENGLISH: ISO_ENGLISH,
    STR_FRENCH: ISO_FRENCH,
    STR_SPANISH: ISO_SPANISH,
    STR_GERMAN: ISO_GERMAN,
    ISO_ENGLISH: ISO_ENGLISH,
    ISO_FRENCH: ISO_FRENCH,
    ISO_SPANISH: ISO_SPANISH,
    ISO_GERMAN: ISO_GERMAN,
}
"""Based on ISO 639 language codes"""


# EVENT MESSAGES
EVT_ID = CARET
EVT_CHANNEL_STOP = "X"
EVT_CHANNEL_UPDATE = "U"
STOPPED_DESC_KEY = {
    STOPPED_PUMP_COMPLETE: (
        "Pumping complete (expiration of timer or volume limit reached)."
    ),
    STOPPED_CALI_COMPLETE: "Calibration pumping complete.",
    STOPPED_MANUAL: (
        "Manual (emergency) stop event (user presses stop button on the pump)."
    ),
    STOPPED_OVERHEAT: "Over temperature condition.",
    STOPPED_OVERCURRENT: "Over current condition.",
}
CSTATUS_DESC_KEY = {
    CSTATUS_PUMPING: "Pumping",
    CSTATUS_PAUSED_BTWN_CYCLES: "Paused between cycles",
    CSTATUS_STOPPED: "Stopped",
    CSTATUS_CALI_PUMPING: "Calibration pumping",
    CSTATUS_CALI_PENDING: (
        "Calibration pending (waiting for measured value to be entered)"
    ),
}
EVT_DESC_KEY = {
    EVT_CHANNEL_STOP: STOPPED_DESC_KEY,
    EVT_CHANNEL_UPDATE: CSTATUS_DESC_KEY,
}


class RegloDataType:
    def __init__(self, value, width) -> None:
        self._value = value
        self._width = uint(width)
        self.check_width()
        pass

    @property
    def value(self):
        return self._value

    @property
    def width(self):
        return self._width

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self._value!r}, width={self._width})"

    def __str__(self) -> str:
        # print("Executing RegloDataType.__str__()")
        return str(self._value)

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

    def __init__(self, value=+1) -> None:
        value = value.lower() if isinstance(value, str) else value
        super().__init__(DIR_KEY[value], width=uint(1))

    def __str__(self) -> str:
        return f"{self._value}"

    def __repr__(self) -> str:
        key = {CCW: "CCW", CW: "CW"}
        return f"DirectionType({key[self._value]})"

    @classmethod
    def from_str(cls, s):
        return cls(s)


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
        return f"FractionalType{self._value}"

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


class RegloMessage:
    """Abstract base class for all messages sent or received"""

    _datetime = None

    def __init__(self, *, datetime=_datetime) -> None:
        self.datetime = datetime
        pass

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"

    @classmethod
    def now(cls):
        return datetime.now().astimezone()

    @property
    def datetime(self):
        return self._datetime

    @datetime.setter
    def datetime(self, value):
        self._datetime = value

    def log_datetime(self):
        self.datetime = self.now()
        return self.datetime


class DeviceMessage(RegloMessage):
    """Abstract base class for messages received from the device"""


class StatusResponse(DeviceMessage):
    BYTES = 1

    def __init__(self, status: str, *, desc="", **kwargs) -> None:
        assert len(status) == 1, "StatusResponse must be a single character."
        self.status = status
        self.desc = desc
        return super().__init__(**kwargs)

    def __repr__(self) -> str:
        name = type(self).__name__
        args = f"{self!r}" if (name == StatusResponse.__name__) else ""
        return f"{name}({args})"

    def __str__(self) -> str:
        return self.status

    @classmethod
    def from_str(cls, s: str, **kwargs):
        assert s.isascii(), "Encountered non-ASCII response."
        assert len(s) == 1, "Encountered multi-character response."
        match s:
            case r if r == SUCCESSFULL:
                return StatusSuccessfull(**kwargs)
            case r if r == UNSUCCESSFULL:
                return StatusUnsuccessfull(**kwargs)
            case r if r == NEGATIVE:
                return StatusNegative(**kwargs)
            case r if r == POSITIVE:
                return StatusPositive(**kwargs)
            case _:
                return StatusUnknown(s, **kwargs)

    @classmethod
    def read(cls, serial: Serial):
        s = serial.read(cls.BYTES).decode()
        return cls.from_str(s, datetime=cls.now())


class StatusUnknown(StatusResponse):
    def __init__(self, status: str, **kwargs) -> None:
        super().__init__(status, desc="Unknown status.", **kwargs)


class StatusSuccessfull(StatusResponse):
    def __init__(self, **kwargs) -> None:
        stat = SUCCESSFULL
        desc = STATUS_DESC_KEY[stat]
        return super().__init__(stat, desc=desc, **kwargs)


class StatusUnsuccessfull(StatusResponse):
    def __init__(self, **kwargs) -> None:
        stat = UNSUCCESSFULL
        desc = STATUS_DESC_KEY[stat]
        return super().__init__(stat, desc=desc, **kwargs)


class StatusNegative(StatusResponse):
    def __init__(self, desc=None, **kwargs) -> None:
        stat = NEGATIVE
        desc = parse_none(desc, STATUS_DESC_KEY[stat])
        return super().__init__(stat, desc=desc, **kwargs)


class StatusPositive(StatusResponse):
    def __init__(self, desc=None, **kwargs) -> None:
        stat = POSITIVE
        desc = parse_none(desc, STATUS_DESC_KEY[stat])
        return super().__init__(stat, desc=desc, **kwargs)


class DataMessage(DeviceMessage):
    # DEFAULTS
    _BEGIN = ""
    _SEP = SP
    _END = CRLF
    _types = None

    def __init__(
        self, *args, begin=_BEGIN, sep=_SEP, end=_END, **kwargs
    ) -> None:
        self._args = args
        self._begin = begin
        self._sep = sep
        self._end = end
        return super().__init__(**kwargs)

    @property
    def args(self):
        return self._args

    @property
    def types(self):
        return parse_none(self._types, tuple(type(a) for a in self._args))

    @property
    def begin(self):
        return self._begin

    @property
    def sep(self):
        return self._sep

    @property
    def end(self):
        return self._end

    def __len__(self):
        return len(self.args)

    def __str__(self) -> str:
        args = self.sep.join(str(a) for a in self.args)
        return f"{self.begin}{args}{self.end}"

    def __repr__(self) -> str:
        optargs = [repr(a) for a in self.args]
        kws = f"begin={self.begin!r}, sep={self.sep!r}, end={self.end!r}"
        return f"DataMessage({", ".join(optargs + [kws])})"

    @classmethod  # NOTE: specialise for subclass call signature
    def parse(cls, s, begin: int | str, sep: str, end: int | str, **kwargs):
        """Parse message string and return `cls_args`, `cls_kwargs`"""
        begin = s[:begin] if isinstance(begin, int) else begin
        end = s[-end:] if isinstance(end, int) else end
        str_data = s.rstrip(end).lstrip(begin)
        cls_kwargs = dict(begin=begin, sep=sep, end=end, **kwargs)
        cls_args = str_data.split(sep)
        return cls.convert_args(cls_args), cls_kwargs

    @classmethod  # automatically convert if types provided by subclass
    def convert_args(cls, *args):
        types = cls._types
        if isinstance(types, (tuple, list)) and len(args) == len(types):
            return tuple(t(arg) for (t, arg) in zip(types, args))
        else:
            return args

    @classmethod  # NOTE: no need to specialise
    def from_str(cls, s: str, *args, **kwargs):
        """Parse message string and convert to a DataMessage type"""
        cls_args, cls_kwargs = cls.parse(s, *args, **kwargs)
        return cls(*cls_args, **cls_kwargs)

    @classmethod
    def read(cls, serial: Serial, expected=None, *args, **kwargs):
        """Read serial message and convert to a DataMessage type"""
        END = kwargs["end"] if "end" in kwargs else cls._END
        expected = parse_none(expected, END)
        s = serial.read_until(expected.encode("ascii")).decode()
        return cls.from_str(s, *args, datetime=cls.now(), **kwargs)


class DataResponse(DataMessage):
    """One or more data parameter values which are delimited by spaces.

    A data response is sent when parameter values must be provided in response
    to a request message.

    - Data Parameter N: A data value returned by the pump.
      The format of each data parameter is defined by the request message that
       a data response message is sent in response to.
      See each request message definition for the definition associated data
       response messages.
    """

    _begin = ""
    _sep = SP
    _end = CRLF

    def __init__(self, *args, **kwargs) -> None:
        kwargs = dict(begin=self.begin, sep=self.sep, end=self.end, **kwargs)
        return super().__init__(*args, **kwargs)

    def __repr__(self) -> str:
        optargs = [repr(a) for a in self.args]
        return f"{type(self).__name__}({", ".join(optargs)})"

    @classmethod  # NOTE: parent method can convert args
    def parse(cls, s, **kwargs):
        cls_args, _ = super().parse(s, cls._begin, cls._sep, cls._end)
        return cls_args, dict(**kwargs)

    @classmethod
    def read(cls, serial: Serial, **kwargs):
        return super().read(serial, cls._end, **kwargs)


# 1.0 COMMUNICATIONS MANAGEMENT
class ChannelAddressingResponse(DataResponse):
    _types = (BooleanType,)


class EventMessagingResponse(DataResponse):
    _types = (BooleanType,)


class SerialVersionResponse(DataResponse):
    _types = (DiscreteType1,)


# 2.0 PUMP DRIVE
class DirectionResponse(DataResponse):
    _types = (DirectionType,)


class PumpErrorResponse(DataResponse):
    _types = (CharType, CharType)  # NOTE: CHECK FORMAT MANUALLY


# 3.0 OPERATIONAL MODES AND SETTINGS
class ModeResponse(DataResponse):
    _types = (CharType,)


class RateOtherResponse(DataResponse):
    _types = (BooleanType,)  # NOTE: double check manually


class RPMResponse(DataResponse):
    _types = (FractionalType,)


class FlowrateResponse(DataResponse):
    _types = (VolumeType1,)


class VolumeFixedResponse(DataResponse):
    _types = (VolumeType1,)


class TimeFixedResponse(DataResponse):
    _types = (TimeType1,)


class PauseTimeResponse(DataResponse):
    _types = (TimeType1,)


class CycleCountResponse(DataResponse):
    _types = (DiscreteType1,)


# class MaxFlowrateResponse(DataResponse):
#     _types = ()  # NOTE: CHECK FORMAT MANUALLY


# class MaxFlowrateCalibratedResponse(DataResponse):
#     _types = ()  # NOTE: CHECK FORMAT MANUALLY


class DispenseTimeByFlowrateResponse(DataResponse):
    _types = (TimeType1,)


class DispenseTimeByRPMResponse(DataResponse):
    _types = (TimeType1,)


# 4.0 CONFIGURATION
class TubeDiameterResponse(DataResponse):
    _types = (DiameterType,)


class BackstepsResponse(DataResponse):
    _types = (DiscreteType1,)


# 5.0 CALIBRATION
class CalDirectionResponse(DataResponse):
    _types = (DirectionType,)


class CalTargetVolumeResponse(DataResponse):
    _types = (VolumeType1,)


class CalActualVolumeResponse(DataResponse):
    _types = (VolumeType1,)


class CalTimeResponse(DataResponse):
    _types = (TimeType1,)


class RuntimeSinceCalResponse(DataResponse):
    _types = (TimeType2,)


# 6.0 SYSTEM
class FirmwareVersionResponse(DataResponse):
    _types = (DiscreteType2,)


class PumpSerialNoResponse(DataResponse):
    _types = (StringType,)


class PumpLanguageResponse(DataResponse):
    _types = (LanguageType,)


class NumberOfChannelsResponse(DataResponse):
    _types = (DiscreteType1,)


class NumberOfRollersResponse(DataResponse):
    _types = (DiscreteType1,)


class RevsSinceResetResponse(DataResponse):
    _types = (DiscreteType4,)


class VolumeSinceResetResponse(DataResponse):
    # NOTE: distinguish from GetTotalVolumeSinceReset?
    _types = (DiscreteType4,)


class TimePumpedSinceResetResponse(DataResponse):
    _types = (DiscreteType4,)


class ModelInfoResponse(DataResponse):
    _types = (StringType, DiscreteType1, DiscreteType1)


class PumpHeadModelResponse(DataResponse):
    _types = (DiscreteType1,)


class PumpTime10thsResponse(DataResponse):
    # NOTE: distinguish from GetTimeFixed?
    _types = (DiscreteType1,)


class LowOrderStepsResponse(DataResponse):
    _types = (DiscreteType3,)


class HighOrderStepsResponse(DataResponse):
    _types = (DiscreteType3,)


class RollerVolumeResponse(DataResponse):
    _types = (VolumeType1,)  # NOTE: in nl units?


class PauseTime10thsResponse(DataResponse):
    # NOTE: distinguish from GetPauseTime?
    _types = (DiscreteType1,)


# class TotalVolumeSinceResetResponse(DataResponse):
#     # NOTE: distinguish from GetVolumeSinceReset?
#     _types = ()  # NOTE: CHECK FORMAT MANUALLY


class EventMessage(DataMessage):
    _prefix = EVT_ID
    _code = EVT_CHANNEL_UPDATE
    _begin = 2
    _sep = VB
    _end = CRLF
    _expected = _end

    def __init__(self, *args, code=None) -> None:
        cls = type(self)
        self._code = parse_none(code, cls._code)
        kwargs = dict(begin=self.begin, sep=cls._sep, end=cls._end)
        super().__init__(*args, **kwargs)
        del self._begin

    @property
    def begin(self):
        return self._prefix + self._code

    @property
    def code(self):
        return self._code

    def __repr__(self) -> str:
        kws = [f"code={self.code!r}"]
        optargs = [repr(a) for a in self.args]
        return f"EventMessage({", ".join(optargs + kws)})"

    @classmethod
    def parse(cls, s):  # NOTE: parent method can convert args
        cls_args, cls_kwargs = super().parse(
            s, begin=cls._begin, sep=cls._sep, end=cls._end
        )
        code = cls_kwargs["begin"][-1]
        return cls_args, dict(code=code)

    @classmethod
    def read(cls, serial: Serial):
        return super().read(serial, cls._expected)


class StoppingEvent(EventMessage):
    _code = EVT_CHANNEL_STOP


class UpdateEvent(EventMessage):
    _code = EVT_CHANNEL_UPDATE


class RequestMessage(RegloMessage):
    _RTYPES = (StatusResponse, DataResponse)

    def __init__(
        self,
        addr: int | uint = 0,
        end=CRLF,
        response: None | StatusResponse | DataResponse = None,
        rtype: None | type[StatusResponse] | type[DataResponse] = None,
    ) -> None:
        assert 0 <= addr <= 8, f"Address must be in range 0–8; got {addr}"
        self._addr = uint(addr)
        self._end = end
        self._response = response
        self._rtype = rtype
        pass

    def __call__(self, serial: Serial):
        return self.write(serial)

    def __len__(self):
        return len(str(self))

    def __repr__(self) -> str:
        return f"RequestMessage({int(self.addr)}, end={self.end!r})"

    def __str__(self):
        return self.end

    @property
    def addr(self):
        return self._addr

    @property
    def end(self):
        return self._end

    @property
    def response(self):
        return self._response

    @response.setter
    def response(self, value):
        if isinstance(value, self._RTYPES):
            value = value
        elif self.rtype is type(None):
            value = None
        elif isinstance(value, str) and issubclass(self.rcls, self._RTYPES):
            value = self.rcls.from_str(value)
        elif isinstance(value, Serial) and issubclass(self.rcls, self._RTYPES):
            value = self.rcls.read(value)
        else:
            raise ValueError(
                "response must be str, StatusResponse or DataResponse"
            )

        self._response = value
        return self.response

    @property
    def rtype(self):
        return parse_none(self._rtype, type(self._response))

    @property
    def rcls(self):
        return self.rtype

    @property
    def encoded(self):
        return self.encode()

    def encode(self):
        return str(self).encode()

    def write(self, serial: Serial):
        bytes_written = serial.write(self.encoded)
        if bytes_written != len(self.encoded):
            raise RuntimeError("Command was not fully written")
        self.log_datetime()
        serial.flush()  # wait to finish transmission

        self.response = serial
        return self.response


class SetPumpAddress(RequestMessage):
    _end = CR
    _rtype = StatusResponse

    def __init__(self, addr: int | uint = 1) -> None:
        assert 1 <= addr <= 8, f"Pump address must be in range 1–8; got {addr}"
        cls = type(self)
        return super().__init__(
            addr, end=cls._end, response=None, rtype=cls._rtype
        )

    def __repr__(self) -> str:
        return f"PumpAddressCommand({int(self.addr)})"

    def __str__(self) -> str:
        return f"{AT}{self.addr}{self.end}"


class DeviceCommand(RequestMessage):
    _code = GET_MODEL_INFO
    _end = CRLF
    _rtype = DataResponse
    BASE_CLASSES = {
        "DeviceCommand",
        "ShortDeviceCommand",
        "GetterCommand",
        "ShortGetterCommand",
        "SetterCommand",
        "ShortSetterCommand",
        "VerboseSetterCommand",
    }
    _types = None

    def __init__(self, addr, *args, code=None, rtype=None) -> None:
        self._args = self.convert_args(args)
        self._code = parse_none(code, self._code)
        rtype = parse_none(rtype, self._rtype)
        return super().__init__(
            addr, end=self._end, response=None, rtype=rtype
        )

    def __repr__(self) -> str:
        cls = type(self)
        name = cls.__name__
        addr = str(int(self.addr))
        code = repr(self.code)
        args = [addr, code] if name in cls.BASE_CLASSES else [addr]
        optargs = [repr(a) for a in self.args]
        return f"{name}({", ".join(args + optargs)})"

    def __str__(self) -> str:
        params = VB.join(str(a) for a in self.args)
        # [CRLF] required for termination
        return f"{self.addr}{self.code}{params}{self.end}"

    @property
    def args(self):
        return self._args

    @property
    def code(self):
        return self._code

    @property
    def types(self):
        return parse_none(self._types, tuple(type(a) for a in self._args))

    @classmethod  # automatically convert if types provided by subclass
    def convert_args(cls, *args):
        types = cls._types
        if isinstance(types, (tuple, list)) and len(args) == len(types):
            return tuple(t(arg) for (t, arg) in zip(types, args))
        else:
            return args


class ShortDeviceCommand(DeviceCommand):
    def __init__(self, addr=0, code=None, rtype=None) -> None:
        super().__init__(addr, code=code, rtype=rtype)


# GETTERS AND SETTERS
class GetterCommand(DeviceCommand):
    """A command that gets device information, returning a DataResponse"""

    _rtype = DataResponse


class ShortGetterCommand(ShortDeviceCommand):
    """A getter with no arguments, returning a DataResponse"""

    _rtype = DataResponse


class StatusGetterCommand(ShortDeviceCommand):
    """A getter with no arguments, returning a StatusResponse"""

    _rtype = StatusResponse


class SetterCommand(DeviceCommand):
    """A command that alters device settings, returning a StatusResponse"""

    _rtype = StatusResponse


class ShortSetterCommand(ShortDeviceCommand):
    """A setter with no arguments, returning a StatusResponse"""

    _rtype = StatusResponse


class VerboseSetterCommand(DeviceCommand):
    """A setter with arguments, returning a DataResponse"""

    _rtype = DataResponse


# 1.0 COMMUNICATIONS MANAGEMENT
class GetChannelAddressing(ShortGetterCommand):  # 1.1
    """Get an integer representing whether (1) or not (0) channel addressing is enabled."""

    _rtype = ChannelAddressingResponse
    _code = GET_CHN_ADR


class SetChannelAddressing(SetterCommand):  # 1.2
    """Set whether channel messaging is enabled (1) or not enabled (0)."""

    _types = (BooleanType,)
    _code = SET_CHN_ADR


class GetEventMessaging(ShortGetterCommand):  # 1.3
    """Get an integer representing whether (1) or not (0) event messages are enabled."""

    _rtype = EventMessagingResponse
    _code = GET_EVT_MSG


class SetEventMessaging(SetterCommand):  # 1.4
    """Set whether event messages are enabled (1) or not enabled (0)."""

    _types = (BooleanType,)
    _code = GET_EVT_MSG


class GetSerialVersion(ShortGetterCommand):  # 1.5
    """Get an integer representing the version of the serial protocol."""

    _rtype = SerialVersionResponse
    _code = GET_SERIAL_VER


# 2.0 PUMP DRIVE
class StartPump(ShortSetterCommand):  # 2.1
    """Start pump."""

    _code = SET_START


class StopPump(ShortSetterCommand):  # 2.2
    """Stop pumping."""

    _code = SET_STOP


class PausePump(ShortSetterCommand):  # 2.3
    """Pause pumping (STOP in RPM or flow rate mode)."""

    _code = SET_PAUSE


class GetDirection(ShortGetterCommand):  # 2.4
    """Get pump direction."""

    _rtype = DirectionResponse
    _code = GET_DIR


class SetClockwise(ShortSetterCommand):  # 2.5
    """Set rotation direction to clockwise."""

    _code = SET_CW


class SetCounterClockwise(ShortSetterCommand):  # 2.6
    """Set rotation direction to counter-clockwise."""

    _code = SET_CCW


class GetPumpError(ShortGetterCommand):  # 2.7
    """Cause of `–` cannot run response = Parameter #1, Limiting value that was exceeded = Parameter #2."""

    _rtype = PumpErrorResponse
    _code = GET_DRIVE_ERR


# 3.0 OPERATIONAL MODES AND SETTINGS
class GetMode(ShortGetterCommand):  # 3.1
    """Get the current channel or pump mode."""

    _rtype = ModeResponse
    _code = GET_MODE


class SetModeRPM(ShortSetterCommand):  # 3.2
    """Set pump/channel to RPM mode."""

    _code = SET_MODE_RPM


class SetModeFlowrate(ShortSetterCommand):  # 3.3
    """Set pump/channel to Flow Rate mode."""

    _code = SET_MODE_FLOWRATE


class SetModeVolumeRateFixed(ShortSetterCommand):  # 3.4
    """Set pump/channel to Volume (at rate) mode."""

    _code = SET_MODE_VOL_RATE


class SetModeVolumeTimeFixed(ShortSetterCommand):  # 3.5
    """Set pump/channel to Volume (over time) mode."""

    _code = SET_MODE_VOL_TIME


class SetModeVolumeRateCycled(ShortSetterCommand):  # 3.6
    """Set pump/channel to Volume + Pause mode."""

    _code = SET_MODE_VOL_PAUSE


class SetModeTimeRateFixed(ShortSetterCommand):  # 3.7
    """Set pump/channel to Time mode."""

    _code = SET_MODE_TIME


class SetModeTimeRateCycled(ShortSetterCommand):  # 3.8
    """Set pump/channel to Time + Pause mode."""

    _code = SET_MODE_RPM


class GetRateOther(GetterCommand):  # 3.9
    """Get flow rate from RPM (S) or flow rate (f) when mode is not RPM or flow rate."""

    _types = (BooleanType,)
    _rtype = RateOtherResponse
    _code = GET_RATE_OTHER


class SetRateOther(SetterCommand):  # 3.10
    """Set RPM flow rate not in RPM or flow rate mode Discrete Type 3."""

    _types = (DiscreteType3,)
    _code = SET_RATE_OTHER


class GetRPM(ShortGetterCommand):  # 3.11
    """Gets the current speed setting in RPM."""

    _rtype = RPMResponse
    _code = GET_RPM


class SetRPM(SetterCommand):  # 3.12
    """RPM mode flow rate setting (0.01 RPM) Discrete Type 3."""

    _types = (DiscreteType3,)
    _code = SET_RPM


class GetFlowrate(ShortGetterCommand):  # 3.13
    """Get current volume/time flow rate (mL/min)."""

    _rtype = FlowrateResponse
    _code = GET_FLOWRATE


class SetFlowrate(VerboseSetterCommand):  # 3.14
    """Set RPM flow rate in volume/time mode (mL/min) Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = FlowrateResponse
    _code = SET_FLOWRATE


class GetVolumeFixed(ShortGetterCommand):  # 3.15
    """Get the current setting for volume in mL."""

    _code = VolumeFixedResponse
    _code = GET_VOL


class SetVolumeFixed(VerboseSetterCommand):  # 3.16
    """Set the current setting for volume in mL. Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = VolumeFixedResponse
    _code = SET_VOL


class GetTimeFixed(ShortGetterCommand):  # 3.17
    """Get the current pump run time."""

    _rtype = TimeFixedResponse
    _code = GET_RUN_TIME


class SetTimeFixed(SetterCommand):  # 3.18
    """Set current pump run time using Time Type 2."""

    _types = (TimeType2,)
    _code = SET_RUN_TIME


class GetPauseTime(ShortGetterCommand):  # 3.19
    """Get pumping pause time."""

    _rtype = PauseTimeResponse
    _code = GET_PAUSE_TIME


class SetPauseTime(SetterCommand):  # 3.20
    """Set pumping pause time using Time Type 2."""

    _types = (TimeType2,)
    _code = SET_PAUSE_TIME


class GetCycleCount(ShortGetterCommand):  # 3.21
    """Get pump cycle count."""

    _rtype = CycleCountResponse
    _code = GET_CYCLE_COUNT


class SetCycleCount(SetterCommand):  # 3.22
    """Set pump cycle count Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_CYCLE_COUNT


class GetMaxFlowrate(ShortGetterCommand):  # 3.23
    """Max flow rate achievable with current settings mL/min."""

    # _rtype = MaxFlowRateResponse
    _code = GET_MAX_FLOWRATE


class GetMaxFlowrateCalibrated(ShortGetterCommand):  # 3.24
    """Max flow rate achievable with current settings using calibration."""

    # _rtype = MaxFlowRateCalibratedResponse
    _code = GET_MAX_FLOWRATE_CALIBRATED


class GetDispenseTimeByFlowrate(GetterCommand):  # 3.25
    """Get time to dispense at a given volume at a given mL/min flow rate.

    Vol, Volume Type 2; flow rate, Volume Type 2.
    """

    _types = (VolumeType2, VolumeType2)
    _rtype = DispenseTimeByFlowrateResponse
    _code = GET_DISPENSE_TIME_FLOWRATE


class GetDispenseTimeByRPM(GetterCommand):  # 3.26
    """Get time to dispense at a given volume at a given RPM.

    mL, Volume Type 2; flow rate, Discrete Type 3.
    """

    _types = (VolumeType2, DiscreteType3)
    _rtype = DispenseTimeByRPMResponse
    _code = GET_DISPENSE_TIME_RPM


# 4.0 CONFIGURATION
class GetTubeDiameter(ShortGetterCommand):  # 4.1
    """Get the current tubing inside diameter in mm. 2 decimal places are returned."""

    _rtype = TubeDiameterResponse
    _code = GET_DIAMETER


class SetTubeDiameter(SetterCommand):  # 4.2
    """Set tubing inside diameter using Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_DIAMETER


class GetBacksteps(ShortGetterCommand):  # 4.3
    """Get the current backsteps setting."""

    _rtype = BackstepsResponse
    _code = GET_BACKSTEP


class SetBacksteps(SetterCommand):  # 4.4
    """Set the current backsteps setting using Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_BACKSTEP


class ResetConfigData(ShortSetterCommand):  # 4.5
    """Resets all user configurable data to default values."""

    _code = RESET_CONFIG_DATA


# 5.0 CALIBRATION
class GetCalDirection(ShortGetterCommand):  # 5.1
    """Get direction flow for calibration."""

    _rtype = CalDirectionResponse
    _code = GET_CAL_DIR


class SetCalDirection(SetterCommand):  # 5.2
    """Set direction flow for calibration J or K using DIRECTION format."""

    _types = (DirectionType,)
    _code = SET_CAL_DIR


class GetCalTargetVolume(ShortGetterCommand):  # 5.3
    """Get the target volume to pump for calibrating, mL."""

    _rtype = CalTargetVolumeResponse
    _code = GET_TARGET_VOL


class SetCalTargetVolume(VerboseSetterCommand):  # 5.4
    """Set the target volume to pump for calibrating using Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = CalTargetVolumeResponse
    _code = SET_TARGET_VOL


class SetCalActualVolume(VerboseSetterCommand):  # 5.5
    """Set actual volume measured during calibration, mL Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = CalActualVolumeResponse
    _code = SET_ACTUAL_VOL


class GetCalTime(ShortGetterCommand):  # 5.6
    """Get the current calibration time."""

    _rtype = CalTimeResponse
    _code = GET_CAL_TIME


class SetCalTime(SetterCommand):  # 5.7
    """Set the current calibration time using Time Type 2."""

    _types = (TimeType2,)
    _code = SET_CAL_TIME


class GetRuntimeSinceCal(ShortGetterCommand):  # 5.8
    """Get the channel run time since last calibration."""

    _rtype = RuntimeSinceCalResponse
    _code = GET_RUNTIME_SINCE


class StartCalibration(ShortSetterCommand):  # 5.9
    """Start calibration on a channel(s)."""

    _code = SET_CAL_START


class CancelCalibration(ShortSetterCommand):  # 5.10
    """Cancel calibration."""

    _code = SET_CAL_CANCEL


# 6.0 SYSTEM
class GetFirmwareVersion(ShortGetterCommand):  # 6.1
    """Returns the pump firmware version."""

    _rtype = FirmwareVersionResponse
    _code = GET_FIRMWARE


class SetFactoryRollerVolume(SetterCommand):  # 6.2
    """Change factory roller step volume for a particular roller count and tubing size.

    - roller count (6, 8, 12), Discrete Type 1
    - index of the tubing diameter (see Table 1), Discrete Type 1
    - RSV, Volume Type 2.
    """

    _types = (DiscreteType1, DiscreteType1, VolumeType2)
    # NOTE: distinguish from SetRollerVolume?
    _code = SET_FACTORY_ROLLVOL


class SaveSetRollerVolume(ShortSetterCommand):  # 6.3
    """Save set roller step settings."""

    # NOTE: distinguish from SetRollerVolume?
    _code = SAVE_SET_ROLLVOL


class ResetRollerVolume(ShortSetterCommand):  # 6.4
    """Reset roller step volume table to defaults."""

    _code = RESET_ROLLVOL


class SetPumpDisplayName(SetterCommand):  # 6.5
    """Set pump name for display under remote control–String."""

    _types = (StringType,)
    _code = SET_PUMP_NAME


class GetPumpSerialNo(ShortGetterCommand):  # 6.6
    """Get pump serial number."""

    _rtype = PumpSerialNoResponse
    _code = GET_SERIALNO


class SetPumpSerialNo(SetterCommand):  # 6.7
    """Set pump serial number–String."""

    _types = (StringType,)
    _code = SET_SERIALNO


class GetPumpLanguage(ShortGetterCommand):  # 6.8
    """Get the current pump language."""

    _rtype = PumpLanguageResponse
    _code = GET_LANG


class SetPumpLanguage(SetterCommand):  # 6.9
    """Set current pump language–Language."""

    _types = (LanguageType,)
    _code = SET_LANG


class GetNumberOfChannels(ShortGetterCommand):  # 6.10
    """Get number of pump channels."""

    _rtype = NumberOfChannelsResponse
    _code = GET_CHANNELNO


class SetNumberOfChannels(SetterCommand):  # 6.11
    """Configure number of pump channels. Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_CHANNELNO


class GetNumberOfRollers(ShortGetterCommand):  # 6.12
    """Get number of rollers for channel."""

    _rtype = NumberOfRollersResponse
    _code = GET_ROLLERNO


class SetNumberOfRollers(SetterCommand):  # 6.13
    """Set number of rollers for channel. Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_ROLLERNO


class GetRevsSinceReset(ShortGetterCommand):  # 6.14
    """Get total number of revolutions since last reset."""

    _rtype = RevsSinceResetResponse
    _code = GET_REV_SINCE_RESET


class GetVolumeSinceReset(ShortGetterCommand):  # 6.15
    """Get channel total volume pumped since last reset, mL."""

    # NOTE: distinguish from GetTotalVolumeSinceReset?
    _rtype = VolumeSinceResetResponse
    _code = GET_VOL_SINCE_RESET


class GetTimePumpedSinceReset(ShortGetterCommand):  # 6.16
    """Get total time pumped for a channel since last reset."""

    _rtype = TimePumpedSinceResetResponse
    _code = GET_TIME_SINCE_RESET


class EnableGUI(ShortSetterCommand):  # 6.17
    """Set control from the pump user interface."""

    _code = SET_GUI_ON


class DiableGIU(ShortSetterCommand):  # 6.18
    """Disable pump user interface."""

    _code = SET_GUI_OFF


class SetDisplayNumbers(SetterCommand):  # 6.19
    """Write numbers to the pump to display while under external control.

    - String (<17 characters).
    """

    _types = (StringType,)
    _code = SET_DISPLAY_NUM


class SetDisplayCharacters(SetterCommand):  # 6.20
    """Write numbers to the pump to display while under external control.

    - String (<17 characters).
    """

    _types = (StringType,)
    _code = SET_DISPLAY_CHR


class GetRunStatus(StatusGetterCommand):  # 6.21
    """Returns whether the pump is currently running or not."""

    _code = GET_RUN_STATUS


class GetModelInfo(ShortGetterCommand):  # 6.22
    """Returns pump model information.

    Returns the following fields, each separated by a space:
    - Pump model description:
     A text field describing the model of pump.
     This description may contain spaces.
    - Pump software version:
     The version of software currently running in the pump.
    - Pump head model type code:
     A code describing the type of pump head installed.
     The first digit represents the number of channels for the pump, and the second 2 digits represent the number of rollers.
     XX if channels do not have the same number of rollers.
    """

    _rtype = ModelInfoResponse
    _code = GET_MODEL_INFO


class GetPumpHeadModel(ShortGetterCommand):  # 6.23
    """Returns the pump head model type code

    - A 4 digit code indicating the ID number of the pump head.
    The first two digits represent the number of channels on the head,
    and the second 2 digits represent the number of rollers.
    """

    _rtype = PumpHeadModelResponse
    _code = GET_HEAD_CODE


class SetPumpHeadModel(SetterCommand):  # 6.24
    """Sets the pump head model type code

    – An up-to 4 digit code setting the ID number of the pump head.
     The first two digits encode the number of channels on the head,
     the second two digits encode the number of rollers on the head.

    This command sets all roller counts to the same value.
    To individually set roller counts for each channel,
    use the non-legacy command designed for this operation.

    Discrete Type 2.
    """

    _types = (DiscreteType2,)
    _code = SET_HEAD_CODE


class GetPumpTime10ths(ShortGetterCommand):  # 6.25
    """Get the current setting for pump time in 1/10 second."""

    # NOTE: distinguish from GetTimeFixed?
    _rtype = PumpTime10thsResponse
    _code = GET_TIME_10THS


class SetPumpTime10ths(SetterCommand):  # 6.26
    """Set the current setting for pump time in 1/10 second. Discrete Type 2."""

    # NOTE: distinguish from SetTimeFixed?
    _types = (DiscreteType2,)
    _code = SET_TIME_10THS


class SetDispenseTimeMins(SetterCommand):  # 6.27
    """Set the current run time setting for dispensing in minutes. Discrete Type 5."""

    # NOTE: distinguish from SetTimeFixed?
    _types = (DiscreteType5,)
    _code = SET_TIME_MINS


class SetDispenseTimeHrs(SetterCommand):  # 6.28
    """Set the current run time setting for dispensing in hours. Discrete Type 5."""

    # NOTE: distinguish from SetTimeFixed?
    _types = (DiscreteType5,)
    _code = SET_TIME_HRS


class GetLowOrderSteps(ShortGetterCommand):  # 6.29
    """Get the low order roller steps. The total number of roller steps which
    are dispensed during an operation is computed as:[(u*65536]+(U)]."""

    _rtype = LowOrderStepsResponse
    _code = GET_ROLLSTEP_LO


class SetLowOrderSteps(SetterCommand):  # 6.30
    """Set the high order roller steps. Discrete Type 6."""

    _types = (DiscreteType6,)
    _code = SET_ROLLSTEP_LO


class GetHighOrderSteps(ShortGetterCommand):  # 6.31
    """Get the high order roller steps. The total number of roller steps which
    are dispensed during an operation is computed as:[(u*65536]+(U)]."""

    _rtype = HighOrderStepsResponse
    _code = GET_ROLLSTEP_HI


class SetHighOrderSteps(SetterCommand):  # 6.32
    """Set the high order roller steps. Discrete Type 6."""

    _types = (DiscreteType6,)
    _code = SET_ROLLSTEP_HI


class GetRollerVolume(ShortGetterCommand):  # 6.33
    """Get the current roller step volume

    Based on the current calibration, tubing diameter and roller count.
    If no calibration has been performed the default volume is returned.
    """

    _rtype = RollerVolumeResponse
    _code = GET_ROLLVOL_CAL


class SetRollerVolume(SetterCommand):  # 6.34
    """Set the calibrated roller step volume to use for this pump or channel.

    This value is used as the calibrated value and is overwritten by
    subsequent calibrations and reset by changing tubing diameter.
    Volume Type 2.
    """

    # NOTE: distinguish from SetFactoryRollerVolume
    _types = (VolumeType2,)
    _code = SET_ROLLVOL_CAL


class ResetCalibrationData(ShortSetterCommand):  # 6.35
    """Reset the pump to discard calibration data, use default roller step volume."""

    # NOTE: distinguish from ResetConfigData
    _code = RESET_CAL


class GetPauseTime10ths(ShortGetterCommand):  # 6.36
    """Get the current setting for pause time in 1/10 second."""

    # NOTE: distinguish from GetPauseTime?
    _rtype = PauseTime10thsResponse
    _code = GET_PAUSE_10THS


class SetPauseTime10ths(SetterCommand):  # 6.37
    """Set the current setting for pause time in 1/10 second. Discrete Type 2."""

    # NOTE: distinguish from SetPauseTime?
    _types = (DiscreteType2,)
    _code = SET_PAUSE_10THS


class SetPauseTimeMins(SetterCommand):  # 6.38
    """Set the current setting for pause time in minutes. Discrete Type 5."""

    # NOTE: distinguish from SetPauseTime?
    _types = (DiscreteType5,)
    _code = SET_PAUSE_MINS


class SetPauseTimeHrs(SetterCommand):  # 6.39
    """Set the current setting for pause time in hours. Discrete Type 5."""

    # NOTE: distinguish from SetPauseTime?
    _types = (DiscreteType5,)
    _code = SET_PAUSE_HRS


class GetTotalVolumeSinceReset(GetterCommand):  # 6.40
    """Get the total volume dispensed since the last reset in µL, mL or liters."""

    # NOTE: distinguish from GetVolumeSinceReset?
    # _types = ()
    # _rtype = TotalVolumeSinceResetResponse
    _code = GET_VOLALL_SINCE_RESET


class SavePumpSettings(ShortSetterCommand):  # 6.41
    """Saves the current pump settings values to memory."""

    _code = SAVE_SET_ALL


class GetFootSwitchStatus(StatusGetterCommand):  # 6.42
    """Get the current state of the foot switch."""

    _code = GET_FOOTSWITCH_STATUS


class RegloChannel:
    _serial = None

    def __init__(
        self,
        addr: int = 1,
        serial: None | str | Serial = _serial,
        tube_id=None | float,
    ) -> None:
        # Open serial port with data format corresponding to the RegloICC pump
        self.addr = addr
        self.serial = serial
        self._log = DataFrame(
            columns=[
                "datetime",
                "request",
                "response",
                "request_string",
                "response_string",
            ]
        )
        self._current_request = None

        pass

    def __repr__(self) -> str:
        return f"RegloChannel({self.addr, self.serial})"

    @property
    def serial(self):
        return self._serial

    @serial.setter
    def serial(self, value):
        if isinstance(value, Serial):
            serial = value
        elif isinstance(value, (str, type(None))):
            serial = Serial(
                get_port(value),
                9600,
                parity=PARITY_NONE,
                stopbits=STOPBITS_ONE,
                bytesize=EIGHTBITS,
            )

        self._serial = serial
        return self._serial

    @property
    def current_request(self):
        return self._current_request

    @current_request.setter
    def current_request(self, value: RequestMessage):
        self.update_log(value)
        self._current_request = value
        return self.current_request

    @property
    def current_response(self):
        if self.current_request is None:
            return None
        return self.current_request.response

    @property
    def log(self):
        return self._log

    def update_log(self, request: RequestMessage):
        row_new = dict(
            datetime=request.datetime,
            request=request,
            response=request.response,
            request_string=str(request),
            response_string=str(request.response),
        )
        self._log = pdcat([self._log, DataFrame(row_new)], ignore_index=True)
        return self.log

    def send_request(
        self,
        request: RequestMessage | type[RequestMessage],
        *args,
        addr=None,
        **kwargs,
    ):
        """Send a new request to the pump and read the response."""
        addr = parse_none(addr, self.addr)

        if isinstance(request, type) and issubclass(request, RequestMessage):
            request = request(addr, *args, **kwargs)
            return self.send_request(request)

        if self.serial is None:
            return None

        request(self.serial)
        self.current_request = request
        return request.response

    # 2.0 PUMP DRIVE
    def start(self):
        return self.send_request(StartPump)

    def stop(self):
        return self.send_request(StopPump)

    def pause(self):
        return self.send_request(PausePump)

    @property
    def direction(self):
        return self.send_request(GetDirection)

    @direction.setter
    def direction(self, value):
        if DIR_KEY[value] == CW:
            return self.send_request(SetClockwise)
        elif DIR_KEY[value] == CCW:
            return self.send_request(SetCounterClockwise)

    @property
    def error(self):
        return self.send_request(GetPumpError)

    # 3.0 OPERATIONAL MODES AND SETTINGS
    @property
    def mode(self):
        return self.send_request(GetMode)

    @mode.setter
    def mode(self, value):
        if value == MODE_RPM:
            return self.send_request(SetModeRPM)
        elif value == MODE_FLOWRATE:
            return self.send_request(SetModeFlowrate)
        elif value == MODE_VOL_RATE:
            return self.send_request(SetModeVolumeRateFixed)
        elif value == MODE_VOL_TIME:
            return self.send_request(SetModeVolumeTimeFixed)
        elif value == MODE_VOL_PAUSE:
            return self.send_request(SetModeVolumeRateCycled)
        elif value == MODE_TIME:
            return self.send_request(SetModeTimeRateFixed)
        elif value == MODE_TIME_PAUSE:
            return self.send_request(SetModeTimeRateCycled)
        else:
            return None

    @property
    def rate_other(self):
        return self.send_request(GetRateOther)

    @rate_other.setter
    def rate_other(self, value):
        return self.send_request(SetRateOther, value)

    @property
    def rpm(self):
        return self.send_request(GetRPM)

    @rpm.setter
    def rpm(self, value):
        return self.send_request(SetRPM, value)

    @property
    def flowrate(self):
        return self.send_request(GetFlowrate)

    @flowrate.setter
    def flowrate(self, value):
        return self.send_request(SetFlowrate, value)

    @property
    def volume_fixed(self):
        return self.send_request(GetVolumeFixed)

    @volume_fixed.setter
    def volume_fixed(self, value):
        return self.send_request(SetVolumeFixed, value)

    @property
    def time_fixed(self):
        return self.send_request(GetTimeFixed)

    @time_fixed.setter
    def time_fixed(self, value):
        return self.send_request(SetTimeFixed, value)

    @property
    def cycle_pause(self):
        return self.send_request(GetPauseTime)

    @cycle_pause.setter
    def cycle_pause(self, value):
        return self.send_request(SetPauseTime, value)

    @property
    def cycle_count(self):
        return self.send_request(GetCycleCount)

    @cycle_count.setter
    def cycle_count(self, value):
        return self.send_request(SetCycleCount, value)

    @property
    def maxflowrate(self):
        return self.send_request(GetMaxFlowrate)

    @property
    def maxflowrate_calibrated(self):
        return self.send_request(GetMaxFlowrateCalibrated)

    def flowrate2dispense(self, volume, flowrate):
        return self.send_request(GetDispenseTimeByFlowrate, volume, flowrate)

    def rpm2dispense(self, volume, rpm):
        return self.send_request(GetDispenseTimeByFlowrate, volume, rpm)

    # 4.0 CONFIGURATION
    @property
    def tube_diameter(self):
        return self.send_request(GetTubeDiameter)

    @tube_diameter.setter
    def tube_diameter(self, value):
        return self.send_request(SetTubeDiameter, value)

    @property
    def backsteps(self):
        return self.send_request(GetBacksteps)

    @backsteps.setter
    def backsteps(self, value):
        return self.send_request(SetBacksteps, value)

    def reset(self):
        return self.send_request(ResetConfigData)

    # 5.0 CALIBRATION
    @property
    def cal_direction(self):
        return self.send_request(GetCalDirection)

    @cal_direction.setter
    def cal_direction(self, value):
        return self.send_request(SetCalDirection, value)

    @property
    def cal_targetvol(self):
        return self.send_request(GetCalTargetVolume)

    @cal_targetvol.setter
    def cal_targetvol(self, value):
        return self.send_request(SetCalTargetVolume, value)

    @property
    def cal_actualvol(self):
        return getattr(self, "_cal_actualvol", None)

    @cal_actualvol.setter
    def cal_actualvol(self, value):
        self._cal_actualvol = self.send_request(SetCalActualVolume, value)
        return self.cal_actualvol

    @property
    def cal_time(self):
        return self.send_request(GetCalTime)

    @cal_time.setter
    def cal_time(self, value):
        return self.send_request(SetCalTime, value)

    @property
    def runtime_since_cal(self):
        return self.send_request(GetRuntimeSinceCal)

    def start_calibration(self):
        return self.send_request(StartCalibration)

    def cancel_calibration(self):
        return self.send_request(CancelCalibration)

    # 6.0 SYSTEM
    @property
    def firmware_version(self):
        return self.send_request(GetFirmwareVersion)

    # @property
    # def factory_rollervol(self):
    #     return self.send_request(SetFactoryRollerVolume)

    @property
    def num_rollers(self):
        return self.send_request(GetNumberOfRollers)

    @num_rollers.setter
    def num_rollers(self, value):
        return self.send_request(SetNumberOfRollers, value)

    @property
    def revs_since_reset(self):
        return self.send_request(GetRevsSinceReset)

    @property
    def vol_since_reset(self):
        return self.send_request(GetVolumeSinceReset)

    @property
    def pumptime_since_reset(self):
        return self.send_request(GetTimePumpedSinceReset)

    @property
    def pumptime(self):
        return self.send_request(GetPumpTime10ths)

    @pumptime.setter
    def pumptime(self, value: int | tuple[int, str | None]):
        return self._set_time_in_units(
            SetPumpTime10ths, SetDispenseTimeMins, SetDispenseTimeHrs, value
        )

    @property
    def loworder_steps(self):
        return self.send_request(GetLowOrderSteps)

    @loworder_steps.setter
    def loworder_steps(self, value):
        return self.send_request(SetLowOrderSteps, value)

    @property
    def highorder_steps(self):
        return self.send_request(GetHighOrderSteps)

    @highorder_steps.setter
    def highorder_steps(self, value):
        return self.send_request(SetHighOrderSteps, value)

    @property
    def rollervol(self):
        return self.send_request(GetRollerVolume)

    @rollervol.setter
    def rollervol(self, value):
        return self.send_request(SetRollerVolume, value)

    def reset_calibration(self):
        return self.send_request(ResetCalibrationData)

    @property
    def pausetime(self):
        return self.send_request(GetPauseTime10ths)

    @pausetime.setter
    def pausetime(self, value: int | tuple[int, str | None]):
        return self._set_time_in_units(
            SetPauseTime10ths, SetPauseTimeMins, SetPauseTimeHrs, value
        )

    @property
    def totalvol_since_reset(self):
        return self.send_request(GetTotalVolumeSinceReset)

    def save_pump_settings(self):
        return self.send_request(SavePumpSettings)

    @property
    def footswitch_status(self):
        return self.send_request(GetFootSwitchStatus)

    # HELPER
    def _set_time_in_units(self, settenths, setmins, sethrs, value):
        value, unit = (value, None) if isinstance(value, int) else value
        if unit in ("10ths", "10ths", "1/10", "0.1", "tenth", "tenths", None):
            return self.send_request(settenths, value)
        elif unit.lower in ("min", "mins", "minute", "minutes", "m"):
            return self.send_request(setmins, value)
        elif unit.lower in ("hr", "hrs", "hour", "hours", "h"):
            return self.send_request(sethrs, value)


class RegloICC:
    """RegloICC pump control.

    Change the size of 'direction', 'mode' and 'speed' according to the total
    number of channels to control.
    In the case herein, 3 channels are being independently controlled.
    """

    def __init__(self, com=None, active=False, direction=0, mode=1, speed=0):
        """Initialize the pump and set up the serial connection."""
        self.com = get_port(com)
        # Open serial port with data format corresponding to the RegloICC pump
        self.serial = Serial(
            self.com,
            9600,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            bytesize=EIGHTBITS,
        )
        print(f"Connected to pump on {self.com}")

        self.active = self.parse_param(active)
        """Defines which channels are active."""

        self.direction = self.parse_param(direction)
        """Defines pump direction for each channel.

        0 = clockwise, 1 = counter-clockwise
        """

        self.mode = self.parse_param(mode)
        """Defines pump-speed mode for each channel.

        0 = rpm, 1 = flowrate, 2 = volume (over time).

        One can add here all other modes.
        """

        self.speed = self.parse_param(speed)
        """Define pump speed for each channel (relative to mode)."""

        self.set_all()

    def __del__(self):
        """Delete the pump."""
        self.serial.close()

    def write_receive(self, command):
        """Write a command to the pump and read the response."""
        self.serial.write(command.encode())  # send to serial
        time.sleep(0.1)  # time to process
        response = self.serial.read(self.serial.in_waiting).decode()
        time.sleep(0.1)  # time to process
        print(f"Sent: {command.strip()}, Response: {response.strip()}")
        return response

    def fwrite_receive(self, channel, inner_command):
        """Format & write a command to the pump and read the response."""
        # \r for the carriage return [CR] required
        # to tell the pump that the command is finished
        return self.write_receive(f"{channel}{inner_command}\r")

    def parse_param(self, param):
        """Parse a channel parameter."""
        return [param] * 3 if not isinstance(param, list) else param

    def set_all(
        self,
        active=None,
        direction=None,
        mode=None,
        speed=None,
        from_unit=None,
    ):
        """Set direction, mode and speed for all channels."""
        pars = (self.parse_param(p) for p in (active, direction, mode, speed))
        for channel, a, d, m, s in zip(range(1, 4), *pars):
            self.set_channel(channel, a, d, m, s, from_unit)
        pass

    def set_channel(
        self,
        channel,
        active=None,
        direction=None,
        mode=None,
        speed=None,
        from_unit=None,
    ):
        self.set_direction(channel, direction)
        self.set_mode(channel, mode)
        self.set_speed(channel, speed, from_unit=from_unit)
        self.set_active(channel, active)
        pass

    def set_active(self, channel, active=None):
        """Activate or deactivate a channel."""
        if active is not None:
            self.active[channel - 1] = active
        else:
            active = self.active[channel - 1]

        return (
            self.start_channel(channel)
            if active
            else self.stop_channel(channel)
        )

    def start_channel(self, channel):
        """Start the corresponding channel."""
        self.active[channel - 1] = True
        # 'H' to start the channel
        return self.fwrite_receive(channel, "H")

    def stop_channel(self, channel):
        """Stop the corresponding channel."""
        self.active[channel - 1] = False
        # 'I' to stop the channel
        return self.fwrite_receive(channel, "I")

    def set_direction(self, channel, direction=None):
        """Set the rotation direction for a single channel."""
        if direction is None:
            direction = self.direction[channel - 1]
        else:
            self.direction[channel - 1] = direction

        # K: counter-clockwise, J: clockwise
        d = {0: "J", 1: "K"}.get(direction, "J")
        return self.fwrite_receive(channel, d)

    def get_direction(self, channel):
        """Get the rotation direction of a single channel."""
        # 'xD' to get the rotation direction
        return self.fwrite_receive(channel, "xD")

    def set_speed(self, channel, *args, from_unit=None):
        """Set the speed for a single channel in the current mode."""
        m = self.mode[channel - 1]
        if m == 0:  # RPM mode
            return self.set_rpm(channel, *args)
        elif m == 1:  # Flow rate mode
            return self.set_flowrate(channel, *args, from_unit=from_unit)
        else:
            raise ValueError(
                "Unsupported mode. Use 0 for RPM or 1 for Flow Rate."
            )

    def set_rpm(self, channel, rpm=None):
        """Set the speed for a single channel in RPM when in RPM mode."""
        if rpm is None:
            rpm = self.speed[channel - 1]
        else:
            self.speed[channel - 1] = rpm

        # in RPM, with speed  Result: '123'
        # Convert the decimal part to a string with two digits
        fixed_speed = int(round(rpm * 100))
        return self.fwrite_receive(channel, f"S{fixed_speed:06d}")

    def get_rpm(self, channel):
        """Read out speed of a single channel in RPM when in RPM mode."""
        # 'S' to get the setting speed in RPM
        return self.fwrite_receive(channel, "S")

    def set_flowrate(self, channel, flowrate=None, s="-", e=3, from_unit=None):
        r"""Set flow rate in mL/min.

        RegloICC uses the format string `mmmmse` to encode the flow rate,
        which translates to :math:`m.mmm \times 10^{se} mL/min`.

        :type channel: int
        :param channel:
        :type flowrate:
        :param flowrate:
            Four-digit integer `mmmm`.
            Represents the 4 sf of the floating point number `m.mmm`.
        :type s: str
        :param s:
            Sign of exponent `"+"` or `"-"`.
        :type e: int
        :param e:
            Exponent.
        :type from_unit: str
        :param from_unit:
            If specified, `flowrate` is converted to mL/min.
            Format as
             `"vol/time"` where `vol = {mL, uL}`, `time = {hr, min, s}`.

        Note
        ----
        When the two strings are concatenated, the result is a string that
         represents the original speed value in a fixed-point notation with
         three digits before the decimal point and two digits after, without
         the decimal separator.
        """
        if flowrate is None:
            flowrate = self.speed[channel - 1]
        else:
            self.speed[channel - 1] = flowrate

        if from_unit is not None:
            mmmmse = flux2mmmmse(flowrate, from_unit=from_unit)
            return self.set_flowrate(channel, *mmmmse, from_unit=None)

        cmd = f"f{flowrate:04d}{s}{e}"
        # print(cmd)
        return self.fwrite_receive(channel, cmd)

    def get_flowrate(self, channel):
        return self.fwrite_receive(channel, "f")

    def set_mode(self, channel, mode=None):
        """Set the operational mode for a single channel."""
        # NOTE: you can add all other modes.
        if mode is None:
            mode = self.mode[channel - 1]
        else:
            self.mode[channel - 1] = mode

        # L = rpm, M = flowrate, G = volume (over time)
        m = {0: "L", 1: "M", 2: "G"}.get(mode, "L")
        return self.fwrite_receive(channel, m)

    def get_mode(self, channel):
        """Get the operational mode of a single channel."""
        # 'xM' to get the operational mode
        return self.fwrite_receive(channel, "xM")


def flux2mmmmse(flowrate, from_unit="mL/min"):
    """Convert flowrate (flux) from vol/time to mmmmse format.

    :param flowrate:
    :param from_unit:

    :rtype: tuple[int, str, int]
    :returns:
        Flow rate tuple `(mmmm, "-", exponent)`.
    """
    v_unit, t_unit = from_unit.split("/", 1)
    time_factors = {"s": 1 / 60, "min": 1, "hr": 60, "hour": 60}
    vol_factors = {"uL": 1e-3, "µL": 1e-3, "mL": 1}
    if t_unit not in time_factors:
        raise ValueError("Invalid time unit. Use 's', min' or 'hr'.")
    if v_unit not in vol_factors:
        raise ValueError("Invalid volume unit. Use 'uL' or 'mL'.")
    flow_mL_min = flowrate * vol_factors[v_unit] / time_factors[t_unit]

    s = sign(flow_mL_min - 1)
    e = 0
    flow_scaled = flow_mL_min * 10 ** (-s * e)
    while not (1 <= flow_scaled < 10) and flow_mL_min != 0:
        e += 1
        flow_scaled = flow_mL_min * 10 ** (-s * e)
    flow_scaled *= 1000
    mmmm = int(min(round(flow_scaled), 9999))

    return (mmmm, "-" if s == -1 else "+", e)
