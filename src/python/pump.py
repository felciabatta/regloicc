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

from numpy import clip, format_float_scientific, sign, uint
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
MODE_RATE = "M"
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
SET_MODE_RATE = MODE_RATE  # 3.3
SET_MODE_VOL_RATE = MODE_VOL_RATE  # 3.4
SET_MODE_VOL_TIME = MODE_VOL_TIME  # 3.5
SET_MODE_VOL_PAUSE = MODE_VOL_PAUSE  # 3.6
SET_MODE_TIME = MODE_TIME  # 3.7
SET_MODE_TIME_PAUSE = MODE_TIME_PAUSE  # 3.8
GET_RATE_OTHER = "xf"  # 3.9
SET_RATE_OTHER = GET_RATE_OTHER  # 3.10
GET_RPM = "S"  # 3.11
SET_RPM = GET_RPM  # 3.12
GET_RATE = "f"  # 3.13
SET_RATE = GET_RATE  # 3.14
GET_VOL = "v"  # 3.15
SET_VOL = GET_VOL  # 3.16
GET_RUN_TIME = "xT"  # 3.17
SET_RUN_TIME = GET_RUN_TIME  # 3.18
GET_PAUSE_TIME = "xP"  # 3.19
SET_PAUSE_TIME = GET_PAUSE_TIME  # 3.20
GET_CYCLE_COUNT = r"\""  # 3.21
SET_CYCLE_COUNT = GET_CYCLE_COUNT  # 3.22
GET_MAX_RATE = "?"  # 3.23
GET_MAX_RATE_CALIBRATED = "!"  # 3.24
GET_DISPENSE_TIME_RATE = "xv"  # 3.25
GET_DISPENSE_TIME_RPM = "xw"  # 3.26

# 4.0 CONFIGURATION
GET_DIAMETER = "+"  # 4.1
SET_DIAMETER = GET_DIAMETER  # 4.2
GET_BACKSTEP = "%"  # 4.3
SET_BACKSTEP = GET_BACKSTEP  # 4.4
RESET_ALL_DATA = "0"  # 4.5

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
GET_FS_STATUS = "C"  # 6.42

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
    STOPPED_PUMP_COMPLETE: "Pumping complete (expiration of timer or volume limit reached).",
    STOPPED_CALI_COMPLETE: "Calibration pumping complete.",
    STOPPED_MANUAL: "Manual (emergency) stop event (user presses stop button on the pump).",
    STOPPED_OVERHEAT: "Over temperature condition.",
    STOPPED_OVERCURRENT: "Over current condition.",
}
CSTATUS_DESC_KEY = {
    CSTATUS_PUMPING: "Pumping",
    CSTATUS_PAUSED_BTWN_CYCLES: "Paused between cycles",
    CSTATUS_STOPPED: "Stopped",
    CSTATUS_CALI_PUMPING: "Calibration pumping",
    CSTATUS_CALI_PENDING: "Calibration pending (waiting for measured value to be entered)",
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

    def __init__(self, value=False) -> None:
        super().__init__(bool(value), width=uint(1))

    def __str__(self) -> str:
        # print("Executing BooleanType.__str__()")
        return f"{self._value:01d}"

    def __repr__(self) -> str:
        return f"BooleanType({self._value})"


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

    def __init__(self, value=0) -> None:
        value = round(float(value), 2)
        len_str = len(self.val2str(value))
        super().__init__(value, width=len_str)

    def val2str(self, x):
        return f"{x:.2f}"

    def __str__(self) -> str:
        return self.val2str(self._value)

    def __repr__(self) -> str:
        return f"FractionalType{self._value}"


class VolumeType(RegloDataType):
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

    def __init__(self, value=float()) -> None:
        value = 0 if value == 0 else clip(value, VOLTYPE_MIN, VOLTYPE_MAX)
        super().__init__(value, width=7)

    def float2sci(self, x):
        s = format_float_scientific(x, precision=3, min_digits=3, exp_digits=1)
        return s.upper()

    def __str__(self) -> str:
        return self.float2sci(self._value).replace(".", "").upper()

    def __repr__(self) -> str:
        return f"VolumeType({self.float2sci(self._value)})"


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


class DiamaterType(RegloDataType):
    def __init__(self, value: float, unit="mm") -> None:
        value = round(clip(value, 0.0, None), 2)
        self.unit = unit
        super().__init__(value, width=7)

    def __str__(self) -> str:
        return f"{self._value:.2f} {self.unit}"

    def __repr__(self) -> str:
        return f"DiameterType({self._value:.2f}, unit={self.unit!r})"


class DeviceMessage:
    def __init__(self) -> None:
        pass

    def __repr__(self) -> str:
        return "DeviceMessage()"


class StatusResponse(DeviceMessage):
    def __init__(self, status: str, desc="") -> None:
        assert len(status) == 1, "StatusResponse must be a single character."
        self.status = status
        self.desc = desc
        pass

    def __repr__(self) -> str:
        name = type(self).__name__
        args = f"{self!r}" if (name == StatusResponse.__name__) else ""
        return f"{name}({args})"

    def __str__(self) -> str:
        return self.status

    @classmethod
    def from_str(cls, s: str):
        match s:
            case r if r == SUCCESSFULL:
                return StatusSuccessfull()
            case r if r == UNSUCCESSFULL:
                return StatusUnsuccessfull()
            case r if r == NEGATIVE:
                return StatusNegative()
            case r if r == POSITIVE:
                return StatusPositive()
            case _:
                return StatusUnknown(s)

    @classmethod
    def read(cls, serial: Serial):
        s = serial.read(1).decode()
        assert s.isascii(), "Encountered non-ASCII response."
        assert len(s) == 1, "Encountered multi-character response."
        return cls.from_str(s)


class StatusUnknown(StatusResponse):
    def __init__(self, status: str) -> None:
        super().__init__(status, desc="Unknown status.")


class StatusSuccessfull(StatusResponse):
    def __init__(self) -> None:
        stat = SUCCESSFULL
        desc = STATUS_DESC_KEY[stat]
        return super().__init__(stat, desc)


class StatusUnsuccessfull(StatusResponse):
    def __init__(self) -> None:
        stat = UNSUCCESSFULL
        desc = STATUS_DESC_KEY[stat]
        return super().__init__(stat, desc)


class StatusNegative(StatusResponse):
    def __init__(self, desc=None) -> None:
        stat = NEGATIVE
        desc = parse_none(desc, STATUS_DESC_KEY[stat])
        return super().__init__(stat, desc)


class StatusPositive(StatusResponse):
    def __init__(self, desc=None) -> None:
        stat = POSITIVE
        desc = parse_none(desc, STATUS_DESC_KEY[stat])
        return super().__init__(stat, desc)


class DataMessage(DeviceMessage):
    # DEFAULTS
    BEGIN = ""
    SEP = SP
    END = CRLF

    def __init__(self, *args, begin=BEGIN, sep=SEP, end=END) -> None:
        self._args = args
        self._begin = begin
        self._sep = sep
        self._end = end
        pass

    @property
    def args(self):
        return self._args

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
    def parse(cls, s, begin: int | str, sep: str, end: int | str):
        """Parse message string and return `cls_args`, `cls_kwargs`"""
        begin = s[:begin] if isinstance(begin, int) else begin
        end = s[-end:] if isinstance(end, int) else end
        str_data = s.rstrip(end).lstrip(begin)
        cls_kwargs = dict(begin=begin, sep=sep, end=end)
        cls_args = str_data.split(sep)
        return cls_args, cls_kwargs

    @classmethod  # NOTE: no need to specialise
    def from_str(cls, s: str, *args, **kwargs):
        """Parse message string and convert to a DataMessage type"""
        cls_args, cls_kwargs = cls.parse(s, *args, **kwargs)
        return cls(*cls_args, **cls_kwargs)

    @classmethod
    def from_read(cls, serial: Serial, expected=None, *args, **kwargs):
        """Read serial message and convert to a DataMessage type"""
        END = kwargs["end"] if "end" in kwargs else cls.END
        expected = parse_none(expected, END)
        s = serial.read_until(expected.encode("ascii")).decode()
        return cls.from_str(s, *args, **kwargs)


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

    def __init__(self, *args) -> None:
        kwargs = dict(begin=self.begin, sep=self.sep, end=self.end)
        return super().__init__(*args, **kwargs)

    def __repr__(self) -> str:
        optargs = [repr(a) for a in self.args]
        return f"{type(self).__name__}({", ".join(optargs)})"

    @classmethod
    def parse(cls, s):
        cls_args, _ = super().parse(s, cls._begin, cls._sep, cls._end)
        return cls_args, dict()

    @classmethod
    def from_read(cls, serial: Serial):
        return super().from_read(serial, cls._end)


class EventMessage(DataMessage):
    _prefix = EVT_ID
    _begin = 2
    _sep = VB
    _end = CRLF
    _expected = _end

    def __init__(self, code, *args) -> None:
        self._code = code
        cls = type(self)
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
        args = [repr(self.code)]
        optargs = [repr(a) for a in self.args]
        return f"EventMessage({", ".join(args + optargs)})"

    @classmethod
    def parse(cls, s):
        cls_args, cls_kwargs = super().parse(
            s, begin=cls._begin, sep=cls._sep, end=cls._end
        )
        code = cls_kwargs["begin"][-1]
        return (code, *cls_args), dict()

    @classmethod
    def from_read(cls, serial: Serial):
        return super().from_read(serial, cls._expected)


class RequestMessage:
    def __init__(
        self,
        addr: int | uint = 0,
        end=CRLF,
        response: None | StatusResponse | DataResponse = None,
    ) -> None:
        assert 0 <= addr <= 8, f"Address must be in range 0–8; got {addr}"
        self.addr = uint(addr)
        self.end = end
        self.response = response
        pass

    def write_to(self, serial: Serial):
        return serial.write(str(self).encode())

    def __call__(self, serial: Serial):
        return self.write_to(serial)

    def __str__(self) -> str:
        return self.end

    def __repr__(self) -> str:
        return f"RequestMessage({int(self.addr)}, end={self.end!r})"


class PumpAddressCommand(RequestMessage):
    def __init__(
        self, addr: int | uint = 1, response: None | StatusResponse = None
    ) -> None:
        assert 1 <= addr <= 8, f"Pump address must be in range 1–8; got {addr}"
        return super().__init__(addr, end=CR, response=response)

    def __str__(self) -> str:
        return f"{AT}{self.addr}{self.end}"

    def __repr__(self) -> str:
        return f"PumpAddressCommand({int(self.addr)})"


class DeviceCommand(RequestMessage):
    def __init__(self, addr, code, *args) -> None:
        self.code = code
        self.args = args
        return super().__init__(addr, end=CRLF)

    def __str__(self) -> str:
        params = VB.join(str(a) for a in self.args)
        # [CRLF] required for termination
        return f"{self.addr}{self.code}{params}{self.end}"

    def __repr__(self) -> str:
        args = [str(int(self.addr)), repr(self.code)]
        optargs = [repr(a) for a in self.args]
        return f"DeviceCommand({", ".join(args + optargs)})"


class RegloICCChannel:
    def __init__(
        self, addr: int = 1, serial: None | str | Serial = None
    ) -> None:
        # Open serial port with data format corresponding to the RegloICC pump
        self.addr = addr
        self.serial = self.find_serial(serial)
        pass

    def request(self, code, *args, addr=None, receive=False):
        """Send a request to the pump and (optionally) read the response."""
        addr = parse_none(addr, self.addr)
        variables = VB.join(str(a) for a in args)
        # [CR] required for termination
        command = f"{addr}{code}{variables}{CRLF}"
        self.serial.write(command.encode())
        print(f"[RegloICC] Requested: {command}")
        if receive:
            time.sleep(0.1)  # process time
            return command, self.receive()
        return command

    def receive(self):
        nbytes = self.serial.in_waiting
        response = self.serial.read(nbytes).decode().strip()
        time.sleep(0.1)  # time to process
        print(f"[RegloICC] Response: {response}")
        return response

    def find_serial(self, serial):
        if isinstance(serial, Serial):
            return serial
        return Serial(
            get_port(serial),
            9600,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            bytesize=EIGHTBITS,
        )


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
