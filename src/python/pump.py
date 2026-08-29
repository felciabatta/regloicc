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

# COMMUNICATIONS MANAGEMENT
CHN_ADR = "~"
EVT_MSG = "xE"
GET_VERSION = "x!"

# PUMP DRIVE
START = "H"
STOP = "I"
PAUSE = "xI"
GET_DIR = "xD"
CW = "J"
CCW = "K"
GET_ERR = "xe"

# OPERATIONAL MODE & SETTINGS
GET_MODE = "xM"
MODE_RPM = "L"
MODE_RATE = "M"
MODE_VOL_RATE = "O"
MODE_VOL_TIME = "G"
MODE_VOL_PAUSE = "Q"
MODE_TIME = "N"
MODE_TIME_PAUSE = "P"

VAL_RPM = "S"
VAL_RATE_VOL_TIME = "f"
VAL_VOL = "v"
VAL_RUN_TIME = "xT"
VAL_PAUSE_TIME = "xP"
VAL_CYCLE_COUNT = r"\""
GET_MAX_RATE = "?"
GET_MAX_RATE_CALIBRATED = "!"
GET_TIME_VOL_RATE = "xv"
GET_TIME_VOL_RPM = "xw"

VOLTYPE_MIN = 1.000e-9
VOLTYPE_MAX = 9.999e9
TIMETYPE_MAX = 35964000

# CONFIGURATION

VAL_DIAMATER = "+"
VAL_BACKSTEP = "%"
RESET_ALL_DATA = "0"

# CALIBRATION

# SYSTEM

# TUBE IDs

# STATUS RESPONSE
RESPONSE_SUCCESSFULL = "*"
RESPONSE_UNSUCCESSFULL = "#"
RESPONSE_NEGATIVE = "–"
RESPONSE_POSITIVE = "+"

# KEYS
DIR_KEY = {
    +1: CCW,
    -1: CW,
    "ccw": CCW,
    "cw": CW,
    "CCW": CCW,
    "CW": CW,
    CCW: CCW,
    CW: CW,
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
        super().__init__(DIR_KEY[value], width=uint(1))

    def __str__(self) -> str:
        return f"{self._value}"

    def __repr__(self) -> str:
        key = {CCW: "CCW", CW: "CW"}
        return f"DirectionType({key[self._value]})"


class DiscreteType(RegloDataType):
    def __init__(
        self, value: uint | int = uint(), width: uint | int = 1, vmax=None
    ) -> None:
        self._vmin = uint()
        self._vmax = vmax if vmax is None else uint(vmax)
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

    def __init__(self, value=uint()) -> None:
        w = clip(len(str(value)), 1, 4)
        super().__init__(value, width=w, vmax=9999)


class DiscreteType2(DiscreteType):
    """Four characters representing a discrete integer value in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 4
    - Range: 0 to 9999
    """

    def __init__(self, value=uint()) -> None:
        super().__init__(value, width=4, vmax=9999)


class DiscreteType3(DiscreteType):
    """Six characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 6
    - Range: 0 to 999999
    """

    def __init__(self, value=uint()) -> None:
        super().__init__(value, width=6, vmax=999999)


class DiscreteType4(DiscreteType):
    """Ten characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 10
    - Range: 0 to 4294967295 (ie uint32)
    """

    def __init__(self, value=uint()) -> None:
        super().__init__(value, width=10, vmax=4294967295)


class DiscreteType5(DiscreteType):
    """Three characters in base 10.
    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 3
    - Range: 0 to 999
    """

    def __init__(self, value=uint()) -> None:
        super().__init__(value, width=3, vmax=999)


class DiscreteType6(DiscreteType):
    """Five characters in base 10.

    The value is right-justified.
    Unused digits to the left are zeros.

    - Width: 5
    - Range: 0 to 99999
    """

    def __init__(self, value=uint()) -> None:
        super().__init__(value, width=5, vmax=99999)


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
    def __init__(
        self, value: uint | int = uint(), width: uint | int = 1
    ) -> None:
        super().__init__(value, width=width, vmax=TIMETYPE_MAX)

    @property
    def hours(self):
        return self._value / 36000

    @property
    def minutes(self):
        return self._value / 600

    @property
    def seconds(self):
        return self._value / 10

    def __repr__(self) -> str:
        name = type(self).__name__
        args = f", width={self._width}" if (name == TimeType.__name__) else ""
        return f"{name}({self._value}{args})"


class TimeType1(TimeType):
    """Up to eight characters representing the time in 0 to 999 hours,
    with a resolution of 0.1 sec.

    Unused digits are not returned.

    - Width: 1 to 8
    - Units: 0.1 second
    - Range: 0 to 35,964,000 (0 to 999 hr)
    """

    def __init__(self, value=uint()) -> None:
        w = clip(len(str(value)), 1, 8)
        super().__init__(value, width=w)


class TimeType2(TimeType):
    """Eight characters representing the time in 0 to 999 hours,
    with a resolution of 0.1 sec.

    The value is right-justified.
    Unused digits to the left are filled with zeros.

    - Width: 8
    - Units: 0.1 second
    - Range: 0 to 35,964,000 (0 to 999 hr)
    """

    def __init__(self, value=uint()) -> None:
        super().__init__(value, width=8)


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
            "StringType can only contain ASCII characters "
            "(excluding `\\r`, ` `)"
        )
        assert isvalid_chr, err_chr

        super().__init__(value, width=w)
        pass

    def __repr__(self) -> str:
        return f"StringType({self._value!r})"


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
        variables = "|".join(str(a) for a in args)
        end = "\r"  # [CR] required for termination
        command = f"{addr}{code}{variables}{end}".strip()
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
