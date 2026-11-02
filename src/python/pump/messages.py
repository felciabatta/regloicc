from datetime import datetime

from serial import Serial

from ..timer import Timer
from ..utils import parse_none
from .constants import (
    AT,
    CHANNEL_ADDRESSING,
    CR,
    CRLF,
    EMPTY,
    EVT_CHANNEL_STOP,
    EVT_CHANNEL_UPDATE,
    EVT_ID,
    FAILURE,
    GET_BACKSTEP,
    GET_CAL_DIR,
    GET_CAL_TIME,
    GET_CHANNELNO,
    GET_CHN_ADR,
    GET_CYCLE_COUNT,
    GET_DIAMETER,
    GET_DIR,
    GET_DISPENSE_TIME_FLOWRATE,
    GET_DISPENSE_TIME_RPM,
    GET_DRIVE_ERR,
    GET_EVT_MSG,
    GET_FIRMWARE,
    GET_FLOWRATE,
    GET_FOOTSWITCH_STATUS,
    GET_HEAD_CODE,
    GET_LANG,
    GET_MAX_FLOWRATE,
    GET_MAX_FLOWRATE_CALIBRATED,
    GET_MODE,
    GET_MODEL_INFO,
    GET_PAUSE_10THS,
    GET_PAUSE_TIME,
    GET_PROTOCOL_VER,
    GET_RATE_OTHER,
    GET_REV_SINCE_RESET,
    GET_ROLLERNO,
    GET_ROLLSTEP_HI,
    GET_ROLLSTEP_LO,
    GET_ROLLVOL_CAL,
    GET_RPM,
    GET_RUN_STATUS,
    GET_RUN_TIME,
    GET_RUNTIME_SINCE,
    GET_SERIALNO,
    GET_TARGET_VOL,
    GET_TIME_10THS,
    GET_TIME_SINCE_RESET,
    GET_VOL,
    GET_VOL_SINCE_RESET,
    GET_VOLALL_SINCE_RESET,
    LEGACY_ADDRESSING,
    NEGATIVE,
    ONEBYTE,
    POSITIVE,
    RESET_CAL,
    RESET_CONFIG_DATA,
    RESET_ROLLVOL,
    SAVE_SET_ALL,
    SAVE_SET_ROLLVOL,
    SET_ACTUAL_VOL,
    SET_BACKSTEP,
    SET_CAL_CANCEL,
    SET_CAL_DIR,
    SET_CAL_START,
    SET_CAL_TIME,
    SET_CCW,
    SET_CHANNELNO,
    SET_CHN_ADR,
    SET_CW,
    SET_CYCLE_COUNT,
    SET_DIAMETER,
    SET_DISPLAY_CHR,
    SET_DISPLAY_NUM,
    SET_FACTORY_ROLLVOL,
    SET_FLOWRATE,
    SET_GUI_OFF,
    SET_GUI_ON,
    SET_HEAD_CODE,
    SET_LANG,
    SET_MODE_FLOWRATE,
    SET_MODE_RPM,
    SET_MODE_TIME,
    SET_MODE_VOL_PAUSE,
    SET_MODE_VOL_RATE,
    SET_MODE_VOL_TIME,
    SET_PAUSE,
    SET_PAUSE_10THS,
    SET_PAUSE_HRS,
    SET_PAUSE_MINS,
    SET_PAUSE_TIME,
    SET_PUMP_NAME,
    SET_RATE_OTHER,
    SET_ROLLERNO,
    SET_ROLLSTEP_HI,
    SET_ROLLSTEP_LO,
    SET_ROLLVOL_CAL,
    SET_RPM,
    SET_RUN_TIME,
    SET_SERIALNO,
    SET_START,
    SET_STOP,
    SET_TARGET_VOL,
    SET_TIME_10THS,
    SET_TIME_HRS,
    SET_TIME_MINS,
    SET_VOL,
    SP,
    STATUS_DESC_KEY,
    SUCCESS,
    VB,
)
from .reglotypes import (
    AddressType,
    BooleanType,
    ChannelAddressType,
    ChannelOrPxAddressType,
    CharType,
    DiameterType,
    DirectionType,
    DiscreteDiameterType,
    DiscreteRPMType,
    DiscreteType1,
    DiscreteType2,
    DiscreteType3,
    DiscreteType4,
    DiscreteType5,
    DiscreteType6,
    FailureType,
    FractionalType,
    LanguageType,
    MiscType,
    MissingType,
    NegativeType,
    PositiveType,
    ProxyAddressType,
    ProxyOrChAddressType,
    PumpAddressType,
    StatusType,
    StringType,
    SuccessType,
    TimeType1,
    TimeType2,
    UnAddressedType,
    UnknownType,
    VolumeType1,
    VolumeType2,
    ByteCodeType,
    StandardByteCodeType,
)


class RegloMessage:
    """Abstract base class for all messages sent or received"""

    def __init__(self, *, timer: Timer | None = None, **kws):
        self.timer = timer
        self.log_time()
        self._unused_kws = kws

    def __repr__(self) -> str:
        return f"{type(self).__name__}()"

    @property
    def datetime(self):
        return self._datetime

    @property
    def elapsed(self):
        return self._elapsed

    def log_time(self):
        self._datetime = self.now()
        tim = self.timer
        self._elapsed = None if tim is None else tim.elapsed

    @classmethod
    def now(cls):
        return datetime.now().astimezone()


class DeviceMessage(RegloMessage):
    """Abstract base class for messages received from the device"""


class OneByteResponse(DeviceMessage):
    _char = "X"

    def __init__(self, char, **kwargs) -> None:
        self._char = char
        super().__init__(**kwargs)

    def __repr__(self) -> str:
        return f"OneByteResponse({self.char!r})"

    def __str__(self) -> str:
        return str(self.char)

    @property
    def char(self):
        return self._char

    @char.setter
    def char(self, value):
        if isinstance(value, str):
            value = CharType(value)
        elif isinstance(value, int):
            value = CharType(str(value))
        elif not isinstance(value, CharType):
            raise ValueError(
                f"value {value!r} cannot be converted to CharType"
            )

        self._char = value

    @classmethod
    def from_str(cls, s, **kwargs):
        return cls(s, **kwargs)

    @classmethod
    def read(cls, serial: Serial, **kwargs):
        s = serial.read(ONEBYTE).decode()
        return cls.from_str(s, **kwargs)


class PumpAddressResponse(OneByteResponse):

    def __init__(self, addr, **kwargs) -> None:
        self.addr = addr
        super().__init__(str(addr), **kwargs)

    def __repr__(self) -> str:
        return f"PumpAddressResponse({self.addr!r})"

    def __str__(self) -> str:
        return str(self.addr)

    @property
    def addr(self) -> PumpAddressType:
        return self._addr

    @addr.setter
    def addr(self, value):
        if isinstance(value, int):
            value = PumpAddressType(value)
        elif isinstance(value, str) and len(value) == 1 and value.isnumeric:
            value = PumpAddressType(int(value))
        elif not isinstance(value, PumpAddressType):
            raise ValueError(
                f"value {value!r} cannot be converted to PumpAddressType"
            )

        self._addr = value


class StatusResponse(OneByteResponse):
    _statustype = None

    def __init__(self, status, **kwargs) -> None:
        self._status = status
        return super().__init__(str(status), **kwargs)

    def __repr__(self) -> str:
        cls = type(self)
        args = f"{self.status!r}" if (cls is StatusResponse) else ""
        return f"{cls.__name__}({args})"

    def __str__(self) -> str:
        return str(self.status)

    @property
    def status(self):
        return self._status

    @property
    def statustype(self):
        return parse_none(self._statustype, type(self.status))

    @property
    def desc(self):
        return getattr(self.status, "desc", "")

    @classmethod
    def from_str(cls, s: str, **kwargs):
        assert s.isascii(), "Encountered non-ASCII response."
        status = StatusType.from_str(s)
        if isinstance(status, SuccessType):
            return SuccessResponse(**kwargs)
        elif isinstance(status, FailureType):
            return FailureResponse(**kwargs)
        elif isinstance(status, NegativeType):
            return NegativeResponse(**kwargs)
        elif isinstance(status, PositiveType):
            return PositiveResponse(**kwargs)
        else:
            return MiscResponse.from_str(s, **kwargs)


class StandardStatusReponse(StatusResponse):
    _statustype = SuccessType

    def __init__(self, **kwargs) -> None:
        status = self._statustype()
        return super().__init__(status, **kwargs)


class SuccessResponse(StandardStatusReponse):
    _statustype = SuccessType


class FailureResponse(StandardStatusReponse):
    _statustype = FailureType


class NegativeResponse(StandardStatusReponse):
    _statustype = NegativeType


class PositiveResponse(StandardStatusReponse):
    _statustype = PositiveType


class MiscResponse(DeviceMessage):
    _misctype = None

    def __init__(self, value, **kwargs) -> None:
        self._value = value
        super().__init__(**kwargs)

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.value!r})"

    def __str__(self) -> str:
        return str(self.value)

    @property
    def value(self):
        return self._value

    @property
    def misctype(self):
        return parse_none(self._misctype, type(self.value))

    @classmethod
    def from_str(cls, s, **kwargs):
        misc = MiscType.from_str(s)
        if isinstance(misc, MissingType):
            return MissingResponse(**kwargs)
        elif isinstance(misc, UnknownType):
            return UnknownResponse(misc, **kwargs)


class MissingResponse(MiscResponse):
    _misctype = MissingType

    def __init__(self, **kwargs) -> None:
        super().__init__(MissingType(), **kwargs)

    def __repr__(self) -> str:
        return "MissingResponse()"


class UnknownResponse(MiscResponse):
    _misctype = UnknownType


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
        self.checkargs()
        return super().__init__(**kwargs)

    def __len__(self):
        return len(self.args)

    def __getitem__(self, key):
        if isinstance(key, int):
            return self.args[key]
        else:
            raise KeyError("index should be int")

    def __str__(self) -> str:
        args = self.sep.join(str(a) for a in self.args)
        return f"{self.begin}{args}{self.end}"

    def __repr__(self) -> str:
        optargs = [repr(a) for a in self.args]
        kws = f"begin={self.begin!r}, sep={self.sep!r}, end={self.end!r}"
        return f"DataMessage({", ".join(optargs + [kws])})"

    @property
    def args(self):
        return self._args

    @property
    def types(self):
        return parse_none(self._types, tuple(type(a) for a in self._args))

    @property
    def ntypes(self):
        return len(self.types)

    @property
    def nargs(self):
        return len(self.args)

    @property
    def begin(self):
        return self._begin

    @property
    def sep(self):
        return self._sep

    @property
    def end(self):
        return self._end

    def checkargs(self):
        isok = self.ntypes == self.nargs
        assert isok, f"Expected {self.ntypes} arguments, got {self.nargs}"
        return True

    @classmethod  # NOTE: specialise for subclass call signature
    def parse(cls, s, begin: int | str, sep: str, end: int | str, **kwargs):
        """Parse message string and return `cls_args`, `cls_kwargs`"""
        begin = s[:begin] if isinstance(begin, int) else begin
        end = s[-end:] if isinstance(end, int) else end
        assert s.startswith(
            begin
        ), f"response does not match format begin = {begin!r}"
        # input(f"end is {end!r}")
        # input(f"string is {s!r}")
        assert s.endswith(end), f"response does not match format end = {end!r}"
        str_data = s.rstrip(end).lstrip(begin)
        cls_kwargs = dict(begin=begin, sep=sep, end=end, **kwargs)
        cls_args = str_data.split(sep)
        # input(s)
        # input(cls_args)
        # input(cls_kwargs)
        return cls.convert_args(*cls_args), cls_kwargs

    @classmethod  # automatically convert if types provided by subclass
    def convert_args(cls, *args):
        types = cls._types
        if isinstance(types, (tuple, list)) and len(args) == len(types):
            # input(args)
            # input(types)
            return tuple(t.from_str(arg) for (t, arg) in zip(types, args))
        else:
            return args

    @classmethod  # NOTE: no need to specialise
    def from_str(cls, s: str, *args, **kwargs):
        """Parse message string and convert to a DataMessage type"""
        # TODO: what if error status response?
        # TODO: what if unknown response type?
        # TODO: what if no response — ""?
        try:
            cls_args, cls_kwargs = cls.parse(s, *args, **kwargs)
            return cls(*cls_args, **cls_kwargs)
        except Exception as e:
            print(f"Warning, encountered exception: {e!r}")
            return StatusResponse.from_str(s, **kwargs)

    @classmethod
    def read(cls, serial: Serial, expected=None, *args, **kwargs):
        """Read serial message and convert to a DataMessage type"""
        END = kwargs["end"] if "end" in kwargs else cls._END
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


class ProtocolVersionResponse(DataResponse):
    _types = (DiscreteType1,)


# 2.0 PUMP DRIVE
class DirectionResponse(DataResponse):
    _types = (DirectionType,)


class PumpErrorResponse(DataResponse):
    _types = (CharType, VolumeType1)  # NOTE: CHECK FORMAT MANUALLY


# 3.0 OPERATIONAL MODES AND SETTINGS
class ModeResponse(DataResponse):
    _types = (StandardByteCodeType,)


class RateOtherResponse(DataResponse):
    _types = (BooleanType,)  # NOTE: double check manually


class RPMResponse(DataResponse):
    _types = (FractionalType,)


class FlowrateResponse(DataResponse):
    _types = (VolumeType1,)


class DispenseVolumeResponse(DataResponse):
    _types = (VolumeType1,)


class DispensePeriodResponse(DataResponse):
    _types = (TimeType1,)


class CyclePauseTimeResponse(DataResponse):
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
    _types = (FractionalType, StringType)


class BackstepsResponse(DataResponse):
    _types = (DiscreteType1,)


# 5.0 CALIBRATION
class CalDirectionResponse(DataResponse):
    _types = (DirectionType,)


class CalTargetVolumeResponse(DataResponse):
    _types = (VolumeType1,)


class CalActualVolumeResponse(DataResponse):
    _types = (VolumeType1,)


class CalDurationResponse(DataResponse):
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


class DispenseTimeTenthsResponse(DataResponse):
    # NOTE: distinguish from GetDispensePeriod?
    _types = (DiscreteType1,)


class LowOrderStepsResponse(DataResponse):
    _types = (DiscreteType3,)


class HighOrderStepsResponse(DataResponse):
    _types = (DiscreteType3,)


class CalRollerVolumeResponse(DataResponse):
    _types = (VolumeType1,)  # NOTE: in nl units?


class CyclePauseTimeTenthsResponse(DataResponse):
    # NOTE: distinguish from GetCyclePauseTime?
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
    def read(cls, serial: Serial, **kwargs):
        return super().read(serial, cls._expected, **kwargs)


class StoppingEvent(EventMessage):
    _code = EVT_CHANNEL_STOP


class UpdateEvent(EventMessage):
    _code = EVT_CHANNEL_UPDATE


class RequestMessage(RegloMessage):
    _RTYPES = (StatusResponse, DataResponse, PumpAddressResponse)
    _addr_icctype = AddressType

    def __init__(
        self,
        addr=None,
        end=CRLF,
        response: (
            None | StatusResponse | DataResponse | PumpAddressResponse
        ) = None,
        rtype: (
            None
            | type[StatusResponse]
            | type[DataResponse]
            | type[PumpAddressResponse]
        ) = None,
        addr_mode=CHANNEL_ADDRESSING,
        addr_icctype=None,
        timer: Timer | None = None,
    ) -> None:

        # assert (
        #     addr is None or 0 <= addr <= 8
        # ), f"Address must be in range 0–8; got {addr}"
        self._addr_mode = addr_mode
        self._addr_icctype = parse_none(addr_icctype, self._addr_icctype)
        self.addr = addr
        self._end = end
        self._response = response
        self._rtype = rtype
        super().__init__(timer=timer)

    def __call__(self, serial: Serial):
        return self.write(serial)

    def __len__(self):
        return len(str(self))

    def __repr__(self) -> str:
        addr = None if self.addr is None else self.addr.value
        return f"RequestMessage({addr}, end={self.end!r})"

    def __str__(self):
        return self.end

    @property
    def addr(self):
        return self._addr

    @addr.setter
    def addr(self, value: int | None | AddressType):
        # input(f"Setting self.addr to {value!r}, of type {type(value)}")
        # input(f"self.addr_type is {self.addr_type}")
        if type(value) is self.addr_type:
            self._addr = value
        elif isinstance(value, AddressType):
            self.addr = value.value
        elif isinstance(value, (int, type(None))):
            self._addr = self.addr_type(value)
        else:
            self.addr = int(value)

    @property
    def addr_mode(self):
        return self._addr_mode

    @property
    def addr_icctype(self):
        return self._addr_icctype

    @property
    def addr_type(self) -> type:
        is_unaddressed = self.addr_icctype is UnAddressedType
        is_icc_mode = self.addr_mode is CHANNEL_ADDRESSING
        if is_unaddressed or is_icc_mode:
            return self.addr_icctype
        else:
            return PumpAddressType

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
            value = self.rcls.from_str(value, timer=self.timer)
        elif isinstance(value, Serial) and issubclass(self.rcls, self._RTYPES):
            value = self.rcls.read(value, timer=self.timer)
        else:
            raise ValueError(
                "response must be str, StatusResponse or DataResponse, "
                f"got type {type(value)}"
            )

        self._response = value

    @property
    def rtype(self):
        """Response message type."""
        return parse_none(self._rtype, type(self._response))

    @property
    def rcls(self):
        """Alias for rtype."""
        return self.rtype

    @property
    def encoded(self):
        """Encoded request string."""
        return self.encode()

    def encode(self):
        """Encode the request string."""
        return str(self).encode()

    def write(self, serial: Serial):
        """Write the request message to serial stream."""
        bytes_written = serial.write(self.encoded)
        if bytes_written != len(self.encoded):
            raise RuntimeError("Command was not fully written")
        self.log_time()  # log clock and elapsed times
        serial.flush()  # wait to finish transmission

        self.response = serial  # log_time is called automatically
        return self.response


class GetPumpAddress(RequestMessage):
    _end = CR
    _rtype = PumpAddressResponse
    _addr_icctype = UnAddressedType
    _addr = None

    def __init__(self, *args, **kwargs) -> None:
        return super().__init__(
            self._addr, end=self._end, rtype=self._rtype, **kwargs
        )

    def __repr__(self) -> str:
        return "GetPumpAddress()"

    def __str__(self) -> str:
        return f"{AT}{self.end}"


class SetPumpAddress(RequestMessage):
    _end = CR
    _rtype = StatusResponse
    _addr_icctype = PumpAddressType

    def __init__(self, addr, **kwargs) -> None:
        return super().__init__(
            addr, end=self._end, rtype=self._rtype, **kwargs
        )

    def __repr__(self) -> str:
        return f"SetPumpAddress({self.addr!r})"

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

    def __init__(self, addr, *args, code=None, rtype=None, **kwargs) -> None:
        self._args = self.convert_args(*args)
        self._code = parse_none(code, self._code)
        rtype = parse_none(rtype, self._rtype)
        return super().__init__(addr, end=self._end, rtype=rtype, **kwargs)

    def __repr__(self) -> str:
        cls = type(self)
        name = cls.__name__
        addr = repr(self.addr)
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
    def code(self) -> str | ByteCodeType:
        return self._code

    @property
    def types(self):
        return parse_none(self._types, tuple(type(a) for a in self._args))

    @classmethod  # automatically convert if types provided by subclass
    def convert_args(cls, *args):
        types = cls._types
        if isinstance(types, (tuple, list)) and len(args) == len(types):
            return tuple(
                arg if isinstance(arg, t) else t(arg)
                for (t, arg) in zip(types, args)
            )
        else:
            return args


class ShortDeviceCommand(DeviceCommand):
    def __init__(self, addr, code=None, rtype=None, **kwargs) -> None:
        super().__init__(addr, code=code, rtype=rtype, **kwargs)


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
    """Get an integer representing whether (1) or not (0) channel addressing
    is enabled.
    """

    _rtype = ChannelAddressingResponse
    _code = GET_CHN_ADR
    _addr_icctype = PumpAddressType


class SetChannelAddressing(SetterCommand):  # 1.2
    """Set whether channel messaging is enabled (1) or not enabled (0)."""

    _types = (BooleanType,)
    _code = SET_CHN_ADR
    _addr_icctype = PumpAddressType


class GetEventMessaging(ShortGetterCommand):  # 1.3
    """Get an integer representing whether (1) or not (0) event messages are
    enabled.
    """

    _rtype = EventMessagingResponse
    _code = GET_EVT_MSG
    _addr_icctype = ProxyAddressType


class SetEventMessaging(SetterCommand):  # 1.4
    """Set whether event messages are enabled (1) or not enabled (0)."""

    _types = (BooleanType,)
    _code = GET_EVT_MSG
    _addr_icctype = ProxyAddressType


class GetProtocolVersion(ShortGetterCommand):  # 1.5
    """Get an integer representing the version of the serial protocol."""

    _rtype = ProtocolVersionResponse
    _code = GET_PROTOCOL_VER
    _addr_icctype = ProxyAddressType


# 2.0 PUMP DRIVE
class StartPump(ShortSetterCommand):  # 2.1
    """Start pump."""

    _code = SET_START
    _addr_icctype = ChannelAddressType


class StopPump(ShortSetterCommand):  # 2.2
    """Stop pumping."""

    _code = SET_STOP
    _addr_icctype = ChannelAddressType


class PausePump(ShortSetterCommand):  # 2.3
    """Pause pumping (STOP in RPM or flow rate mode)."""

    _code = SET_PAUSE
    _addr_icctype = ChannelAddressType


class GetDirection(ShortGetterCommand):  # 2.4
    """Get pump direction."""

    _rtype = DirectionResponse
    _code = GET_DIR
    _addr_icctype = ChannelAddressType


class SetClockwise(ShortSetterCommand):  # 2.5
    """Set rotation direction to clockwise."""

    _code = SET_CW
    _addr_icctype = ChannelAddressType


class SetCounterClockwise(ShortSetterCommand):  # 2.6
    """Set rotation direction to counter-clockwise."""

    _code = SET_CCW
    _addr_icctype = ChannelAddressType


class GetPumpError(ShortGetterCommand):  # 2.7
    """Cause of `-` cannot run response = Parameter #1, Limiting value that
    was exceeded = Parameter #2.
    """

    _rtype = PumpErrorResponse
    _code = GET_DRIVE_ERR
    _addr_icctype = ChannelAddressType


# 3.0 OPERATIONAL MODES AND SETTINGS
class GetMode(ShortGetterCommand):  # 3.1
    """Get the current channel or pump mode."""

    _rtype = ModeResponse
    _code = GET_MODE
    _addr_icctype = ChannelAddressType


class SetModeRPM(ShortSetterCommand):  # 3.2
    """Set pump/channel to RPM mode."""

    _code = SET_MODE_RPM
    _addr_icctype = ChannelAddressType


class SetModeFlowrate(ShortSetterCommand):  # 3.3
    """Set pump/channel to Flow Rate mode."""

    _code = SET_MODE_FLOWRATE
    _addr_icctype = ChannelAddressType


class SetModeVolumeAtRate(ShortSetterCommand):  # 3.4
    """Set pump/channel to Volume (at rate) mode."""

    _code = SET_MODE_VOL_RATE
    _addr_icctype = ChannelAddressType


class SetModeVolumeOverPeriod(ShortSetterCommand):  # 3.5
    """Set pump/channel to Volume (over time) mode."""

    _code = SET_MODE_VOL_TIME
    _addr_icctype = ChannelAddressType


class SetModeVolumeAtRateCycle(ShortSetterCommand):  # 3.6
    """Set pump/channel to Volume + Pause mode."""

    _code = SET_MODE_VOL_PAUSE
    _addr_icctype = ChannelAddressType


class SetModeRateOverPeriod(ShortSetterCommand):  # 3.7
    """Set pump/channel to Time mode."""

    _code = SET_MODE_TIME
    _addr_icctype = ChannelAddressType


class SetModeRateOverPeriodCycle(ShortSetterCommand):  # 3.8
    """Set pump/channel to Time + Pause mode."""

    _code = SET_MODE_RPM
    _addr_icctype = ChannelAddressType


class SetMode(ShortSetterCommand):  # 3.2–3.8
    """Set pump/channel to a specific mode using code as input argument."""

    _addr_icctype = ChannelAddressType

    def __init__(self, addr, code: str | StandardByteCodeType, **kwargs):
        if isinstance(code, str):
            code = StandardByteCodeType.from_str(code)
        elif isinstance(code, type) and issubclass(code, StandardByteCodeType):
            code = code()
        elif isinstance(code, StandardByteCodeType):
            code = code
        else:
            raise TypeError(
                f"code must be str or StandardByteCodeType, got {type(code)}"
            )
        return super().__init__(addr, code=code, **kwargs)

    def __repr__(self) -> str:
        return f"SetMode({self.addr!r}, {self.code!r})"

    @classmethod
    def from_code(cls, code: str | ByteCodeType, **kwargs):
        # TODO: implement mapping from code to SetModeXXX class
        pass


class GetRateOther(GetterCommand):  # 3.9
    """Get flow rate from RPM (S) or flow rate (f) when mode is not RPM or
    flow rate.
    """

    _types = (BooleanType,)
    _rtype = RateOtherResponse
    _code = GET_RATE_OTHER
    _addr_icctype = ChannelAddressType


class SetRateOther(SetterCommand):  # 3.10
    """Set RPM flow rate not in RPM or flow rate mode Discrete Type 3."""

    _types = (DiscreteType3,)
    _code = SET_RATE_OTHER
    _addr_icctype = ChannelAddressType


class GetRPM(ShortGetterCommand):  # 3.11
    """Gets the current speed setting in RPM."""

    _rtype = RPMResponse
    _code = GET_RPM
    _addr_icctype = ChannelAddressType


class SetRPM(SetterCommand):  # 3.12
    """RPM mode flow rate setting (0.01 RPM) Discrete Type 3."""

    _types = (DiscreteRPMType,)
    _code = SET_RPM
    _addr_icctype = ChannelAddressType


class GetFlowrate(ShortGetterCommand):  # 3.13
    """Get current volume/time flow rate (mL/min)."""

    _rtype = FlowrateResponse
    _code = GET_FLOWRATE
    _addr_icctype = ChannelAddressType


class SetFlowrate(VerboseSetterCommand):  # 3.14
    """Set RPM flow rate in volume/time mode (mL/min) Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = FlowrateResponse
    _code = SET_FLOWRATE
    _addr_icctype = ChannelAddressType


class GetDispenseVolume(ShortGetterCommand):  # 3.15
    """Get the current setting for volume in mL."""

    _code = DispenseVolumeResponse
    _code = GET_VOL
    _addr_icctype = ChannelAddressType


class SetDispenseVolume(VerboseSetterCommand):  # 3.16
    """Set the current setting for volume in mL. Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = DispenseVolumeResponse
    _code = SET_VOL
    _addr_icctype = ChannelAddressType


class GetDispensePeriod(ShortGetterCommand):  # 3.17
    """Get the current pump run time."""

    _rtype = DispensePeriodResponse
    _code = GET_RUN_TIME
    _addr_icctype = ChannelAddressType


class SetDispensePeriod(SetterCommand):  # 3.18
    """Set current pump run time using Time Type 2."""

    _types = (TimeType2,)
    _code = SET_RUN_TIME
    _addr_icctype = ChannelAddressType


class GetCyclePauseTime(ShortGetterCommand):  # 3.19
    """Get pumping pause time."""

    _rtype = CyclePauseTimeResponse
    _code = GET_PAUSE_TIME
    _addr_icctype = ChannelAddressType


class SetCyclePauseTime(SetterCommand):  # 3.20
    """Set pumping pause time using Time Type 2."""

    _types = (TimeType2,)
    _code = SET_PAUSE_TIME
    _addr_icctype = ChannelAddressType


class GetCycleCount(ShortGetterCommand):  # 3.21
    """Get pump cycle count."""

    _rtype = CycleCountResponse
    _code = GET_CYCLE_COUNT
    _addr_icctype = ChannelAddressType


class SetCycleCount(SetterCommand):  # 3.22
    """Set pump cycle count Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_CYCLE_COUNT
    _addr_icctype = ChannelAddressType


class GetMaxFlowrate(ShortGetterCommand):  # 3.23
    """Max flow rate achievable with current settings mL/min."""

    # _rtype = MaxFlowRateResponse
    _code = GET_MAX_FLOWRATE
    _addr_icctype = ChannelAddressType


class GetMaxFlowrateCalibrated(ShortGetterCommand):  # 3.24
    """Max flow rate achievable with current settings using calibration."""

    # _rtype = MaxFlowRateCalibratedResponse
    _code = GET_MAX_FLOWRATE_CALIBRATED
    _addr_icctype = ChannelAddressType


class GetDispenseTimeByFlowrate(GetterCommand):  # 3.25
    """Get time to dispense at a given volume at a given mL/min flow rate.

    Vol, Volume Type 2; flow rate, Volume Type 2.
    """

    _types = (VolumeType2, VolumeType2)
    _rtype = DispenseTimeByFlowrateResponse
    _code = GET_DISPENSE_TIME_FLOWRATE
    _addr_icctype = ProxyAddressType


class GetDispenseTimeByRPM(GetterCommand):  # 3.26
    """Get time to dispense at a given volume at a given RPM.

    mL, Volume Type 2; flow rate, Discrete Type 3.
    """

    _types = (VolumeType2, DiscreteType3)
    _rtype = DispenseTimeByRPMResponse
    _code = GET_DISPENSE_TIME_RPM
    _addr_icctype = ProxyAddressType


# 4.0 CONFIGURATION
class GetTubeDiameter(ShortGetterCommand):  # 4.1
    """Get the current tubing inside diameter in mm.
    2 decimal places are returned.
    """

    _rtype = TubeDiameterResponse
    _code = GET_DIAMETER
    _addr_icctype = ChannelAddressType


class SetTubeDiameter(SetterCommand):  # 4.2
    """Set tubing inside diameter using Discrete Type 2."""

    _types = (DiscreteDiameterType,)
    _code = SET_DIAMETER
    _addr_icctype = ChannelAddressType


class GetBacksteps(ShortGetterCommand):  # 4.3
    """Get the current backsteps setting."""

    _rtype = BackstepsResponse
    _code = GET_BACKSTEP
    _addr_icctype = ChannelAddressType


class SetBacksteps(SetterCommand):  # 4.4
    """Set the current backsteps setting using Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_BACKSTEP
    _addr_icctype = ChannelAddressType


class ResetConfigData(ShortSetterCommand):  # 4.5
    """Resets all user configurable data to default values."""

    _code = RESET_CONFIG_DATA
    _addr_icctype = ChannelAddressType


# 5.0 CALIBRATION
class GetCalDirection(ShortGetterCommand):  # 5.1
    """Get direction flow for calibration."""

    _rtype = CalDirectionResponse
    _code = GET_CAL_DIR
    _addr_icctype = ChannelAddressType


class SetCalDirection(SetterCommand):  # 5.2
    """Set direction flow for calibration J or K using DIRECTION format."""

    _types = (DirectionType,)
    _code = SET_CAL_DIR
    _addr_icctype = ChannelAddressType


class GetCalTargetVolume(ShortGetterCommand):  # 5.3
    """Get the target volume to pump for calibrating, mL."""

    _rtype = CalTargetVolumeResponse
    _code = GET_TARGET_VOL
    _addr_icctype = ChannelAddressType


class SetCalTargetVolume(VerboseSetterCommand):  # 5.4
    """Set the target volume to pump for calibrating using Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = CalTargetVolumeResponse
    _code = SET_TARGET_VOL
    _addr_icctype = ChannelAddressType


class SetCalActualVolume(VerboseSetterCommand):  # 5.5
    """Set actual volume measured during calibration, mL Volume Type 2."""

    _types = (VolumeType2,)
    _rtype = CalActualVolumeResponse
    _code = SET_ACTUAL_VOL
    _addr_icctype = ChannelAddressType


class GetCalDuration(ShortGetterCommand):  # 5.6
    """Get the current calibration time."""

    _rtype = CalDurationResponse
    _code = GET_CAL_TIME
    _addr_icctype = ChannelAddressType


class SetCalDuration(SetterCommand):  # 5.7
    """Set the current calibration time using Time Type 2."""

    _types = (TimeType2,)
    _code = SET_CAL_TIME
    _addr_icctype = ChannelAddressType


class GetRuntimeSinceCal(ShortGetterCommand):  # 5.8
    """Get the channel run time since last calibration."""

    _rtype = RuntimeSinceCalResponse
    _code = GET_RUNTIME_SINCE
    _addr_icctype = ChannelAddressType


class StartCalibration(ShortSetterCommand):  # 5.9
    """Start calibration on a channel(s)."""

    _code = SET_CAL_START
    _addr_icctype = ChannelAddressType


class CancelCalibration(ShortSetterCommand):  # 5.10
    """Cancel calibration."""

    _code = SET_CAL_CANCEL
    _addr_icctype = ChannelAddressType


# 6.0 SYSTEM
class GetFirmwareVersion(ShortGetterCommand):  # 6.1
    """Returns the pump firmware version."""

    _rtype = FirmwareVersionResponse
    _code = GET_FIRMWARE
    _addr_icctype = ProxyAddressType


class SetFactoryRollerVolume(SetterCommand):  # 6.2
    """Change factory roller step volume for a particular roller count and
    tubing size.

    - roller count (6, 8, 12), Discrete Type 1
    - index of the tubing diameter (see Table 1), Discrete Type 1
    - RSV, Volume Type 2.
    """

    _types = (DiscreteType1, DiscreteType1, VolumeType2)
    # NOTE: distinguish from SetCalRollerVolume?
    _code = SET_FACTORY_ROLLVOL
    _addr_icctype = ProxyAddressType


class SaveRollerStepSetting(ShortSetterCommand):  # 6.3
    """Save set roller step settings."""

    # NOTE: distinguish from SetCalRollerVolume?
    _code = SAVE_SET_ROLLVOL
    _addr_icctype = ProxyAddressType


class RestoreFactoryRollerVolume(ShortSetterCommand):  # 6.4
    """Reset roller step volume table to defaults."""

    _code = RESET_ROLLVOL
    _addr_icctype = ProxyAddressType


class SetDisplayName(SetterCommand):  # 6.5
    """Set pump name for display under remote control–String."""

    _types = (StringType,)
    _code = SET_PUMP_NAME
    _addr_icctype = ProxyAddressType


class GetPumpSerialNo(ShortGetterCommand):  # 6.6
    """Get pump serial number."""

    _rtype = PumpSerialNoResponse
    _code = GET_SERIALNO
    _addr_icctype = ProxyAddressType


class SetPumpSerialNo(SetterCommand):  # 6.7
    """Set pump serial number–String."""

    _types = (StringType,)
    _code = SET_SERIALNO
    _addr_icctype = ProxyAddressType


class GetPumpLanguage(ShortGetterCommand):  # 6.8
    """Get the current pump language."""

    _rtype = PumpLanguageResponse
    _code = GET_LANG
    _addr_icctype = ProxyAddressType


class SetPumpLanguage(SetterCommand):  # 6.9
    """Set current pump language–Language."""

    _types = (LanguageType,)
    _code = SET_LANG
    _addr_icctype = ProxyAddressType


class GetNumberOfChannels(ShortGetterCommand):  # 6.10
    """Get number of pump channels."""

    _rtype = NumberOfChannelsResponse
    _code = GET_CHANNELNO
    _addr_icctype = ProxyAddressType


class SetNumberOfChannels(SetterCommand):  # 6.11
    """Configure number of pump channels. Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_CHANNELNO
    _addr_icctype = ProxyAddressType


class GetNumberOfRollers(ShortGetterCommand):  # 6.12
    """Get number of rollers for channel."""

    _rtype = NumberOfRollersResponse
    _code = GET_ROLLERNO
    _addr_icctype = ChannelAddressType


class SetNumberOfRollers(SetterCommand):  # 6.13
    """Set number of rollers for channel. Discrete Type 2."""

    _types = (DiscreteType2,)
    _code = SET_ROLLERNO
    _addr_icctype = ChannelAddressType


class GetRevsSinceReset(ShortGetterCommand):  # 6.14
    """Get total number of revolutions since last reset."""

    _rtype = RevsSinceResetResponse
    _code = GET_REV_SINCE_RESET
    _addr_icctype = ChannelAddressType


class GetVolumeSinceReset(ShortGetterCommand):  # 6.15
    """Get channel total volume pumped since last reset, mL."""

    # NOTE: distinguish from GetTotalVolumeSinceReset?
    # same, but different output format, and only in mL
    _rtype = VolumeSinceResetResponse
    _code = GET_VOL_SINCE_RESET
    _addr_icctype = ChannelAddressType


class GetTimePumpedSinceReset(ShortGetterCommand):  # 6.16
    """Get total time pumped for a channel since last reset."""

    _rtype = TimePumpedSinceResetResponse
    _code = GET_TIME_SINCE_RESET
    _addr_icctype = ChannelAddressType


class EnableGUI(ShortSetterCommand):  # 6.17
    """Set control from the pump user interface."""

    _code = SET_GUI_ON
    _addr_icctype = ProxyAddressType


class DisableGUI(ShortSetterCommand):  # 6.18
    """Disable pump user interface."""

    _code = SET_GUI_OFF
    _addr_icctype = ProxyAddressType


class SetDisplayNumbers(SetterCommand):  # 6.19
    """Write numbers to the pump to display while under external control.

    - String (<17 characters).
    """

    _types = (StringType,)
    _code = SET_DISPLAY_NUM
    _addr_icctype = ProxyAddressType


class SetDisplayCharacters(SetterCommand):  # 6.20
    """Write numbers to the pump to display while under external control.

    - String (<17 characters).
    """

    _types = (StringType,)
    _code = SET_DISPLAY_CHR
    _addr_icctype = ProxyAddressType


class GetRunStatus(StatusGetterCommand):  # 6.21
    """Returns whether the pump is currently running or not."""

    _code = GET_RUN_STATUS
    _addr_icctype = ProxyAddressType


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
     The first digit represents the number of channels for the pump, and
     the second 2 digits represent the number of rollers.
     `"XX"` if channels do not have the same number of rollers.
    """

    _rtype = ModelInfoResponse
    _code = GET_MODEL_INFO
    _addr_icctype = ProxyAddressType


class GetPumpHeadModel(ShortGetterCommand):  # 6.23
    """Returns the pump head model type code

    - A 4 digit code indicating the ID number of the pump head.
    The first two digits represent the number of channels on the head,
    and the second 2 digits represent the number of rollers.
    """

    _rtype = PumpHeadModelResponse
    _code = GET_HEAD_CODE
    _addr_icctype = ProxyAddressType


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
    _addr_icctype = ProxyAddressType


class GetDispenseTimeTenths(ShortGetterCommand):  # 6.25
    """Get the current setting for pump time in 1/10 second."""

    # NOTE: distinguish from GetDispensePeriod?
    # - same, but different output format
    _rtype = DispenseTimeTenthsResponse
    _code = GET_TIME_10THS
    _addr_icctype = ChannelAddressType


class SetDispenseTimeTenths(SetterCommand):  # 6.26
    """Set the current setting for pump time in 1/10 second.
    Discrete Type 2.
    """

    # NOTE: distinguish from SetDispensePeriod?
    # - same, but different input format
    _types = (DiscreteType2,)
    _code = SET_TIME_10THS
    _addr_icctype = ChannelAddressType


class SetDispenseTimeMins(SetterCommand):  # 6.27
    """Set the current run time setting for dispensing in minutes.
    Discrete Type 5.
    """

    # NOTE: distinguish from SetDispensePeriod?
    _types = (DiscreteType5,)
    _code = SET_TIME_MINS
    _addr_icctype = ChannelAddressType


class SetDispenseTimeHrs(SetterCommand):  # 6.28
    """Set the current run time setting for dispensing in hours.
    Discrete Type 5.
    """

    # NOTE: distinguish from SetDispensePeriod?
    _types = (DiscreteType5,)
    _code = SET_TIME_HRS
    _addr_icctype = ChannelAddressType


class GetLowOrderSteps(ShortGetterCommand):  # 6.29
    """Get the low order roller steps. The total number of roller steps which
    are dispensed during an operation is computed as:[(u*65536]+(U)]."""

    _rtype = LowOrderStepsResponse
    _code = GET_ROLLSTEP_LO
    _addr_icctype = ChannelAddressType


class SetLowOrderSteps(SetterCommand):  # 6.30
    """Set the high order roller steps. Discrete Type 6."""

    _types = (DiscreteType6,)
    _code = SET_ROLLSTEP_LO
    _addr_icctype = ChannelAddressType


class GetHighOrderSteps(ShortGetterCommand):  # 6.31
    """Get the high order roller steps. The total number of roller steps which
    are dispensed during an operation is computed as:[(u*65536]+(U)]."""

    _rtype = HighOrderStepsResponse
    _code = GET_ROLLSTEP_HI
    _addr_icctype = ChannelAddressType


class SetHighOrderSteps(SetterCommand):  # 6.32
    """Set the high order roller steps. Discrete Type 6."""

    _types = (DiscreteType6,)
    _code = SET_ROLLSTEP_HI
    _addr_icctype = ChannelAddressType


class GetCalRollerVolume(ShortGetterCommand):  # 6.33
    """Get the current roller step volume

    Based on the current calibration, tubing diameter and roller count.
    If no calibration has been performed the default volume is returned.
    """

    _rtype = CalRollerVolumeResponse
    _code = GET_ROLLVOL_CAL
    _addr_icctype = ChannelAddressType


class SetCalRollerVolume(SetterCommand):  # 6.34
    """Set the calibrated roller step volume to use for this pump or channel.

    This value is used as the calibrated value and is overwritten by
    subsequent calibrations and reset by changing tubing diameter.
    Volume Type 2.
    """

    # NOTE: distinguish from SetFactoryRollerVolume
    # - factory volume is the "default", channel-independent
    # - calibration volume overrides default, if set
    _types = (VolumeType2,)
    _code = SET_ROLLVOL_CAL
    _addr_icctype = ChannelAddressType


class ResetCalibrationData(ShortSetterCommand):  # 6.35
    """Reset the pump to discard calibration data, use default roller step
    volume.
    """

    # NOTE: distinguish from ResetConfigData
    # - config is for tube diameter and backsteps
    # - calibration is for tuning volume/rate units to tube & roller specs
    _code = RESET_CAL
    _addr_icctype = ProxyAddressType  # TODO: Double check


class GetCyclePauseTimeTenths(ShortGetterCommand):  # 6.36
    """Get the current setting for pause time in 1/10 second."""

    # NOTE: distinguish from GetCyclePauseTime?
    # - same, but different output format
    _rtype = CyclePauseTimeTenthsResponse
    _code = GET_PAUSE_10THS
    _addr_icctype = ChannelAddressType


class SetCyclePauseTimeTenths(SetterCommand):  # 6.37
    """Set the current setting for pause time in 1/10 second.
    Discrete Type 2.
    """

    # NOTE: distinguish from SetCyclePauseTime?
    # - same, but different input format
    _types = (DiscreteType2,)
    _code = SET_PAUSE_10THS
    _addr_icctype = ChannelAddressType


class SetCyclePauseTimeMins(SetterCommand):  # 6.38
    """Set the current setting for pause time in minutes. Discrete Type 5."""

    # NOTE: distinguish from SetCyclePauseTime?
    _types = (DiscreteType5,)
    _code = SET_PAUSE_MINS
    _addr_icctype = ChannelAddressType


class SetCyclePauseTimeHrs(SetterCommand):  # 6.39
    """Set the current setting for pause time in hours. Discrete Type 5."""

    # NOTE: distinguish from SetCyclePauseTime?
    _types = (DiscreteType5,)
    _code = SET_PAUSE_HRS
    _addr_icctype = ChannelAddressType


class GetTotalVolumeSinceReset(ShortGetterCommand):  # 6.40
    """Get the total volume dispensed since the last reset in µL, mL or
    liters.
    """

    # NOTE: distinguish from GetVolumeSinceReset?
    # - same, but converts to a "nice" unit
    # _types = ()
    # _rtype = TotalVolumeSinceResetResponse
    _code = GET_VOLALL_SINCE_RESET
    _addr_icctype = ChannelAddressType


class SavePumpSettings(ShortSetterCommand):  # 6.41
    """Saves the current pump settings values to memory."""

    _code = SAVE_SET_ALL
    _addr_icctype = ProxyAddressType


class GetFootSwitchStatus(StatusGetterCommand):  # 6.42
    """Get the current state of the foot switch."""

    _code = GET_FOOTSWITCH_STATUS
    _addr_icctype = ProxyAddressType
