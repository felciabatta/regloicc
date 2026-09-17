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

from pandas import DataFrame
from pandas import concat as pdcat
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial

from ..utils import flux2mmmmse, get_port, parse_none
from .constants import (
    AT,
    CARET,
    CCW,
    CR,
    CRLF,
    CSTATUS_CALI_PENDING,
    CSTATUS_CALI_PUMPING,
    CSTATUS_DESC_KEY,
    CSTATUS_PAUSED_BTWN_CYCLES,
    CSTATUS_PUMPING,
    CSTATUS_STOPPED,
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
    ERR_CHN_SETTING,
    ERR_CYCLE_ZERO,
    ERR_MAX_RATE,
    ERR_MAX_VOL,
    EVT_CHANNEL_STOP,
    EVT_CHANNEL_UPDATE,
    EVT_DESC_KEY,
    EVT_ID,
    FAILURE,
    FRENCH,
    FS_GROUNDED,
    FS_OPEN,
    GERMAN,
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
    GET_SERIAL_VER,
    GET_SERIALNO,
    GET_TARGET_VOL,
    GET_TIME_10THS,
    GET_TIME_SINCE_RESET,
    GET_VOL,
    GET_VOL_SINCE_RESET,
    GET_VOLALL_SINCE_RESET,
    ISO_ENGLISH,
    ISO_FRENCH,
    ISO_GERMAN,
    ISO_LANG_KEY,
    ISO_SPANISH,
    LANG_KEY,
    LF,
    MODE_FLOWRATE,
    MODE_RPM,
    MODE_TIME,
    MODE_TIME_PAUSE,
    MODE_VOL_PAUSE,
    MODE_VOL_RATE,
    MODE_VOL_TIME,
    NEGATIVE,
    POSITIVE,
    PUMP_RUNNING,
    PUMP_STOPPED,
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
    SET_EVT_MSG,
    SET_FACTORY_ROLLVOL,
    SET_FLOWRATE,
    SET_GUI_OFF,
    SET_GUI_ON,
    SET_HEAD_CODE,
    SET_LANG,
    SET_MODE_FLOWRATE,
    SET_MODE_RPM,
    SET_MODE_TIME,
    SET_MODE_TIME_PAUSE,
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
    SPANISH,
    STATUS_CHARS,
    STATUS_DESC_KEY,
    STOPPED_CALI_COMPLETE,
    STOPPED_DESC_KEY,
    STOPPED_MANUAL,
    STOPPED_OVERCURRENT,
    STOPPED_OVERHEAT,
    STOPPED_PUMP_COMPLETE,
    STR_ENGLISH,
    STR_FRENCH,
    STR_GERMAN,
    STR_LANG_KEY,
    STR_SPANISH,
    SUCCESS,
    TIMETYPE_MAX,
    TUBE_IDS,
    VB,
    VOLTYPE_MAX,
    VOLTYPE_MIN,
)
from .messages import (
    BackstepsResponse,
    CalActualVolumeResponse,
    CalDirectionResponse,
    CalTargetVolumeResponse,
    CalTimeResponse,
    CancelCalibration,
    ChannelAddressingResponse,
    CycleCountResponse,
    DataMessage,
    DataResponse,
    DeviceCommand,
    DeviceMessage,
    DiableGIU,
    DirectionResponse,
    DispenseTimeByFlowrateResponse,
    DispenseTimeByRPMResponse,
    EnableGUI,
    EventMessage,
    EventMessagingResponse,
    FailureResponse,
    FirmwareVersionResponse,
    FlowrateResponse,
    GetBacksteps,
    GetCalDirection,
    GetCalTargetVolume,
    GetCalTime,
    GetChannelAddressing,
    GetCycleCount,
    GetDirection,
    GetDispenseTimeByFlowrate,
    GetDispenseTimeByRPM,
    GetEventMessaging,
    GetFirmwareVersion,
    GetFlowrate,
    GetFootSwitchStatus,
    GetHighOrderSteps,
    GetLowOrderSteps,
    GetMaxFlowrate,
    GetMaxFlowrateCalibrated,
    GetMode,
    GetModelInfo,
    GetNumberOfChannels,
    GetNumberOfRollers,
    GetPauseTime,
    GetPauseTime10ths,
    GetPumpError,
    GetPumpHeadModel,
    GetPumpLanguage,
    GetPumpSerialNo,
    GetPumpTime10ths,
    GetRateOther,
    GetRevsSinceReset,
    GetRollerVolume,
    GetRPM,
    GetRunStatus,
    GetRuntimeSinceCal,
    GetSerialVersion,
    GetterCommand,
    GetTimeFixed,
    GetTimePumpedSinceReset,
    GetTotalVolumeSinceReset,
    GetTubeDiameter,
    GetVolumeFixed,
    GetVolumeSinceReset,
    HighOrderStepsResponse,
    LowOrderStepsResponse,
    MiscResponse,
    MissingResponse,
    ModelInfoResponse,
    ModeResponse,
    NegativeResponse,
    NumberOfChannelsResponse,
    NumberOfRollersResponse,
    PausePump,
    PauseTime10thsResponse,
    PauseTimeResponse,
    PositiveResponse,
    PumpErrorResponse,
    PumpHeadModelResponse,
    PumpLanguageResponse,
    PumpSerialNoResponse,
    PumpTime10thsResponse,
    RateOtherResponse,
    RegloMessage,
    RequestMessage,
    ResetCalibrationData,
    ResetConfigData,
    ResetRollerVolume,
    RevsSinceResetResponse,
    RollerVolumeResponse,
    RPMResponse,
    RuntimeSinceCalResponse,
    SavePumpSettings,
    SaveSetRollerVolume,
    Serial,
    SerialVersionResponse,
    SetBacksteps,
    SetCalActualVolume,
    SetCalDirection,
    SetCalTargetVolume,
    SetCalTime,
    SetChannelAddressing,
    SetClockwise,
    SetCounterClockwise,
    SetCycleCount,
    SetDispenseTimeHrs,
    SetDispenseTimeMins,
    SetDisplayCharacters,
    SetDisplayNumbers,
    SetEventMessaging,
    SetFactoryRollerVolume,
    SetFlowrate,
    SetHighOrderSteps,
    SetLowOrderSteps,
    SetModeFlowrate,
    SetModeRPM,
    SetModeTimeRateCycled,
    SetModeTimeRateFixed,
    SetModeVolumeRateCycled,
    SetModeVolumeRateFixed,
    SetModeVolumeTimeFixed,
    SetNumberOfChannels,
    SetNumberOfRollers,
    SetPauseTime,
    SetPauseTime10ths,
    SetPauseTimeHrs,
    SetPauseTimeMins,
    SetPumpAddress,
    SetPumpDisplayName,
    SetPumpHeadModel,
    SetPumpLanguage,
    SetPumpSerialNo,
    SetPumpTime10ths,
    SetRateOther,
    SetRollerVolume,
    SetRPM,
    SetterCommand,
    SetTimeFixed,
    SetTubeDiameter,
    SetVolumeFixed,
    ShortDeviceCommand,
    ShortGetterCommand,
    ShortSetterCommand,
    StartCalibration,
    StartPump,
    StatusGetterCommand,
    StatusResponse,
    StoppingEvent,
    StopPump,
    SuccessResponse,
    TimeFixedResponse,
    TimePumpedSinceResetResponse,
    TubeDiameterResponse,
    UnknownResponse,
    UpdateEvent,
    VerboseSetterCommand,
    VolumeFixedResponse,
    VolumeSinceResetResponse,
)
from .reglotypes import (
    AddressType,
    BooleanType,
    CharType,
    ClockwiseType,
    CounterClockwiseType,
    DiameterType,
    DirectionType,
    DiscreteType,
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
    RegloDataType,
    StatusType,
    StringType,
    SuccessType,
    TimeType,
    TimeType1,
    TimeType2,
    UnknownType,
    VolumeType1,
    VolumeType2,
)


class RegloChannel:
    _serial = None

    def __init__(
        self,
        addr: int = 1,
        serial: None | str | Serial = _serial,
        tube_id=None | float,
        verbose=True,
        tube_diameter=0.51,
        display_message="we be pumpin'",
        mode=MODE_FLOWRATE,
    ) -> None:
        # Open serial port with data format corresponding to the RegloICC pump
        self.addr = addr
        self.serial = serial
        self._log = DataFrame(
            columns=[
                "datetime",
                "request_desc",
                "response_desc",
                "request",
                "response",
            ]
        )
        self.verbose = verbose
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
                timeout=3,
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
            datetime=[request.datetime],
            request_desc=[repr(request)],
            response_desc=[repr(request.response)],
            request=[request],
            response=[request.response],
        )
        self._log = pdcat([self._log, DataFrame(row_new)], ignore_index=True)
        if self.verbose:
            print(self.log.iloc[-1:])
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
