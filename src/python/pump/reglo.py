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
from rich.progress import Progress
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial

from ..utils import flux2mmmmse, get_port, parse_none, timer
from .constants import (
    AT,
    CARET,
    CCW,
    CHANNEL_ADDRESSING,
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
    FOOTSWITCH_GROUNDED,
    FOOTSWITCH_OPEN,
    FRENCH,
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
    CalDurationResponse,
    CalRollerVolumeResponse,
    CalTargetVolumeResponse,
    CancelCalibration,
    ChannelAddressingResponse,
    CycleCountResponse,
    CyclePauseTimeResponse,
    CyclePauseTimeTenthsResponse,
    DataMessage,
    DataResponse,
    DeviceCommand,
    DeviceMessage,
    DirectionResponse,
    DisableGUI,
    DispensePeriodResponse,
    DispenseTimeByFlowrateResponse,
    DispenseTimeByRPMResponse,
    DispenseTimeTenthsResponse,
    DispenseVolumeResponse,
    EnableGUI,
    EventMessage,
    EventMessagingResponse,
    FailureResponse,
    FirmwareVersionResponse,
    FlowrateResponse,
    GetBacksteps,
    GetCalDirection,
    GetCalDuration,
    GetCalRollerVolume,
    GetCalTargetVolume,
    GetChannelAddressing,
    GetCycleCount,
    GetCyclePauseTime,
    GetCyclePauseTimeTenths,
    GetDirection,
    GetDispensePeriod,
    GetDispenseTimeByFlowrate,
    GetDispenseTimeByRPM,
    GetDispenseTimeTenths,
    GetDispenseVolume,
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
    GetProtocolVersion,
    GetPumpAddress,
    GetPumpError,
    GetPumpHeadModel,
    GetPumpLanguage,
    GetPumpSerialNo,
    GetRateOther,
    GetRevsSinceReset,
    GetRPM,
    GetRunStatus,
    GetRuntimeSinceCal,
    GetterCommand,
    GetTimePumpedSinceReset,
    GetTotalVolumeSinceReset,
    GetTubeDiameter,
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
    PositiveResponse,
    ProtocolVersionResponse,
    PumpAddressResponse,
    PumpErrorResponse,
    PumpHeadModelResponse,
    PumpLanguageResponse,
    PumpSerialNoResponse,
    RateOtherResponse,
    RegloMessage,
    RequestMessage,
    ResetCalibrationData,
    ResetConfigData,
    RestoreFactoryRollerVolume,
    RevsSinceResetResponse,
    RPMResponse,
    RuntimeSinceCalResponse,
    SavePumpSettings,
    SaveRollerStepSetting,
    Serial,
    SetBacksteps,
    SetCalActualVolume,
    SetCalDirection,
    SetCalDuration,
    SetCalRollerVolume,
    SetCalTargetVolume,
    SetChannelAddressing,
    SetClockwise,
    SetCounterClockwise,
    SetCycleCount,
    SetCyclePauseTime,
    SetCyclePauseTimeHrs,
    SetCyclePauseTimeMins,
    SetCyclePauseTimeTenths,
    SetDispensePeriod,
    SetDispenseTimeHrs,
    SetDispenseTimeMins,
    SetDispenseTimeTenths,
    SetDispenseVolume,
    SetDisplayCharacters,
    SetDisplayName,
    SetDisplayNumbers,
    SetEventMessaging,
    SetFactoryRollerVolume,
    SetFlowrate,
    SetHighOrderSteps,
    SetLowOrderSteps,
    SetMode,
    SetModeFlowrate,
    SetModeRateOverPeriod,
    SetModeRateOverPeriodCycle,
    SetModeRPM,
    SetModeVolumeAtRate,
    SetModeVolumeAtRateCycle,
    SetModeVolumeOverPeriod,
    SetNumberOfChannels,
    SetNumberOfRollers,
    SetPumpAddress,
    SetPumpHeadModel,
    SetPumpLanguage,
    SetPumpSerialNo,
    SetRateOther,
    SetRPM,
    SetterCommand,
    SetTubeDiameter,
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
    TimePumpedSinceResetResponse,
    TubeDiameterResponse,
    UnknownResponse,
    UpdateEvent,
    VerboseSetterCommand,
    VolumeSinceResetResponse,
)
from .reglotypes import (
    AddressType,
    BooleanType,
    ByteCodeType,
    ChannelAddressType,
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
    ModeFlowrate,
    ModeRateOverPeriod,
    ModeRateOverPeriodCycle,
    ModeRPM,
    ModeVolumeAtRate,
    ModeVolumeAtRateCycle,
    ModeVolumeOverPeriod,
    NegativeType,
    PositiveType,
    PumpAddressType,
    RegloDataType,
    StandardByteCodeType,
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


class RegloDevice:
    _serial = None
    _addr_mode = CHANNEL_ADDRESSING

    def __init__(
        self,
        addr: ChannelAddressType = ChannelAddressType(),
        serial: None | str | Serial = _serial,
        pump_addr: PumpAddressType | None = None,
        verbose=True,
        tube_diameter=0.51,
        display_message="we be pumpin'",
        mode=ModeFlowrate(),
        flowrate_limit=0.5,
    ) -> None:
        # Open serial port with data format corresponding to the RegloICC pump
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
        self._flowrate_limit = flowrate_limit

        self.addr = addr
        self.pump_addr = pump_addr
        self.channel_addressing = self._addr_mode
        self.tube_diameter = round(tube_diameter * 100)
        self.display_nums = display_message
        self.stop()

        return None

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
                get_port(value, "ISMATEC Reglo ICC"),
                9600,
                parity=PARITY_NONE,
                stopbits=STOPBITS_ONE,
                bytesize=EIGHTBITS,
                timeout=3,
            )

        self._serial = serial

    @property
    def current_request(self):
        return self._current_request

    @current_request.setter
    def current_request(self, value: RequestMessage | DeviceCommand):
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
        request: (
            RequestMessage
            | DeviceCommand
            | type[RequestMessage]
            | type[GetPumpAddress]
            | type[SetPumpAddress]
            | type[DeviceCommand]
        ),
        *args,
        addr=None,
        **kwargs,
    ):
        """Send a new request to the pump and read the response."""

        if isinstance(request, type) and issubclass(request, RequestMessage):
            args = (parse_none(addr, self.addr), *args)
            request = request(*args, addr_mode=self._addr_mode, **kwargs)
            return self.send_request(request)

        if self.serial is None:
            return None

        request(self.serial)
        self.current_request = request
        return request.response

    @property
    def flowrate_limit(self):
        return self._flowrate_limit

    # PUMP ADDRESSING
    @property
    def pump_addr(self) -> PumpAddressType:
        return self.send_request(GetPumpAddress).addr

    @pump_addr.setter
    def pump_addr(self, value):
        if value is None:
            return None
        return self.send_request(SetPumpAddress, addr=value)

    # 1.0 COMMUNICATIONS MANAGEMENT
    @property
    def channel_addressing(self):  # 1.1
        # NOTE: always legacy pump addr (1–8), as set by "@<pump_addr>"
        response = self.send_request(GetChannelAddressing)
        self._addr_mode = int(response.args[0])
        return response

    @channel_addressing.setter
    def channel_addressing(self, value):  # 1.2
        # NOTE: always legacy pump addr (1–8), as set by "@<pump_addr>"
        response = self.send_request(SetChannelAddressing, value)
        self._addr_mode = int(value)
        return response

    @property
    def event_messaging(self):  # 1.3 — pump-level
        # TODO: auto switch between addr=0 and self.addr depending on ICC
        return self.send_request(GetEventMessaging)

    @event_messaging.setter
    def event_messaging(self, value):  # 1.4
        return self.send_request(SetEventMessaging, value)

    @property
    def protocol_version(self):  # 1.5
        return self.send_request(GetProtocolVersion)

    # 2.0 PUMP DRIVE
    def start(self):  # 2.1
        return self.send_request(StartPump)

    def stop(self):  # 2.2
        return self.send_request(StopPump)

    def pause(self):  # 2.3
        return self.send_request(PausePump)

    @property
    def direction(self):  # 2.4
        return self.send_request(GetDirection)

    @direction.setter
    def direction(self, value):  # 2.5 – 2.6
        if DIR_KEY[value] == CW:
            return self.send_request(SetClockwise)
        elif DIR_KEY[value] == CCW:
            return self.send_request(SetCounterClockwise)

    @property
    def error(self):  # 2.7
        return self.send_request(GetPumpError)

    # 3.0 OPERATIONAL MODES AND SETTINGS
    @property
    def mode(self):  # 3.1
        return self.send_request(GetMode)

    @mode.setter
    def mode(self, value):  # 3.2 – 3.8
        return self.send_request(SetMode, value)

    @property
    def rate_other(self):  # 3.9
        return self.send_request(GetRateOther)

    @rate_other.setter
    def rate_other(self, value):  # 3.10
        return self.send_request(SetRateOther, value)

    @property
    def rpm(self):  # 3.11
        return self.send_request(GetRPM)

    @rpm.setter
    def rpm(self, value):  # 3.12
        return self.send_request(SetRPM, value)

    @property
    def flowrate(self):  # 3.13
        response = self.send_request(GetFlowrate)
        return float(response[0])

    @flowrate.setter
    def flowrate(self, value):  # 3.14
        if value > self.flowrate_limit:
            print(
                f"Warning: requested flowrate {value} mL/min exceeds limit — "
                f"rate will be capped at {self.flowrate_limit} mL/min.\n\n"
                "To use a higher flowrate, increase flowrate_limit.\n"
            )
        vmax = min(self.flowrate_limit, self.maxflowrate_calibrated)
        value = VolumeType2(value, vmax=vmax)
        return self.send_request(SetFlowrate, value)

    @property
    def dispense_volume(self):  # 3.15
        return self.send_request(GetDispenseVolume)

    @dispense_volume.setter
    def dispense_volume(self, value):  # 3.16
        return self.send_request(SetDispenseVolume, value)

    @property
    def dispense_period(self):  # 3.17
        return self.send_request(GetDispensePeriod)

    @dispense_period.setter
    def dispense_period(self, value):  # 3.18
        return self.send_request(SetDispensePeriod, value)

    @property
    def cycle_pause_time(self):  # 3.19
        return self.send_request(GetCyclePauseTime)

    @cycle_pause_time.setter
    def cycle_pause_time(self, value):  # 3.20
        return self.send_request(SetCyclePauseTime, value)

    @property
    def cycle_count(self):  # 3.21
        return self.send_request(GetCycleCount)

    @cycle_count.setter
    def cycle_count(self, value):  # 3.22
        return self.send_request(SetCycleCount, value)

    @property
    def maxflowrate(self):  # 3.23
        response = self.send_request(GetMaxFlowrate)
        return float(response[0])

    @property
    def maxflowrate_calibrated(self):  # 3.24
        response = self.send_request(GetMaxFlowrateCalibrated)
        return float(response[0])

    def flowrate2dsptime(self, volume, flowrate):  # 3.25
        return self.send_request(GetDispenseTimeByFlowrate, volume, flowrate)

    def rpm2dsptime(self, volume, rpm):  # 3.26
        return self.send_request(GetDispenseTimeByRPM, volume, rpm)

    # 4.0 CONFIGURATION
    @property
    def tube_diameter(self):  # 4.1
        return self.send_request(GetTubeDiameter)

    @tube_diameter.setter
    def tube_diameter(self, value):  # 4.2
        return self.send_request(SetTubeDiameter, value)

    @property
    def backsteps(self):  # 4.3
        return self.send_request(GetBacksteps)

    @backsteps.setter
    def backsteps(self, value):  # 4.4
        return self.send_request(SetBacksteps, value)

    def reset_config(self):  # 4.5
        return self.send_request(ResetConfigData)

    # 5.0 CALIBRATION
    @property
    def cal_direction(self):  # 5.1
        return self.send_request(GetCalDirection)

    @cal_direction.setter
    def cal_direction(self, value):  # 5.2
        return self.send_request(SetCalDirection, value)

    @property
    def cal_targetvol(self):  # 5.3
        return self.send_request(GetCalTargetVolume)

    @cal_targetvol.setter
    def cal_targetvol(self, value):  # 5.4
        return self.send_request(SetCalTargetVolume, value)

    @property
    def cal_actualvol(self):
        return getattr(self, "_cal_actualvol", None)

    @cal_actualvol.setter
    def cal_actualvol(self, value):  # 5.5
        response = self.send_request(SetCalActualVolume, value)
        self._cal_actualvol = SetCalActualVolume._types[0](value)
        return response

    @property
    def cal_duration(self):  # 5.6
        return self.send_request(GetCalDuration)

    @cal_duration.setter
    def cal_duration(self, value):  # 5.7
        return self.send_request(SetCalDuration, value)

    @property
    def runtime_since_cal(self):  # 5.8
        return self.send_request(GetRuntimeSinceCal)

    def start_calibration(self):  # 5.9
        return self.send_request(StartCalibration)

    def cancel_calibration(self):  # 5.10
        return self.send_request(CancelCalibration)

    def calibrate(self, target=None, duration=None, direction=None):  # 5.2–10
        if target is not None:
            self.cal_targetvol = target
        if duration is not None:
            self.cal_duration = duration
        if direction is not None:
            self.cal_direction = direction

        target = float(self.cal_targetvol[0])
        duration = int(self.cal_duration[0])
        direction = self.direction[0]

        proceed = input(
            f"Calibration requested for channel {self.addr!s} with:\n"
            f"– Target Volume = {target} mL\n"
            f"– Duration      = {duration*0.1} s\n"
            f"– Direction     = {direction!r}\n\n"
            "Do you wish to proceed?  y/[n]  "
        )
        if proceed.lower() != "y":
            return None

        response = self.send_request(StartCalibration)

        if isinstance(response, SuccessResponse):
            n = duration + 50
            timer_complete = timer(
                n,
                0.1,
                description=f"Calibrating channel {self.addr}...",
                interrupt_message="calibration cancelled by user",
            )
            if not timer_complete:
                return self.cancel_calibration()

            actual = input(
                f"Target volume was {target} mL, "
                "please measure and enter actual volume or mass (grams): "
            )
            self.cal_actualvol = float(actual)
        return response

    # 6.0 SYSTEM
    @property
    def firmware_version(self):  # 6.1
        return self.send_request(GetFirmwareVersion)

    def override_factory_rollervol(self, n, idx_tube, vol):  # 6.2
        return self.send_request(SetFactoryRollerVolume, n, idx_tube, vol)

    def save_rollerstep_settings(self):  # 6.3
        return self.send_request(SaveRollerStepSetting)

    def restore_factory_rollervol(self):  # 6.4
        return self.send_request(RestoreFactoryRollerVolume)

    @property
    def display_name(self):
        return getattr(self, "_display_name", None)

    @display_name.setter
    def display_name(self, value):  # 6.5
        response = self.send_request(SetDisplayName, value)
        self._display_name = SetDisplayName._types[0](value)
        return response

    @property
    def serial_number(self):  # 6.6
        return self.send_request(GetPumpSerialNo)

    @serial_number.setter
    def serial_number(self, value):  # 6.7
        return self.send_request(SetPumpSerialNo, value)

    @property
    def language(self):  # 6.8
        return self.send_request(GetPumpLanguage)

    @language.setter
    def language(self, value):  # 6.9
        return self.send_request(SetPumpLanguage, value)

    @property
    def num_channels(self):  # 6.10
        return self.send_request(GetNumberOfChannels)

    @num_channels.setter
    def num_channels(self, value):  # 6.11
        return self.send_request(GetNumberOfChannels, value)

    @property
    def num_rollers(self):  # 6.12
        return self.send_request(GetNumberOfRollers)

    @num_rollers.setter
    def num_rollers(self, value):  # 6.13
        return self.send_request(SetNumberOfRollers, value)

    @property
    def revs_since_reset(self):  # 6.14
        return self.send_request(GetRevsSinceReset)

    @property
    def vol_since_reset(self):  # 6.15
        return self.send_request(GetVolumeSinceReset)

    @property
    def pumptime_since_reset(self):  # 6.16
        return self.send_request(GetTimePumpedSinceReset)

    def enable_gui(self):  # 6.17
        return self.send_request(EnableGUI)

    def disable_gui(self):  # 6.18
        return self.send_request(DisableGUI)

    @property
    def display_nums(self):
        return getattr(self, "_display_nums", None)

    @display_nums.setter
    def display_nums(self, value):  # 6.19
        response = self.send_request(SetDisplayNumbers, value)
        self._display_nums = SetDisplayNumbers._types[0](value)
        return response

    @property
    def display_chars(self):
        return getattr(self, "_display_chars", None)

    @display_chars.setter
    def display_chars(self, value):  # 6.20
        response = self.send_request(SetDisplayCharacters, value)
        self._display_chars = SetDisplayCharacters._types[0](value)
        return response

    @property
    def run_status(self):  # 6.21
        return self.send_request(GetRunStatus)

    @property
    def model_info(self):  # 6.22
        return self.send_request(GetModelInfo)

    @property
    def pumphead_model(self):  # 6.23
        return self.send_request(GetPumpHeadModel)

    @pumphead_model.setter
    def pumphead_model(self):  # 6.24
        return self.send_request(SetPumpHeadModel)

    @property
    def pumptime(self):  # 6.25
        return self.send_request(GetDispenseTimeTenths)

    @pumptime.setter
    def pumptime(self, value: int | tuple[int, str | None]):  # 6.26 – 6.28
        return self._set_time_in_units(
            SetDispenseTimeTenths,
            SetDispenseTimeMins,
            SetDispenseTimeHrs,
            value,
        )

    @property
    def loworder_steps(self):  # 6.29
        return self.send_request(GetLowOrderSteps)

    @loworder_steps.setter
    def loworder_steps(self, value):  # 6.30
        return self.send_request(SetLowOrderSteps, value)

    @property
    def highorder_steps(self):  # 6.31
        return self.send_request(GetHighOrderSteps)

    @highorder_steps.setter
    def highorder_steps(self, value):  # 6.32
        return self.send_request(SetHighOrderSteps, value)

    @property
    def rollervol(self):  # 6.33
        return self.send_request(GetCalRollerVolume)

    @rollervol.setter
    def rollervol(self, value):  # 6.34
        return self.send_request(SetCalRollerVolume, value)

    def reset_calibration(self):  # 6.35
        return self.send_request(ResetCalibrationData)

    @property
    def pausetime(self):  # 6.36
        return self.send_request(GetCyclePauseTimeTenths)

    @pausetime.setter
    def pausetime(self, value: int | tuple[int, str | None]):  # 6.37 – 6.39
        return self._set_time_in_units(
            SetCyclePauseTimeTenths,
            SetCyclePauseTimeMins,
            SetCyclePauseTimeHrs,
            value,
        )

    @property
    def totalvol_since_reset(self):  # 6.40
        return self.send_request(GetTotalVolumeSinceReset)

    def save_pump_settings(self):  # 6.41
        return self.send_request(SavePumpSettings)

    @property
    def footswitch_status(self):  # 6.42
        return self.send_request(GetFootSwitchStatus)

    # HELPER
    def _set_time_in_units(self, settenths, setmins, sethrs, value):
        value, unit = value if isinstance(value, tuple) else (value, None)
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
        return None

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
        return None

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
