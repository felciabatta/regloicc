"""Reglo ICC Peristaltic Pump Control Library.

For managing the communication with a reglo icc with 3 independent channels.

Example
-------

```python
# Initialize the pump with the specified COM port
pump = RegloICC('COM3') # Replace 'COM5' with your actual COM port

# Start channel 3

pump.start_channel(channel)
v, s, e = microflux2mmmmse(50,unit='min')
pump.set_direction(1, 0)
pump.set_mode(1, 1)
pump.set_flowrate(1,v ,s, e)
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

from numpy import sign
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial

from .utils import get_port


class RegloICC:
    """RegloICC pump control.

    Change the size of 'direction', 'mode' and 'speed'
    according to the total number of channels to control.
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
        response = self.serial.read(
            self.serial.in_waiting
        ).decode()  # response
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
            If specified, `flowrate` is converted from the given unit to mL/min.
            Format as `"vol/time"` where `vol = {mL, uL}`, `time = {hr, min, s}`.

        Note
        ----
        When the two strings are concatenated,
        the result is a string that represents
        the original speed value in a fixed-point notation
        with three digits before the decimal point and two digits after,
        without the decimal separator.
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


def microflux2mmmmse(flowrate: float, unit="hour"):
    """Convert flow rate from μL/unit to mL/min in `mmmmse` format.

    :param flowrate:
    :param unit:

    :rtype: tuple[int, str, int]
    :returns:
        Flow rate tuple `(mmmm, "-", exponent)`.
    """

    return flux2mmmmse(flowrate, from_unit=f"uL/{unit}")
