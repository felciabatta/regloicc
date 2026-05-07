"""Reglo ICC Peristaltic Pump Control Library

For managing the communication with a reglo icc with 3 independent channels

Example
-------

```python
# Initialize the pump with the specified COM port
pump = RegloICC('COM3') # Replace 'COM5' with your actual COM port

# Start channel 3

pump.start_channel(channel)
v, s, e = velocity_converter_mu(50,time='min')
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
from serial.tools import list_ports


def get_port(com=None):
    if com is None:
        ports = list_ports.comports()
        if not ports:
            raise ValueError("No COM ports found. Please connect the pump.")

        print("Available COM ports:")
        for p, port in enumerate(ports):
            print(f"{p}  {port.device}: {port.description}")
        print("")

        p = int(input("COM port not specified. Please select a port: "))
        return ports[p].device
    elif isinstance(com, str):
        return com
    else:
        raise ValueError("Invalid COM port spec. Use None or string.")


class RegloICC:
    def __init__(
        self, com=None, direction=[0, 0, 0], mode=[0, 0, 0], speed=[0, 0, 0]
    ):
        """Initialize the pump and set up the serial connection."""

        # Open serial port with data format corresponding to the RegloICC pump
        self.com = get_port(com)
        self.serial = Serial(
            self.com,
            9600,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            bytesize=EIGHTBITS,
        )
        print(f"Connected to pump on {self.com}")

        self.direction = direction
        # 0 = clockwise, 1 = counter-clockwise

        self.mode = mode
        # 0 = RPM, 1 = Flow Rate, 2 = Volume (over time)
        # one can add here all other modes

        self.speed = speed  # rotation speed for each channel in RPM mode
        # Change the size of 'direction', 'mode' and 'speed'
        # according to the total number of channels to control.
        # In the case herein, 3 channels are being independently controlled

    def __del__(self):
        """Delete the pump"""

        self.serial.close()

    def write_receive(self, command):
        """Write a command to the pump and read the response."""

        self.serial.write(command.encode())  # send to serial
        time.sleep(0.1)  # time to process
        return self.serial.read(self.serial.in_waiting).decode()  # response

    def fwrite_receive(self, channel, inner_command):
        """Format & write a command to the pump and read the response."""

        # \r for the carriage return [CR] required
        # to tell the pump that the command is finished
        return self.write_receive(f"{channel}{inner_command}\r")

    def start_channel(self, channel):
        """Start the corresponding channel"""

        # 'H' to start the channel
        return self.fwrite_receive(channel, "H")

    def stop_channel(self, channel):
        """Stop the corresponding channel"""

        # 'I' to stop the channel
        return self.fwrite_receive(channel, "I")

    def set_direction(self, channel, direction):
        """Set the rotation direction for a single channel"""

        self.direction[channel - 1] = direction
        # K: counter-clockwise, J: clockwise
        d = {0: "J", 1: "K"}.get(direction, "J")
        return self.fwrite_receive(channel, d)

    def get_direction(self, channel):
        """Get the rotation direction of a single channel"""

        # 'xD' to get the rotation direction
        return self.fwrite_receive(channel, "xD")

    def set_rpm(self, channel, rpm):
        """Set the speed for a single channel in RPM when in RPM mode"""

        self.speed[channel - 1] = rpm

        # in RPM, with speed  Result: '123'
        # Convert the decimal part to a string with two digits
        fixed_speed = int(round(rpm * 100))
        return self.fwrite_receive(channel, f"S{fixed_speed:06d}")

    def get_rpm(self, channel):
        """Read out speed of a single channel in RPM when in RPM mode"""

        # 'S' to get the setting speed in RPM
        return self.fwrite_receive(channel, "S")

    def set_flowrate(self, channel, flowrate: int, s="-", e=3):
        r"""Set flow rate in mL/min

        RegloICC uses the format string `mmmmse` to encode the flow rate,
        which translates to :math:`m.mmm \times 10^{se} mL/min`.

        :type channel: int
        :param channel:
        :type flowrate: int
        :param flowrate:
            Four-digit integer `mmmm`.
            Represents the 4 sf of the floating point number `m.mmm`.
        :type s: str
        :param s:
            Sign of exponent `"+"` or `"-"`.
        :type e: int
        :param e:
            Exponent.

        Note
        ----
        When the two strings are concatenated,
        the result is a string that represents
        the original speed value in a fixed-point notation
        with three digits before the decimal point and two digits after,
        without the decimal separator.
        """

        self.speed[channel - 1] = flowrate

        cmd = f"f{flowrate:04d}{s}{e}"
        print(cmd)
        return self.fwrite_receive(channel, cmd)

    def get_flowrate(self, channel):
        return self.fwrite_receive(channel, "f")

    def set_mode(self, channel, mode):
        """Set the operational mode for a single channel
        (you can add all other modes)
        """
        self.mode[channel - 1] = mode

        # L = rpm, M = flowrate, G = volume (over time)
        m = {0: "L", 1: "M", 2: "G"}.get(mode, "L")
        return self.fwrite_receive(channel, m)

    def get_mode(self, channel):
        """Get the operational mode of a single channel"""

        # 'xM' to get the operational mode
        return self.fwrite_receive(channel, "xM")


def flux2mmmmse(flowrate, from_unit="mL/min"):
    """Convert flowrate (flux) from mL/min to mmmmse format.

    :param flowrate:
    :param from_unit:

    :rtype: tuple[int, str, int]
    :returns:
        Flow rate tuple `(mmmm, "-", exponent)`.
    """

    v_unit, t_unit = from_unit.split("/", 1)
    time_factors = {"s": 1 / 60, "min": 1, "hr": 60}
    vol_factors = {"uL": 1e-3, "mL": 1}
    if t_unit not in time_factors:
        raise ValueError("Invalid time unit. Use 's' min' or 'hr'.")
    if v_unit not in vol_factors:
        raise ValueError("Invalid volume unit. Use 'uL' or 'mL'.")
    flow_mL_min = flowrate * vol_factors[v_unit] / time_factors[t_unit]

    s = sign(flow_mL_min - 1)
    e = 0
    flow_scaled = flow_mL_min * 10 ** (-s * e)
    while not (1 <= flow_scaled < 10):
        e += 1
        flow_scaled = flow_mL_min * 10 ** (-s * e)
    flow_scaled *= 1000
    mmmm = int(min(round(flow_scaled), 9999))

    return (mmmm, "-" if s == -1 else "+", e)


def velocity_converter_mu(speed: float, unit="hour"):
    """Convert flow rate from μL/unit to mL/min in `mmmmse` format.

    :param speed:
    :param unit:

    :rtype: tuple[int, str, int]
    :returns:
        Flow rate tuple `(mmmm, "-", exponent)`.
    """

    # Convert to minutes if needed
    minutes_per = {"min": 1, "hour": 60}
    if unit not in minutes_per:
        raise ValueError("Invalid time unit. Use 'min' or 'hour'.")

    flow_uL_min = speed / minutes_per[unit]

    for exponent in range(1, 4):
        scaled = flow_uL_min * 10**exponent
        if abs(scaled - round(scaled)) < 1e-6:
            return int(round(scaled)), "-", exponent

    # Default to exponent 3
    return int(round(flow_uL_min * 1000)), "-", 3


def velocity_converter(speed, volume="muL", time="hour"):  # UNUSED
    """
    Convert flow rate to pump-compatible mmmmse format (flow_int, exponent).
    Chooses the smallest exponent (1–3) that keeps 1–3 decimal digits.
    """

    # Time conversion to minutes
    time_units = {"min": 1, "hour": 60}

    # Volume conversion to mL
    volume_factors = {"muL": 1 / 1000, "mL": 1}

    e = {"muL": 3, "mL": 1}
    mult = {"muL": 1000, "mL": 1}

    if time not in time_units:
        raise ValueError("Invalid time unit. Use , 'min', or 'hour'.")
    if volume not in volume_factors:
        raise ValueError("Invalid volume unit. Use 'muL' or 'mL'.")

    # Convert to mL/min
    flow_mL_min = speed * volume_factors[volume] / time_units[time]

    print(flow_mL_min)
    # If nothing fits exactly, round to 3 decimals (e=3)
    return int(round(flow_mL_min * mult[volume])), e[volume]
