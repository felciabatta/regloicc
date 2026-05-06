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
pump.set_speed_volume_time(1,v ,s, e)
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

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial


class RegloICC:
    def __init__(self, COM):
        """Initialize the pump and set up the serial connection."""

        self.COM = COM
        # Open serial port with data format corresponding to the RegloICC pump
        self.sp = Serial(
            self.COM,
            9600,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            bytesize=EIGHTBITS,
        )

        self.direction = [0, 0, 0]
        # 0 = clockwise, 1 = counter-clockwise

        self.mode = [0, 0, 0]
        # 0 = RPM, 1 = Flow Rate, 2 = Volume (over time)
        # one can add here all other modes

        self.speed = [0, 0, 0]  # rotation speed for each channel in RPM mode
        # Change the size of 'direction', 'mode' and 'speed'
        # according to the total number of channels to control.
        # In the case herein, 3 channels are being independently controlled

    def __del__(self):
        """Delete the pump"""

        self.sp.close()

    def start_channel(self, channel):
        """Start the corresponding channel"""

        # 'H' to start the channel
        # \r for the carriage return [CR] required
        # to tell the pump that the command is finished
        command = f"{channel}H\r".encode()
        self.sp.write(command)  # send to serial
        time.sleep(0.1)  # time to process
        # read the pump response
        return self.sp.read(self.sp.in_waiting).decode()

    def stop_channel(self, channel):
        """Stop the corresponding channel"""

        # 'I' to stop the channel
        command = f"{channel}I\r".encode()
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def set_direction(self, channel, direction):
        """Set the rotation direction for a single channel"""

        if direction == 1:
            command = f"{channel}K\r".encode()  # counter-clockwise
        else:
            command = f"{channel}J\r".encode()  # clockwise
        self.sp.write(command)
        # python count starts from 0
        self.direction[channel - 1] = direction
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def get_direction(self, channel):
        """Get the rotation direction of a single channel"""

        # 'xD' to get the rotation direction
        command = f"{channel}xD\r".encode()
        self.sp.write(command)
        time.sleep(0.1)
        # read the rotation direction from the corresponding channel
        return self.sp.read(self.sp.in_waiting).decode()

    def set_speed(self, channel, speed):
        """Set the speed for a single channel in RPM when in RPM mode"""

        # in RPM, with speed  Result: '123'
        # Convert the decimal part to a string with two digits
        fixed_speed = int(round(speed * 100))
        cmd = f"{channel}S{fixed_speed:06d}\r".encode()

        self.sp.write(cmd)
        self.speed[channel - 1] = speed

    def set_speed_volume_time(self, channel, speed: int, s="-", e=3):
        r"""Set flow rate in mL/min

        RegloICC uses the format string `mmmmse` to encode the flow rate,
        which translates to :math:`m.mmm \times 10^{se} mL/min`.

        :type channel: int
        :param channel:
        :type speed: int
        :param speed:
            Four-digit integer `mmmm`.
            Represents the 4 sf of the floating point number `m.mmm`.
        :type s: str
        :param s:
            Sign of exponent `"+"` or `"-"`.
        :type e" int
        :param e:
            Exponent.
        """
        cmd = f"{channel}f{speed:04d}{s}{e}\r".encode()
        print(cmd)

        self.sp.write(cmd)
        self.speed[channel - 1] = speed

        # --------------------------------------------------------------------
        # When the two strings are concatenated,
        # the result is a string that represents
        # the original speed value in a fixed-point notation
        # with three digits before the decimal point and two digits after,
        # without the decimal separator.
        # --------------------------------------------------------------------

    def get_speed(self, channel):
        """Read out speed of a single channel in RPM when in RPM mode"""

        # 'S' to get the setting speed in RPM
        command = f"{channel}S\r".encode()
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def get_flowrate(self, channel):
        command = f"{channel}f\r".encode()
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def set_mode(self, channel, mode):
        """Set the operational mode for a single channel
        (you can add all other modes)
        """
        if mode == 0:
            command = f"{channel}L\r".encode()  # RPM mode
        elif mode == 1:
            command = f"{channel}M\r".encode()  # Flow rate mode
        else:
            command = f"{channel}G\r".encode()  # Volume (over time) mode
        self.sp.write(command)
        self.mode[channel - 1] = mode
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def get_mode(self, channel):
        """Get the operational mode of a single channel"""
        command = f"{channel}xM\r".encode()  # 'xM' to get the operational mode
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()


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
    print("heiiiii")
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
