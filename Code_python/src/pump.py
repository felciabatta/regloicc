"""Reglo ICC Peristaltic Pump Control Library
"""

import time

import serial


#---------------------------------------------------------------------------------#
# LIBRARY MANAGING THE COMMUNICATION WITH A REGLO ICC WITH 3 INDEPENDENT CHANNELS #
#---------------------------------------------------------------------------------#


def velocity_converter_mu(speed, time='hour'):
    """
    Convert flow rate in μL/min to pump-compatible mmmmse format.
    Returns (mmmm, '-', exponent).
    """
    # Convert time to minutes if needed
    time_factors = {
        'min': 1,
        'hour': 60
    }

    if time not in time_factors:
        raise ValueError("Invalid time unit. Use 'min' or 'hour'.")

    flow_uL_min = speed / time_factors[time]

    for exponent in range(1, 4):
        scaled = flow_uL_min * 10**exponent
        if abs(scaled - round(scaled)) < 1e-6:
            return int(round(scaled)), '-', exponent

    # Default to exponent 3
    return int(round(flow_uL_min * 1000)), '-', 3

def velocity_converter(speed, volume='muL', time='hour'):
    """
    Convert flow rate to pump-compatible mmmmse format (flow_int, exponent).
    Chooses the smallest exponent (1–3) that keeps 1–3 decimal digits.
    """
    print('heiiiii')
    # Time conversion to minutes
    time_factors = {
        'min': 1,
        'hour': 60
    }

    # Volume conversion to mL
    volume_factors = {
        'muL': 1/1000,
        'mL': 1
    }

    e = {'muL':3,
         'mL': 1}
    mult =  {'muL':1000,
         'mL': 1}

    if time not in time_factors:
        raise ValueError("Invalid time unit. Use , 'min', or 'hour'.")
    if volume not in volume_factors:
        raise ValueError("Invalid volume unit. Use 'muL' or 'mL'.")

    # Convert to mL/min
    flow_mL_min = speed * volume_factors[volume] / time_factors[time]

    print(flow_mL_min)
    # If nothing fits exactly, round to 3 decimals (e=3)
    return int(round(flow_mL_min * mult[volume])), e[volume]

class RegloICC:
    # Initialize the pump
    def __init__(self, COM):
        self.COM = COM
        # Open the serial port with the data format corresponding to the RegloICC pump
        self.sp = serial.Serial(self.COM, 9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.direction = [0, 0, 0]  # 0 = clockwise, 1 = counter-clockwise
        self.mode = [0, 0, 0]  # 0 = RPM, 1 = Flow Rate, 2 = Volume (over time), one can add here all other modes
        self.speed = [0, 0, 0]  # rotation speed for each channel in RPM mode
        # Change the size of 'direction', 'mode' and 'speed' according to the total number of channels to control.
        # In the case herein, 3 channels are being independently controlled

    # Delete the pump
    def __del__(self):
        self.sp.close()

    # Start the corresponding channel
    def start_channel(self, channel):
        command = f'{channel}H\r'.encode() # 'H' to start the channel
        # \r for the carriage return [CR] required to tell the pump that the command is finished
        self.sp.write(command) # write the command to the pump
        time.sleep(0.1) # give the pump time to process the command after sending it before reading the response
        return self.sp.read(self.sp.in_waiting).decode() # read the pump response

    # Stop the corresponding channel
    def stop_channel(self, channel):
        command = f'{channel}I\r'.encode() # 'I' to stop the channel
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    # Set the rotation direction for a single channel
    def set_direction(self, channel, direction):
        if direction == 1:
            command = f'{channel}K\r'.encode() # counter-clockwise
        else:
            command = f'{channel}J\r'.encode() # clockwise
        self.sp.write(command)
        self.direction[channel - 1] = direction # pyhton count starts from 0
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    # Get the rotation direction of a single channel
    def get_direction(self, channel):
        command = f'{channel}xD\r'.encode() # 'xD' to get the rotation direction
        self.sp.write(command)
        time.sleep(0.1)
        # read the rotation direction from the corresponding channel
        return self.sp.read(self.sp.in_waiting).decode()

    # Set the speed for a single channel in RPM when in RPM mode
    def set_speed(self, channel, speed): # in RPM, with speed  Result: '123'                       #
    #                                                                                #
    ## Convert the decimal part to a string with two digits :                        #
       fixed_speed = int(round(speed * 100))             #
       cmd = f'{channel}S{fixed_speed:06d}\r'.encode()

       self.sp.write(cmd)

    def set_speed_volume_time(self,channel,speed,s='-',e=3):
        #The speed is given in ml/min - hence we convert in muL.
        #Speed given in 4 digit mmmmse converted as m.mmm
        cmd =f'{channel}f{speed:04d}{s}{e}\r'.encode()
        print(cmd)

        self.sp.write(cmd)

    ## When the two strings are concatenated, the result is a string that represents #
    ## the original speed value in a fixed-point notation with three digits before   #
    ## the decimal point and two digits after, without the decimal separator.        #
    #--------------------------------------------------------------------------------#

    # Read out speed of a single channel in RPM when in RPM mode
    def get_speed(self, channel):
        command = f'{channel}S\r'.encode() # 'S' to get the setting speed in RPM
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def get_speed(self, channel):
        command = f'{channel}S\r'.encode() # 'S' to get the setting speed in RPM
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    def get_flowrate(self, channel):
        command = f'{channel}f\r'.encode() #
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    # Set the operational mode for a single channel (you can add all other modes)
    def set_mode(self, channel, mode):
        if mode == 0:
            command = f'{channel}L\r'.encode()  # RPM mode
        elif mode == 1:
            command = f'{channel}M\r'.encode()  # Flow rate mode
        else:
            command = f'{channel}G\r'.encode()  # Volume (over time) mode
        self.sp.write(command)
        self.mode[channel - 1] = mode
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

    # Get the operational mode of a single channel
    def get_mode(self, channel):
        command = f'{channel}xM\r'.encode() # 'xM' to get the operational mode
        self.sp.write(command)
        time.sleep(0.1)
        return self.sp.read(self.sp.in_waiting).decode()

#----------------------------------------------------------------------------#
#   EXAMPLES ON HOW TO USE THE DEFINED CLASS TO CONTROL THE Reglo ICC PUMP   #
#----------------------------------------------------------------------------#

# ### Initialize the pump with the specified COM port
#pump = RegloICC('COM3') # Replace 'COM5' with your actual COM port

# ### Start channel 3

#pump.start_channel(channel)
# v,s,e =velocity_converter_mu(50,time='min')
# pump.set_direction(1, 0)
# pump.set_mode(1, 1)
# pump.set_speed_volume_time(1,v ,s, e)
# print(pump.get_flowrate(1))
# pump.start_channel(1)
# time.sleep(10)
# # ### Stop channel 3
# pump.stop_channel(1)
# print('finito')

# del pump

# print('end')
