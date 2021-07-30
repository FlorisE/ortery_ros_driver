import re
import subprocess
from collections import namedtuple


def rwo(command):
    """Wrap subprocess.run to always capture output."""
    proc = subprocess.run(command, capture_output=True)
    return proc.stdout.decode("utf-8")

def get_device_count():
    """Get the number of devices connected to this PC."""
    output = rwo("OTADCommand.exe get_device_count")
    m = re.search('^([0-9]+)\\r\\n$', output)
    return m.group(1)


class InvalidIdException(Exception):
    """Exception thrown when an invalid ID was passed."""
    def __init__(self, device_id):
        Exception.__init__(self)
        self.device_id = device_id


class DeviceInfo():
    """Information of a device."""
    def __init__(self, product_name, device_id):
        self.product_name = product_name
        self.device_id = device_id


def get_device_info(device_id):
    output = rwo(f"OTADCommand.exe get_device_info {device_id}")
    e = 'get_device_info :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e:
        raise InvalidIdException(device_id)

    s = '^Product Name : ([A-Za-z0-9 ]+)\\r\\nDevice ID : ([0-9]+)\\r\\n'
    m = re.search(s, output)
    return DeviceInfo(m.group(1), m.group(2))


Command = namedtuple("Command", "name value description")
command_dict = {
    12801: Command("otadDEVICE_COMMAND_CABLERLEASE_OFF",
                   12801,
                   "Shutter Release OFF"),
    12802: Command("otadDEVICE_COMMAND_CABLERLEASE_HALFWAY",
                   12802,
                   "Shutter Release Halfway (Focus)"),
    12803: Command("otadDEVICE_COMMAND_CABLERLEASE_COMPLETELY",
                   12803,
                   "Shutter Release Completely (Snap)"),
    13057: Command("otadDEVICE_COMMAND_TURNTABLE_STOP",
                   13057,
                   "Stop the turntable"),
    13058: Command("otadDEVICE_COMMAND_TURNTABLE_RELEASE",
                   13058,
                   "Release the motor of turntable"),
    16641: Command("otadDEVICE_PROPERTY_TURNTABLE_STATE",
                   16641,
                   "State of turntable")
    }



class UnknownCommand():
    pass


def get_command_desc(device_id):
    """Get a list of commands supported by the device."""
    output = rwo(f"OTADCommand.exe get_command_desc {device_id}")
    e = 'get_command_desc :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e:
        raise InvalidIdException(device_id)

    command_ids = re.findall("([0-9]+)\r\n", output)
    return [command_dict.get(int(command_id), UnknownCommand()) 
            for command_id in command_ids]
