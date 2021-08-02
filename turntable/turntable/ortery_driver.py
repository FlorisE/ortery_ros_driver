import re
import subprocess
from collections import namedtuple


def rwo(command, debug=False):
    """Wrap subprocess.run to always capture output."""
    proc = subprocess.run(command, capture_output=True)
    return proc.stdout.decode("utf-8")


def get_device_count(debug=False):
    """Get the number of devices connected to this PC."""
    output = rwo("OTADCommand.exe get_device_count", debug)
    m = re.search('^([0-9]+)\\r\\n$', output)
    return m.group(1)


class InvalidIdException(Exception):
    """Exception thrown when an invalid ID was passed, or device is offline."""
    def __init__(self, device_id):
        Exception.__init__(self)
        self.device_id = device_id


class DeviceInfo():
    """Information of a device."""
    def __init__(self, product_name, device_id):
        self.product_name = product_name
        self.device_id = device_id


def get_device_info(device_id, debug=False):
    output = rwo(f"OTADCommand.exe get_device_info {device_id}", debug)
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


def get_command_desc(device_id, debug=False):
    """Get a list of commands supported by the device."""
    output = rwo(f"OTADCommand.exe get_command_desc {device_id}", debug)
    e = 'get_command_desc :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e:
        raise InvalidIdException(device_id)

    command_ids = re.findall("([0-9]+)\r\n", output)
    return [command_dict.get(int(command_id), UnknownCommand()) 
            for command_id in command_ids]


Property = namedtuple("Property", "name value description")
property_dict = {
    16641: Property("otadDEVICE_PROPERTY_TURNTABLE_STATE",
                    16641,
                    "State of turntable"),
    16643: Property("otadDEVICE_PROPERTY_TURNTABLE_TOTAL_STEPS",
                    16643,
                    "Total step of turntable")
    }


class UnknownProperty():
    pass


def get_property_desc(device_id, debug=False):
    """Get a list of properties that can be read or set by the device."""
    output = rwo(f"OTADCommand.exe get_property_desc {device_id}", debug)
    e = 'get_property_desc :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e:
        raise InvalidIdException(device_id)

    property_ids = re.findall("([0-9]+)\r\n", output)
    return [property_dict.get(int(property_id), UnknownProperty()) 
            for property_id in property_ids]


def get_property_data(device_i, property_id, debug=False):
    """Get the data for a specified property."""
    cmd = f"OTADCommand.exe get_property_data {device_i} {property_id}"
    output = rwo(cmd, debug)
    e = 'get_property_data :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e:
        raise InvalidIdException(device_id)
    m = re.search("^([0-9]+)", output)
    return m.group(1)


class SetPropertyValueUnsupportedException(Exception):
    """Exception for when setting this property is not supported."""
    def __init__(self, device_i, property_id, data):
        Exception.__init__(self)
        self.device_i = device_i
        self.property_id = property_id
        self.data = data


class SetPropertiesValueUnsupportedException(Exception):
    """Exception for when setting this property is not supported."""
    def __init__(self, device_i, properties, data):
        Exception.__init__(self)
        self.device_i = device_i
        self.properties = properties
        self.data = data


class DeviceNotSupportPropertyException(Exception):
    """Exception for when setting this property is not supported."""
    def __init__(self, device_i, property_id, data):
        Exception.__init__(self)
        self.device_i = device_i
        self.property_id = property_id
        self.data = data


class DeviceNotSupportPropertiesException(Exception):
    """Exception for when setting this property is not supported."""
    def __init__(self, device_i, properties, data):
        Exception.__init__(self)
        self.device_i = device_i
        self.properties = properties
        self.data = data


def set_property_data(device_i, property_id, data, debug=False):
    """Sets the data for a specified property."""
    if device_i is None:
        raise ValueError("device_i")
    if property_id is None:
        raise ValueError("property_id")
    if data is None:
        raise ValueError("data")
    cmd = f"OTADCommand.exe set_property_data {device_i} {property_id} {data}"
    output = rwo(cmd, debug)
    e1 = 'set_property_data :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e1:
        raise InvalidIdException(device_i)
    e2 = 'set_property_data :  command exec fail ( error code : 0x004000a)\r\n'
    if output == e2:
        raise SetPropertyValueUnsupportedException(device_i, property_id, data)
    e2 = 'set_property_data :  command exec fail ( error code : 0x0040005)\r\n'
    if output == e2:
        raise DeviceNotSupportPropertyException(device_i, property_id, data)
    return True


def set_properties_data(device_i, properties, data, debug=False):
    if type(device_i) is not int:
        raise ValueError("device_i needs to be an int")
    if type(properties) is not list:
        raise ValueError("device_i should be a list")
    if len(properties) == 0:
        raise ValueError("At least one property should be specified")
    if len(properties) > 20:
        raise ValueError("Maximum of 20 properties can be set at a time")
    cmd_builder = f"OTADCommand.exe set_properties_data {device_i} {data}"
    for property in properties:
        cmd_builder += f" {property}"
    print(cmd_builder)
    output = rwo(cmd_builder, debug)
    print(output)
    e = 'set_properties_data :  command exec fail ( error code : 0x0040001)\r\n'
    if output == e:
        raise InvalidIdException(device_i)
    e = 'set_properties_data :  command exec fail ( error code : 0x004000a)\r\n'
    if output == e:
        raise SetPropertiesValueUnsupportedException(device_i, properties, data)
    e = 'set_properties_data :  command exec fail ( error code : 0x0040005)\r\n'
    if output == e:
        raise DeviceNotSupportPropertiesException(device_i, properties, data)
    return True
