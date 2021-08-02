# A ROS 2 Driver to control Ortery turntables

This package wraps the `OTADCommand.exe` utility for controlling Ortery turntables (e.g. Ortery PhotoCapture 360) and is hence meant to be used in ROS 2 for Windows.
The file `ortery_driver.py` can also be used stand-alone in case you want to control the turntable directly via Python.

To use it make sure that the folder containing `OTADCommand.exe` is added to the `PATH` environment variable.

Currently implemented ROS 2 services:
* get_device_count: Returns the number of devices.
* get_device_info (int device_i): Returns the information for device _i_ (`device_i` is enumerated from 0)
* get_command_desc (int device_i): Returns the list of commands supported by this device.
* get_property_desc (int device_i): Returns the list of properties supported by this device.
* set_property_data (int device_i, int property_id, int data): Sets property with the specified id on the specified device to `data`.
* set_properties_data (int device_i, int[] properties, int data): Sets the value of all specified properties on the specified device to `data`.
* send_command (int device_i, int command_id): Sends the specified command to the device.
* turntable (int device_i, int speed, int direction, int step): Rotate the turntable. Speeds can be 0 (low), 1 (normal) or 2 (high). Direction can be 0 (clockwise) or 1 (counter clockwise). Steps can be 0 (continuous) or a value up to 665535. To convert degrees to steps use the formula `degrees * total steps / 360`.
* turntable_degrees (int device_i, int speed, int direction, int degrees): Convenience function to rotate by degrees. See `turntable`. To avoid rounding errors, use a multiple of 22.5 degrees (1/16 of a full rotation).
* turntable_stop (int device_i): Convenience function to stop the turntable when it is turning.

See the documentation for `OTADCommand.exe` for more information about commands, properties and statuses.
