# A ROS 2 Driver to control Ortery turntables

This package wraps the `OTADCommand.exe` utility for controlling Ortery turntables (e.g. Ortery PhotoCapture 360) and can be used under ROS 2 for Windows on the machine connected to the turntable, or on a remote machine connecting via SSH.
The file `ortery_driver.py` can also be used stand-alone in case you want to control the turntable directly via Python.

We don't have access to any other turntable except for the Ortery PhotoCapture 360, hence we only tested it on that specific model. Feel free to contribute if you want to add specific functionality for other models.

To use it, make sure that the folder containing `OTADCommand.exe` is added to the `PATH` environment variable on the machine connected to the turntable.

Currently implemented ROS 2 services:
* get\_device\_count: Returns the number of devices.
* get\_device\_info (int device\_i): Returns the information for device _i_ (`device_i` is enumerated from 0)
* get\_command\_desc (int device\_i): Returns the list of commands supported by this device.
* get\_property\_desc (int device\_i): Returns the list of properties supported by this device.
* set\_property\_data (int device\_i, int property\_id, int data): Sets property with the specified id on the specified device to `data`.
* set\_properties\_data (int device\_i, int[] properties, int data): Sets the value of all specified properties on the specified device to `data`.
* send\_command (int device\_i, int command\_id): Sends the specified command to the device.
* turntable\_stop (int device\_i): Convenience function to stop the turntable when it is turning.

The following ROS 2 action servers are implemented:
* turntable (int device\_i, int speed, int direction, int step): Rotate the turntable. Speeds can be 0 (low), 1 (normal) or 2 (high). Direction can be 0 (clockwise) or 1 (counter clockwise). Steps can be 0 (continuous) or a value up to 665535. To convert degrees to steps use the formula `degrees * total steps / 360`.
* turntable\_degrees (int device\_i, int speed, int direction, int degrees): Convenience function to rotate by degrees. See `turntable`. To avoid rounding errors, use a multiple of 22.5 degrees (1/16 of a full rotation).

To use a remote PC connected via SSH to the machine controlling the turntable, use the following parameters:
* use\_ssh (type: boolean, default: false): Enable the usage of SSH.
* ssh\_host (type: string, default: ""): Host name (or IP address) of the machine to which the turntable is connected.
* ssh\_user (type: string, default: ""): User name for the SSH connection.
* ssh\_password (type: string, default: ""): Password for the SSH connection.

To be able to connect to a Windows machine using SSH, ensure that the OpenSSH Server has been enabled on the Windows machine.


See the documentation for `OTADCommand.exe` for more information about commands, properties and statuses.
